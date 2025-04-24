from copy import deepcopy as copy

import numpy as np
from numpy.linalg import pinv
from scipy.spatial.transform import Rotation

from ruka_hand.control.operator import RUKAOperator
from ruka_hand.utils.constants import *
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import *
from ruka_hand.utils.zmq import ZMQPublisher, create_pull_socket


class OculusTeleoperator:
    def __init__(
        self,
        host,
        oculus_left_port,
        oculus_right_port,
        frequency,
        moving_average_limit=10,
        hands=["left", "right"],
    ):
        self.timer = FrequencyTimer(frequency)
        self.frequency = frequency

        self.keypoints_sockets = dict(
            left=create_pull_socket(host, oculus_left_port),
            right=create_pull_socket(host, oculus_right_port),
        )

        self.knuckle_points = (
            OCULUS_JOINTS["knuckles"][0],
            OCULUS_JOINTS["knuckles"][-2],
        )

        self.moving_average_limit = moving_average_limit
        self.coord_moving_average_queues, self.frame_moving_average_queues = {
            "left": [],
            "right": [],
        }, {"left": [], "right": []}

        self.hand_names = hands

    def _init_hands(self):
        self.hands = {}
        for hand_name in self.hand_names:
            self.hands[hand_name] = RUKAOperator(
                hand_type=hand_name,
                moving_average_limit=5,
            )

    # Extract VR keypoints and process it
    def _extract_data_from_token(self, token):
        data = self._process_data_token(token)
        information = dict()
        keypoint_vals = [0] if data.startswith("absolute") else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(":")[1].strip().split("|")
        for vector_str in vector_strings:
            vector_vals = vector_str.split(",")
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))

        information["keypoints"] = keypoint_vals
        return information

    # Convert Homogenous matrix to cartesian coords
    def _homo2cart(self, homo_mat):
        # Here we will use the resolution scale to set the translation resolution
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(homo_mat[:3, :3]).as_rotvec(degrees=False)

        cart = np.concatenate([t, R], axis=0)

        return cart

    # Process the data token
    def _process_data_token(self, data_token):
        return data_token.decode().strip()

    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[0]

    def _get_ordered_joints(self, projected_translated_coords):
        # Extract joint data based on HAND_JOINTS
        extracted_joints = {
            joint: projected_translated_coords[indices]
            for joint, indices in HAND_JOINTS.items()
        }
        # Concatenate the extracted joint data in the same order as the dictionary keys
        ordered_joints = np.concatenate(
            [extracted_joints[joint] * 100.0 for joint in HAND_JOINTS],
            axis=0,
        )
        reshaped_joints = ordered_joints.reshape(5, 5, 3)

        return reshaped_joints

    # Transform keypoints and right hand frame and arm frame
    def transform_keypoints(self, hand_coords, hand_name):
        translated_coords = self._translate_coords(hand_coords)

        hand_dir_frame = self._get_hand_dir_frame(
            hand_coords[0],
            translated_coords[self.knuckle_points[0]],
            translated_coords[self.knuckle_points[1]],
            hand_name,
        )

        # Get the ordered joints by dot producting the palm direction to with the
        # translated coordinates
        transformation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        rotation_matrix = np.array(hand_dir_frame[1:])  # This has (X,Y,Z) as ordered
        transformed_rotation_matrix = transformation_matrix @ rotation_matrix
        projected_translated_coords = (
            translated_coords @ transformed_rotation_matrix.T
        )  # This should have each hand coord projected on the hand frame

        ordered_joints = self._get_ordered_joints(projected_translated_coords)

        return ordered_joints, hand_dir_frame

    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord):

        # This function is used for retargeting hand keypoints from oculus. The frames here are in robot frame.
        palm_normal = normalize_vector(
            np.cross(index_knuckle_coord, pinky_knuckle_coord)
        )  # Current Z
        palm_direction = normalize_vector(
            index_knuckle_coord + pinky_knuckle_coord
        )  # Current Y
        cross_product = normalize_vector(
            np.cross(palm_direction, palm_normal)
        )  # Current X
        return [cross_product, palm_direction, palm_normal]

    def _get_hand_dir_frame(
        self, origin_coord, index_knuckle_coord, pinky_knuckle_coord, hand_name
    ):

        if hand_name == "left":
            palm_normal = normalize_vector(
                np.cross(index_knuckle_coord, pinky_knuckle_coord)
            )  # Unity space - Y
        else:
            palm_normal = normalize_vector(
                np.cross(pinky_knuckle_coord, index_knuckle_coord)
            )  # Unity space - Y
        palm_direction = normalize_vector(
            index_knuckle_coord + pinky_knuckle_coord
        )  # Unity space - Z

        if hand_name == "left":
            cross_product = normalize_vector(
                index_knuckle_coord - pinky_knuckle_coord
            )  # Unity space - X
        else:
            cross_product = normalize_vector(
                pinky_knuckle_coord - index_knuckle_coord
            )  # Unity space - X

        return [origin_coord, cross_product, palm_normal, palm_direction]

    def _get_hand_coords(self, keypoints):
        if keypoints[0] == 0:
            data_type = "absolute"
        else:
            data_type = "relative"

        return (
            data_type,
            np.asanyarray(keypoints[1:]).reshape(OCULUS_NUM_KEYPOINTS, 3),
        )

    def _get_finger_coords(self, raw_keypoints, raw_keypoints_left):
        return dict(
            index=np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS["index"]]]),
            middle=np.vstack(
                [raw_keypoints[0], raw_keypoints[OCULUS_JOINTS["middle"]]]
            ),
            ring=np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS["ring"]]]),
            thumb=np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS["thumb"]]]),
        ), dict(
            index=np.vstack(
                [raw_keypoints_left[0], raw_keypoints[OCULUS_JOINTS["index"]]]
            ),
            middle=np.vstack(
                [raw_keypoints_left[0], raw_keypoints[OCULUS_JOINTS["middle"]]]
            ),
            ring=np.vstack(
                [raw_keypoints_left[0], raw_keypoints[OCULUS_JOINTS["ring"]]]
            ),
            thumb=np.vstack(
                [raw_keypoints_left[0], raw_keypoints[OCULUS_JOINTS["thumb"]]]
            ),
        )

    def _operate_hand(self, hand_name, transformed_hand_coords):
        if hand_name in self.hands.keys():

            transformed_hand_coords = moving_average(
                transformed_hand_coords,
                self.coord_moving_average_queues[hand_name],
                self.moving_average_limit,
            )

            self.hands[hand_name].step(transformed_hand_coords)

    def _run_robots(self):
        for name in ["left", "right"]:

            raw_keypoints = self.keypoints_sockets[name].recv()
            keypoint_dict = self._extract_data_from_token(raw_keypoints)
            _, hand_data = self._get_hand_coords(keypoint_dict["keypoints"])
            transformed_hand_coords, transformed_hand_frame = self.transform_keypoints(
                hand_data, name
            )

            self._operate_hand(name, transformed_hand_coords)

    def run(self):

        self._init_hands()

        while True:
            try:
                self.timer.start_loop()
                self._run_robots()

                self.timer.end_loop()

            except KeyboardInterrupt:
                if "left" in self.keypoints_sockets:
                    self.keypoints_sockets["left"].close()
                if "right" in self.keypoints_sockets:
                    self.keypoints_sockets["right"].close()
                break

    def stream_hand(self):

        self.keypoints_publishers = dict()
        for hand_name in self.hand_names:
            port = LEFT_STREAM_PORT if hand_name == "left" else RIGHT_STREAM_PORT
            self.keypoints_publishers[hand_name] = ZMQPublisher(
                host=HOST, port=port
            )  # noqa: F405

        while True:
            try:
                self.timer.start_loop()
                for hand_name in self.hand_names:
                    raw_keypoints = self.keypoints_sockets[hand_name].recv()
                    keypoint_dict = self._extract_data_from_token(raw_keypoints)
                    _, hand_data = self._get_hand_coords(keypoint_dict["keypoints"])
                    transformed_hand_coords, _ = self.transform_keypoints(
                        hand_data, hand_name
                    )
                    transformed_hand_coords = moving_average(
                        transformed_hand_coords,
                        self.coord_moving_average_queues[hand_name],
                        self.moving_average_limit,
                    )
                    self.keypoints_publishers[hand_name].pub(
                        transformed_hand_coords, "keypoints"
                    )
                self.timer.end_loop()
            except KeyboardInterrupt:
                for hand_name in self.hand_names:
                    self.keypoints_sockets[hand_name].close()
                break


if __name__ == "__main__":
    vr_operator = OculusTeleoperator(
        HOST,
        OCULUS_LEFT_PORT,
        OCULUS_RIGHT_PORT,
        90,
        hands=["left", "right"],
    )
    vr_operator.run()
