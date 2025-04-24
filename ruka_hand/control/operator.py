# This operator is used for preprocessing any data before it is sent to the controller
# Will listen to the zmq topic that are being published by the fingerdata streamer
from ruka_hand.control.controller import HandController
from ruka_hand.utils.constants import *
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import *
from ruka_hand.utils.zmq import ZMQSubscriber


class RUKAOperator:
    def __init__(
        self,
        hand_type,
        moving_average_limit=1,
        fingertip_overshoot_ratio=0,
        joint_angle_overshoot_ratio=0,
        record=False,
        data_save_dir=None,
    ):

        self.hand_type = hand_type
        self.moving_average_limit = moving_average_limit
        self.motor_moving_average_queue = []
        self.manus_moving_average_queue = []

        # Initialize controller
        self.controller = HandController(
            hand_type=hand_type,
            frequency=100,
            single_move_len=1,
            record=record,
            data_save_dir=data_save_dir,
        )

        self.fingertip_overshoot_ratio = fingertip_overshoot_ratio
        self.joint_angle_overshoot_ratio = joint_angle_overshoot_ratio

    def _overshoot_fingertips(
        self,
        fingertips,
        scale_x_axis_of_top_four=0.1,
        scale_z_axis=1.5,
        sigma_factor=1,
    ):
        """
        Compute a residual vector to move fingertips closer to their neighbors
        while keeping distant fingertips apart.

        Args:
            fingertips (np.ndarray): (5,3) array of fingertip positions relative to the wrist.
            scaling_factor (float): A factor to control the strength of the movement.

        Returns:
            np.ndarray: (5,3) residual vectors to be added to fingertips.
        """
        # Compute pairwise distances
        distances = np.linalg.norm(
            fingertips[:, np.newaxis, :] - fingertips[np.newaxis, :, :], axis=-1
        )  # (5,5) distance matrix

        # Compute weights: Closer points get stronger attraction
        sigma = sigma_factor * np.mean(distances)  # Set a characteristic scale
        weights = np.exp(-(distances**2) / (2 * sigma**2))  # (5,5)

        # Zero out self-contributions (no self-attraction)
        np.fill_diagonal(weights, 0)

        # Compute movement vectors: weighted attraction to neighbors
        weighted_sum = np.dot(weights, fingertips) / np.sum(
            weights, axis=1, keepdims=True
        )
        residuals = weighted_sum - fingertips

        # Scale and return
        residuals[1:, 0] *= scale_x_axis_of_top_four
        residuals[:, 2] *= scale_z_axis
        return fingertips + self.fingertip_overshoot_ratio * residuals

    def _overshoot_joint_angles(self, joint_angles):

        # NOTE: This code only overshoots joint angles for the four fingers and not the thumb
        # thumb angles are very off so linear overshooting doesn't work!
        angle_maxes = np.array(
            [[96, 100, 80], [91, 100, 80], [91, 91, 73], [93, 100, 80]]
        )
        angle_mins = np.array([[-26, 12, 9], [-34, 10, 8], [20, 3, 3], [15, 0, 0]])
        # angle_mins = np.array([[-26, 0, 0], [-34, 0, 0], [20, 0, 0], [15, 0, 0]])
        angle_range = angle_maxes - angle_mins
        joint_angle_residual = (
            ((joint_angles[1:] - angle_mins) / angle_range)
            * self.joint_angle_overshoot_ratio
            * angle_range
        )
        joint_angles[1:] = joint_angles[1:] + joint_angle_residual

        return joint_angles

    def _handle_input_type(self, fingertips, joint_angles, input_type):

        fingertips = self._overshoot_fingertips(fingertips)
        joint_angles = self._overshoot_joint_angles(joint_angles)

        if input_type == "fingertips":
            return fingertips
        if input_type == "joint_angles":
            return joint_angles
        if input_type == "thumb_special":
            thumb_fingertip = np.expand_dims(fingertips[0], 0)
            return np.concatenate([thumb_fingertip, joint_angles[1:]], axis=0)

    def _init_subscribers(self, r2r_teleop=False):

        if r2r_teleop:
            self.stream_port = (
                LEFT_STREAM_PORT if self.hand_type == "right" else RIGHT_STREAM_PORT
            )  # We'll read from opposite ports
        else:
            self.stream_port = (
                LEFT_STREAM_PORT if self.hand_type == "left" else RIGHT_STREAM_PORT
            )

        self.keypoints_subscriber = ZMQSubscriber(
            host=HOST, port=self.stream_port, topic="keypoints"
        )

    def _get_model_input(self, flip_x_axis=False):

        keypoints = self.keypoints_subscriber.recv()
        if flip_x_axis:  # This is only used in robot_to_robot teleop
            keypoints[:, :, 0] = -keypoints[:, :, 0]
        print(f"keypoints.shape: {keypoints.shape}")

        fingertips = calculate_fingertips(keypoints)
        joint_angles = calculate_joint_angles(keypoints)

        if np.isnan(joint_angles).any() or np.isnan(fingertips).any():
            print("NAN values inputted skipping")
            return None

        return self._handle_input_type(
            fingertips=fingertips,
            joint_angles=joint_angles,
            input_type=self.controller.input_type,
        )

    def step(self, keypoints):
        fingertips = calculate_fingertips(keypoints)
        joint_angles = calculate_joint_angles(keypoints)
        if np.isnan(joint_angles).any() or np.isnan(fingertips).any():
            print("NAN values inputted skipping")
            return
        model_input = self._handle_input_type(
            fingertips=fingertips,
            joint_angles=joint_angles,
            input_type=self.controller.input_type,
        )
        self.controller.step(
            input_data=model_input,
            moving_average_info={
                "queue": self.motor_moving_average_queue,
                "limit": self.moving_average_limit,
            },
        )  # (5,3)

    def run(self, r2r_teleop=False, frequency=25):

        timer = FrequencyTimer(frequency)
        self._init_subscribers(r2r_teleop)

        while True:
            timer.start_loop()
            model_input = self._get_model_input(r2r_teleop)
            if model_input is None:
                continue

            self.controller.step(
                input_data=model_input,
                moving_average_info={
                    "queue": self.motor_moving_average_queue,
                    "limit": self.moving_average_limit,
                },
            )  # (5,3)
            timer.end_loop()

    def reset(self):
        self.controller.reset()
