import numpy as np
import zmq

from ruka_hand.utils.constants import (
    HOST,
    LEFT_GLOVE_ID,
    LEFT_STREAM_PORT,
    RIGHT_GLOVE_ID,
    RIGHT_STREAM_PORT,
)
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.zmq import ZMQPublisher


class MANUSStreamer:
    def __init__(self, hand_type, frequency, host):
        self.glove_id = LEFT_GLOVE_ID if hand_type == "left" else RIGHT_GLOVE_ID
        self.hand_type = hand_type

        context = zmq.Context()
        # Socket to talk to Manus SDK
        print("Connecting to SDK")
        self.socket = context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, True)
        self.socket.connect("tcp://localhost:8000")

        self.timer = FrequencyTimer(frequency)
        self.viz_data, self.joint_angles, self.fingertips, self.keypoints = (
            {},
            np.zeros((5, 3)),
            np.zeros((5, 3)),
            np.zeros((5, 5, 3)),
        )
        self.visualize = False
        self.stream_port = (
            LEFT_STREAM_PORT if hand_type == "left" else RIGHT_STREAM_PORT
        )
        self.publisher = ZMQPublisher(host=host, port=self.stream_port)

    def _set_visualization_data(self, received_data):
        if received_data[0] == self.glove_id:
            positions = [
                [float(received_data[i]), float(received_data[i + 2])]
                for i in range(1, 176, 7)
            ]
            self.viz_data["Wrist"] = positions[0]
            self.viz_data["Index"] = np.reshape(np.array(positions[1:6]), (10))
            self.viz_data["Middle"] = np.reshape(np.array(positions[6:11]), (10))
            self.viz_data["Ring"] = np.reshape(np.array(positions[11:16]), (10))
            self.viz_data["Pinky"] = np.reshape(np.array(positions[16:21]), (10))
            self.viz_data["Thumb"] = np.reshape(np.array(positions[21:25]), (8))

    def _set_joint_angles(self, received_data):

        if self.hand_type == "left":
            data_range = (0, 20)

        elif self.hand_type == "right":
            data_range = (20, 40)

        received_data = received_data[data_range[0] : data_range[1]]
        joint_angles = [
            list(map(float, received_data[i : i + 4]))
            for i in range(0, len(received_data), 4)
        ]

        self.joint_angles[1, :] = joint_angles[0][
            1:
        ]  # It starts with the index finger - whereas we first start with the thumb
        self.joint_angles[2, :] = joint_angles[1][1:]
        self.joint_angles[3, :] = joint_angles[2][1:]
        self.joint_angles[4, :] = joint_angles[3][1:]
        self.joint_angles[0, :] = joint_angles[4][1:]

    def _set_keypoints(self, received_data):
        if received_data[0] == self.glove_id:
            positions = [
                list(map(float, received_data[i : i + 3])) for i in range(1, 176, 7)
            ]

            self.keypoints[1] = np.array(positions[1:6])
            self.keypoints[2] = np.array(positions[6:11])
            self.keypoints[3] = np.array(positions[16:21])
            self.keypoints[4] = np.array(positions[11:16])
            self.keypoints[0] = np.concatenate(
                [np.array([positions[0]]), np.array(positions[21:25])], axis=0
            )

    def _set_fingertips(
        self, received_data
    ):  # This will for sure have 176 numbers in it
        if received_data[0] == self.glove_id:
            positions = [
                list(map(float, received_data[i : i + 3])) for i in range(1, 176, 7)
            ]
            self.fingertips[1] = np.array(positions[5])
            self.fingertips[2] = np.array(positions[10])
            self.fingertips[3] = np.array(positions[20])
            self.fingertips[4] = np.array(positions[15])
            self.fingertips[0] = np.array(positions[24])

            self.fingertips[:, 2] *= -1

    def stream(self):
        print(f"** STARTING HAND DATA STREAMING **")
        while True:

            self.timer.start_loop()

            # receive the message from the socket
            message = self.socket.recv()
            message = message.decode("utf-8")
            data = message.split(",")
            # print(len(data))
            if len(data) == 40:
                self._set_joint_angles(data)
            elif len(data) == 352:
                self._set_fingertips(received_data=data[0:176])
                self._set_fingertips(received_data=data[176:352])

                self._set_keypoints(received_data=data[0:176])
                self._set_keypoints(received_data=data[0:176])
            elif len(data) == 176:
                self._set_fingertips(received_data=data[0:176])
                self._set_keypoints(received_data=data[0:176])

            self.publisher.pub(data_array=self.joint_angles, topic_name="joint_angles")
            self.publisher.pub(data_array=self.fingertips, topic_name="fingertips")
            self.publisher.pub(data_array=self.keypoints, topic_name="keypoints")

            self.timer.end_loop()


if __name__ == "__main__":

    streamer = MANUSStreamer(
        hand_type="left",
        frequency=100,
        host=HOST,
    )
    streamer.stream()
