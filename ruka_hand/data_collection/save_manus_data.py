# Will read and save the manus data in a csv file
import os
import time

import h5py

from ruka_hand.data_collection.recorder import Recorder
from ruka_hand.utils.constants import HOST, LEFT_STREAM_PORT, RIGHT_STREAM_PORT
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import *
from ruka_hand.utils.zmq import ZMQSubscriber


class MANUSDataCollector(Recorder):
    def __init__(self, data_save_dir, host, frequency, hand_type):

        stream_port = LEFT_STREAM_PORT if hand_type == "left" else RIGHT_STREAM_PORT
        self.keypoints_subscriber = ZMQSubscriber(host, stream_port, "keypoints")
        self.timer = FrequencyTimer(frequency)

        self.data_save_dir = data_save_dir
        if not os.path.exists(data_save_dir):
            os.makedirs(data_save_dir)

        self._recorder_file_name = f"{self.data_save_dir}/manus_data.h5"
        self.manus_data = dict()
        self.manus_data["hand_type"] = hand_type

    def _get_manus_data(self):
        keypoints = self.keypoints_subscriber.recv()
        joint_angles = calculate_joint_angles(keypoints)
        fingertips = keypoints[:, -1, :]

        return dict(
            joint_angle=joint_angles,
            fingertip=fingertips,
            keypoint=keypoints,
            timestamp=time.time(),
        )

    def stream(self):

        self.num_datapoints = 0
        self.record_start_time = time.time()

        while True:
            self.timer.start_loop()
            try:

                datapoint = self._get_manus_data()
                for key in datapoint.keys():
                    if key not in self.manus_data:
                        self.manus_data[key] = [datapoint[key]]
                    else:
                        self.manus_data[key].append(datapoint[key])

                self.num_datapoints += 1
                self.timer.end_loop()

            except:  # Save the data
                break

        print(f"** MANUS SAVING DONE IN {self._recorder_file_name} **")
        self.record_end_time = time.time()
        self._compress_data(data_dict=self.manus_data)

        print("Saved manus_data data in {}.".format(self._recorder_file_name))

    def _compress_data(self, data_dict):
        self._display_statistics(self.num_datapoints)
        self._add_metadata(self.num_datapoints)

        # Writing to dataset
        print("Compressing keypoint data...")
        with h5py.File(self._recorder_file_name, "w") as file:
            # Main data
            for key in data_dict.keys():
                if key != "timestamp" and key != "hand_type":
                    data_dict[key] = np.array(data_dict[key], dtype=np.float32)
                elif key == "hand_type":
                    # Convert string array to fixed-length bytes for HDF5 compatibility
                    string_data = np.array(data_dict[key])
                    data_dict[key] = string_data.astype(
                        "S"
                    )  # automatically determines max length
                else:
                    data_dict["timestamp"] = np.array(
                        data_dict["timestamp"], dtype=np.float64
                    )

                # Check if the data is scalar (single value) or array
                if np.isscalar(data_dict[key]) or data_dict[key].shape == ():
                    # Create dataset without compression for scalar values
                    file.create_dataset(key + "s", data=data_dict[key])
                else:
                    # Apply compression only for array data
                    file.create_dataset(
                        key + "s",
                        data=data_dict[key],
                        compression="gzip",
                        compression_opts=6,
                    )

            # Other metadata
            file.update(self.metadata)


if __name__ == "__main__":
    collector = MANUSDataCollector(
        data_save_dir="/data_ssd/irmak/robot-hand-project/data/all_four_fingers/linux_sdk_data/demonstration_2",
        host=HOST,
        stream_port=RIGHT_STREAM_PORT,
        frequency=15,
    )
    collector.stream()
