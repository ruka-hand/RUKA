# Will start the data collection on both the robot and the manus in different processes
import argparse
import os
from multiprocessing import Process

import numpy as np

from ruka_hand.data_collection.save_manus_data import MANUSDataCollector
from ruka_hand.data_collection.save_robot_data import RUKADataCollector
from ruka_hand.utils.constants import HOST, LEFT_GLOVE_ID, RIGHT_GLOVE_ID
from ruka_hand.utils.manus_streamer import MANUSStreamer


class DataCollector:
    def __init__(self, demo_dir, hand_type, demo_num, record_fps):

        self._storage_path = f"{demo_dir}/demonstration_{demo_num}"
        if not os.path.exists(self._storage_path):
            os.makedirs(self._storage_path)

        self.record_fps = record_fps
        self.hand_type = hand_type
        self.glove_id = LEFT_GLOVE_ID if hand_type == "left" else RIGHT_GLOVE_ID

        self._start_recording()

    def get_processes(self):
        return self.processes

    def _start_ruka_data_collection(self):
        collector = RUKADataCollector(
            data_save_dir=self._storage_path,
            frequency=self.record_fps,
            num_intervals=10,
            hand_type=self.hand_type,
            wait_period=-1,
        )

        collector.save_data_with_random_walk(
            finger_names=["Thumb"],
            step_size=11,
            sample_num=500,
            walk_len=100,
        )

    def _start_manus_data_collection(self):
        collector = MANUSDataCollector(
            data_save_dir=self._storage_path,
            host=HOST,
            frequency=self.record_fps,
            hand_type=self.hand_type,
        )
        collector.stream()

    def _start_manus_data_stream(self):
        streamer = MANUSStreamer(
            hand_type=self.hand_type,
            frequency=100,
            host=HOST,
        )
        streamer.stream()

    def _start_recording(self):
        self.processes = []
        self.processes.append(Process(target=self._start_manus_data_stream))
        self.processes.append(Process(target=self._start_manus_data_collection))
        self.processes.append(Process(target=self._start_ruka_data_collection))


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Collect data with the robot hands.")
    parser.add_argument(
        "-ht",
        "--hand_type",
        type=str,
        help="Hand you'd like to collect data with",
        default="right",
    )
    parser.add_argument(
        "-d",
        "--demo_num",
        type=str,
        help="Demo number",
        default="1",
    )
    parser.add_argument(
        "-r",
        "--root_data",
        type=str,
        help="Root data directory",
        default="/data",
    )
    args = parser.parse_args()

    hand_type = args.hand_type
    demo_num = args.demo_num
    root_data = args.root_data
    demo_dir = f"{root_data}/{hand_type}_hand"

    collector = DataCollector(
        demo_dir=demo_dir,
        demo_num=f"{demo_num}",  # after 5
        hand_type=hand_type,
        record_fps=15,
    )

    processes = collector.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()
