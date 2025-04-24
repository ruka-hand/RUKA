import argparse
import os
from multiprocessing import Process

from ruka_hand.control.operator import RUKAOperator
from ruka_hand.utils.constants import HOST
from ruka_hand.utils.manus_streamer import MANUSStreamer
from ruka_hand.utils.manus_visualizer import ManusVisualizer


class ManusTeleoperator:
    def __init__(
        self,
        hand_names,
        frequency,
        record=False,
        data_save_dir=None,
    ):
        self.freq = frequency
        self.hand_names = hand_names
        self.record = record
        self.data_save_dir = data_save_dir
        self._start_teleop()

        if record:
            os.makedirs(self.data_save_dir, exist_ok=True)

    def get_processes(self):
        return self.processes

    def _start_teleop(self):

        self.processes = []
        for hand_name in self.hand_names:
            self.processes.append(
                Process(target=self._start_manus_visualizer, args=(hand_name,))
            )
        for hand_name in self.hand_names:
            self.processes.append(
                Process(target=self._start_manus_data_stream, args=(hand_name,))
            )
        for hand_name in self.hand_names:
            self.processes.append(
                Process(target=self._start_ruka_operator, args=(hand_name,))
            )

    def _start_manus_data_stream(self, hand_name):
        streamer = MANUSStreamer(
            hand_type=hand_name,
            frequency=100,
            host=HOST,
        )
        streamer.stream()

    def _start_manus_visualizer(self, hand_name):
        visualizer = ManusVisualizer(
            host=HOST, frequency=3, hand_type=hand_name, is_robot=False, is_3d=True
        )
        visualizer.stream()

    def _start_ruka_operator(self, hand_name):
        operator = RUKAOperator(
            hand_type=hand_name,
            moving_average_limit=10,
            fingertip_overshoot_ratio=0.1,
            joint_angle_overshoot_ratio=0.3,
            record=self.record,
            data_save_dir=self.data_save_dir,
        )
        try:
            operator.run()
        except KeyboardInterrupt:
            operator.controller.close()

    def run(self):
        for process in self.processes:
            process.start()
        for process in self.processes:
            process.join()


if __name__ == "__main__":
    manus_teleoperator = ManusTeleoperator(
        hand_names=["right"],
        frequency=50,
        record=False,
    )
    manus_teleoperator.run()
