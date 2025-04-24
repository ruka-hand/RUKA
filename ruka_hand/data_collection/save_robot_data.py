# This method will move the robot and save data in a h5py file
import os
import random
import time
from copy import deepcopy as copy
from itertools import permutations

from tqdm import tqdm

from ruka_hand.control.hand import Hand
from ruka_hand.data_collection.recorder import Recorder
from ruka_hand.utils.extract_control_table import df_controlTable
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import *


class RUKADataCollector(Recorder):
    def __init__(self, num_intervals, wait_period, frequency, hand_type, data_save_dir):
        self.hand = Hand(hand_type)
        self.hand_pos = copy(self.hand.tensioned_pos)
        self.data_save_dir = data_save_dir
        if not os.path.exists(data_save_dir):
            os.makedirs(data_save_dir)

        self.timer = FrequencyTimer(frequency)
        self.frequency = frequency
        self.num_intervals = num_intervals
        self.wait_period = wait_period

        self._recorder_file_name = f"{self.data_save_dir}/ruka_data.h5"
        self.ruka_data = dict()

        # Input motor limits
        self.ruka_data["max_motor_lim"] = self.hand.max_lim
        self.ruka_data["min_motor_lim"] = self.hand.min_lim

        self._data_names = dict(
            present_position="Present Position",
            temperature="Present Temperature",
            current="Present Current",
        )

        self.num_datapoints = 0
        self.record_start_time = time.time()

    def _update_ruka_data(self, commanded_position):
        curr_data = dict()
        for key, value in self._data_names.items():
            row = df_controlTable[df_controlTable["Data Name"] == value]
            addr = int(row["Address"].values[0])
            size = int(row["Size(Byte)"].values[0])
            data = self.hand.read_any(addr, size)
            curr_data[key] = np.array(data)
            if key == "present_position":
                curr_data["timestamp"] = (
                    time.time()
                )  # Save the time of the present position

        curr_data["commanded_position"] = np.array(commanded_position)

        for key in curr_data.keys():
            if key not in self.ruka_data:
                self.ruka_data[key] = [curr_data[key]]
            else:
                self.ruka_data[key].append(curr_data[key])

        self.num_datapoints += 1
        # return curr_data

    def move_to_pos(
        self,
        curr_pos,
        des_pos,
        traj_len=50,
    ):

        if traj_len == 1:
            self.hand.set_pos(des_pos)
            self.hand_pos = des_pos
            self._update_ruka_data(commanded_position=des_pos)
            return

        if traj_len == 1:
            trajectory = [des_pos]
        else:
            trajectory = np.linspace(curr_pos, des_pos, traj_len)[
                1:
            ]  # Don't include the first pose

        for hand_pos in trajectory:
            self.timer.start_loop()

            self.hand.set_pos(hand_pos)
            self.hand_pos = hand_pos

            self._update_ruka_data(commanded_position=hand_pos)

            self.timer.end_loop()

    def wait(self):

        if self.wait_period == -1:
            return

        for _ in range(int(self.wait_period * self.frequency)):
            self.timer.start_loop()
            self._update_ruka_data(commanded_position=self.hand_pos)
            self.timer.end_loop()

    def _iterate_through_fingers(self, motor_intervals, f_id, s_id, finger_names):
        for finger_name in finger_names:
            rand_res = np.random.normal(0, 1, size=2)
            rand_res = np.clip(
                (rand_res * 10).astype(int), 1, 20
            )  # It should always be positive
            motor_ids = self.hand.fingers_dict[finger_name]
            first_finger_val = (
                motor_intervals[
                    f_id,
                    motor_ids[0],
                ]
                + rand_res[0]
            )
            sec_finger_val = (
                motor_intervals[
                    s_id,
                    motor_ids[1],
                ]
                + rand_res[1]
            )
            self.hand_pos[motor_ids[0]] = int(first_finger_val)
            self.hand_pos[motor_ids[1]] = int(sec_finger_val)

        self.move_to_pos(
            curr_pos=self.hand.read_pos(),
            des_pos=self.hand_pos,
            traj_len=5,
        )

        self.wait()

    def _reset_four_fingers(self, motor_order):
        reset_pos = self.hand.read_pos()
        for finger_name in ["Index", "Middle", "Ring", "Pinky"]:
            motor_ids = self.hand.fingers_dict[finger_name]
            reset_pos[motor_ids[motor_order]] = self.hand.tensioned_pos[
                motor_ids[motor_order]
            ]

        self.reset(reset_pos)

    def _reset_thumb(self, motor_ids_to_reset):
        reset_pos = self.hand.read_pos()
        for motor_id in motor_ids_to_reset:
            reset_pos[motor_id] = self.hand.tensioned_pos[motor_id]

        self.reset(reset_pos)

    def reset(self, des_pos=None):
        self.move_to_pos(
            curr_pos=self.hand.read_pos(),
            des_pos=des_pos,
            traj_len=30,
        )

        self.wait()

    def save_data_with_random_walk(self, finger_names, step_size, sample_num, walk_len):

        print("Saving data with random walk")
        pbar = tqdm(total=sample_num * walk_len)
        for sample_id in range(sample_num):
            try:
                des_hand_pos = copy(self.hand.tensioned_pos)
                walk_ranges = dict()
                desired_poses = dict()
                for finger_name in finger_names:
                    motor_ids = self.hand.fingers_dict[finger_name]
                    walk_ranges[finger_name] = [
                        self.hand.min_lim[motor_ids],
                        self.hand.max_lim[motor_ids],
                    ]

                    desired_pose = [
                        random.randint(
                            walk_ranges[finger_name][0][i],
                            walk_ranges[finger_name][1][i],
                        )
                        for i in range(len(walk_ranges[finger_name][0]))
                    ]
                    des_hand_pos[motor_ids] = desired_pose
                    desired_poses[finger_name] = desired_pose

                self.move_to_pos(
                    self.hand.read_pos(),
                    des_hand_pos,
                    traj_len=10,
                )
                for step_id in range(walk_len):
                    for finger_name in finger_names:
                        motor_ids = self.hand.fingers_dict[finger_name]
                        random_nums = np.random.random(len(walk_ranges[finger_name][0]))
                        new_des_pos = []
                        for i, rand_num in enumerate(random_nums):
                            if (
                                rand_num < 0.33
                                and desired_poses[finger_name][i] + step_size
                                < walk_ranges[finger_name][1][i]
                            ):
                                new_des_pos.append(
                                    desired_poses[finger_name][i] + step_size
                                )
                            elif (
                                rand_num > 0.66
                                and desired_poses[finger_name][i] - step_size
                                > walk_ranges[finger_name][0][i]
                            ):
                                new_des_pos.append(
                                    desired_poses[finger_name][i] - step_size
                                )
                            else:
                                new_des_pos.append(desired_poses[finger_name][i])
                        desired_poses[finger_name] = new_des_pos

                        des_hand_pos[motor_ids] = desired_poses[finger_name]

                    self.move_to_pos(
                        self.hand.read_pos(),
                        des_hand_pos,
                        traj_len=1,
                    )
                    self.wait()
                    pbar.update(1)
                    pbar.set_description(f"Sample: {sample_id} | Step: {step_id}")

            except KeyboardInterrupt:
                break

        pbar.close()
        print(f"** ROBOT SAVING DONE IN {self._recorder_file_name}")
        self.record_end_time = time.time()
        self._compress_data(data_dict=self.ruka_data)
        print("Saved manus_data data in {}.".format(self._recorder_file_name))
        self.hand.close()
