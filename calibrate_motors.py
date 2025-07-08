# This script will move the the fingers with order automatically
import os
import time

import numpy as np

from ruka_hand.control.hand import *
from ruka_hand.utils.file_ops import get_repo_root
from ruka_hand.utils.trajectory import move_to_pos


class HandCalibrator:
    def __init__(
        self,
        hand_type,
        curr_lim=100,
        no_load_curr = 5,
        motor_ids=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
        verbose=True,
    ):
        self.hand = Hand(hand_type)
        self.curr_lim = curr_lim
        self.no_load_curr = no_load_curr
        self.motor_ids = motor_ids
        self.verbose = verbose

        self.curled = None
        self.tensioned = None

        # self.reset_motors()

    def reset_motors(self):
        print("Resetting motors...")
        for _ in range(10):
            curr_pos = self.hand.read_pos()
            time.sleep(.1)
            print(f"curr_pos: {curr_pos}, des_pos: {self.hand.tensioned_pos}")
            move_to_pos(curr_pos=curr_pos, des_pos=self.hand.tensioned_pos, hand=self.hand, traj_len=50)

        print("Motors reset! Now, manually adjust each tendon.")
        self.hand.dxl_client.set_torque_enabled(False)
        input("Press enter when done...")
        self.hand.dxl_client.set_torque_enabled(True)

        print(f"Tension pos: {[self.hand.read_pos()[i - 1] for i in self.motor_ids]}")

    def find_bound(self, motor_id):
        # Add cases for specific motors here
        # index, raise curr_lim
        # check to see if other fingers need raising
        # should we run three times? maybe more? to make sure it actually does it right
        # use present position
        t_curl = 1.5
        if motor_id in [1, 2, 3]:
            self.curr_lim = 40
        elif motor_id in [4, 5]:
            if motor_id == 4:
                self.curr_lim = 250
            else:
                self.curr_lim = 200
            t_curl = 3

        # People can manually rotate the servos to give tension, so that the start pos can vary
        # Reset the motors to be around 400
        start_pos = np.array([self.hand.read_pos()[i - 1] for i in self.motor_ids])
        tension_pos = start_pos[motor_id - 1]
        if self.verbose:
            print("------------ MOTOR ", motor_id, " ------------")
            print(f" - Manually tensioned at {tension_pos}")

        if self.hand.hand_type == "right":
            l_bound = max(100, tension_pos)
            u_bound = 3800
        elif self.hand.hand_type == "left":
            l_bound = 300
            u_bound = min(tension_pos, 4000)

        pos = start_pos.copy()
        cur = 1000000
        init_curl = True
        target_pos = (u_bound + l_bound) // 2 - 1

        while abs(u_bound - l_bound) > 10:  # or abs(cur) < self.curr_lim:
            # last_target = target_pos
            target_pos = (u_bound + l_bound) // 2 - 1

            # if not init_curl and abs(cur) >= self.curr_lim: 
            #     if self.hand.hand_type == "right":
            #         target_pos = max(last_target, target_pos)
            #     else:
            #         target_pos = min(last_target, target_pos)
            
            pos[motor_id - 1] = target_pos
            self.hand.set_pos(pos)
            time.sleep(t_curl)

            cur = self.hand.read_single_cur(motor_id)
            pres_pos = self.hand.read_pos()[motor_id - 1]
            if self.verbose:
                print(f"L: {l_bound}\tU: {u_bound}\tT: {target_pos}\tP: {pres_pos}\tCurrent: {cur}")

            # Reached the max curl
            if self.hand.hand_type == "right" and (u_bound - pres_pos < 2):
                break
            elif self.hand.hand_type == "left" and (pres_pos - l_bound < 2):
                break

            # Skip the calibration when there is no load
            if init_curl and abs(cur) < self.no_load_curr:
                print("No load. Skipping calibration.")
                break
            init_curl = False

            if abs(cur) < self.curr_lim:
                if self.hand.hand_type == "right":
                    print(" - increasing the lower bound")
                    l_bound = pres_pos + 1
                    u_bound -= 1
                else:
                    print(" - decreasing the upper bound")
                    u_bound = pres_pos - 1
                    l_bound += 1
            else:
                if self.hand.hand_type == "right":
                    print(" - decreasing the upper bound")
                    u_bound = int((3*pres_pos + u_bound)/4)
                else:
                    print(" - increasing the lower bound")
                    l_bound = int((3*pres_pos + l_bound)/4)

        pres_pos = self.hand.read_pos()[motor_id - 1]
        print(f"Tension pos: {tension_pos}, curled pos: {pres_pos}")
        self.hand.set_pos(start_pos)
        time.sleep(2)

        return tension_pos, pres_pos

    def find_curled_tensioned(self):
        self.curled = np.array([0] * len(self.motor_ids))
        self.tensioned = np.array([0] * len(self.motor_ids))
        for i in range(len(self.motor_ids)):
            self.tensioned[i], self.curled[i] = self.find_bound(self.motor_ids[i])

    def test_curled(self):
        start_pos = self.tensioned - 100
        start_pos[start_pos < 100] = 100

        for motor_id in self.motor_ids:
            # Reset pos
            curr_pos = self.hand.read_pos()
            time.sleep(.1)
            move_to_pos(curr_pos=curr_pos, des_pos=start_pos, hand=self.hand, traj_len=50, sleep_time=0.02)

            print(f"Testing motor {motor_id}")
            time.sleep(.1)
            curr_pos = self.hand.read_pos()
            test_pos = start_pos.copy()
            test_pos[motor_id-1] = self.curled[motor_id-1]
            move_to_pos(curr_pos=curr_pos, des_pos=test_pos, hand=self.hand, traj_len=200, sleep_time=0.01)

        # Reset pos
        curr_pos = self.hand.read_pos()
        time.sleep(.1)
        move_to_pos(curr_pos=curr_pos, des_pos=start_pos, hand=self.hand, traj_len=50, sleep_time=0.02)

        print("Testing all motors")
        time.sleep(.1)
        curr_pos = self.hand.read_pos()
        test_pos = self.curled.copy()
        # test_pos[0] = curr_pos[0]  # pass thumb abduction
        test_pos[1:2] = curr_pos[1:2]  # pass thumb joint abductions
        move_to_pos(curr_pos=curr_pos, des_pos=test_pos, hand=self.hand, traj_len=200, sleep_time=0.01)


    def save_motor_limits(self, save_path):
        if self.curled is None or self.tensioned is None:
            self.find_curled_tensioned()
        np.savez(save_path, curled=self.curled, tensioned=self.tensioned)

    def load_motor_limits(self, load_path):
        data = np.load(load_path)
        self.curled = data["curled"]
        self.tensioned = data["tensioned"]

def parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="Calibrate RUKA hand motors")
    parser.add_argument(
        "--hand-type",
        type=str,
        default="right",
        choices=["right", "left"],
        help="Type of hand to calibrate (right or left)",
    )
    parser.add_argument(
        "--verbose",
        type=bool,
        default=True,
        help="Enable testing mode with debug prints",
    )
    parser.add_argument(
        "--curr-lim", type=int, default=100, help="Current limit for calibration"
    )
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    repo_root = get_repo_root()
    if not os.path.exists(f"{repo_root}/motor_limits"):
        os.makedirs(f"{repo_root}/motor_limits")

    limit_file = f"{repo_root}/motor_limits/{args.hand_type}_motor_limits.npz"

    calibrator = HandCalibrator(
        hand_type=args.hand_type,
        curr_lim=args.curr_lim,
        verbose=args.verbose,
    )
    calibrator.reset_motors()
    calibrator.find_curled_tensioned()
    calibrator.save_motor_limits(limit_file)

    calibrator2 = HandCalibrator( 
        hand_type=args.hand_type,
        curr_lim=args.curr_lim,
        verbose=args.verbose,
    )
    calibrator2.load_motor_limits(limit_file)
    while True:
        calibrator2.test_curled()
