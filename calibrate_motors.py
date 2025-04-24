# This script will move the the fingers with order automatically
import os
import time

import numpy as np

from ruka_hand.control.hand import *
from ruka_hand.utils.file_ops import get_repo_root


class HandCalibrator:
    def __init__(
        self,
        data_save_path,
        hand_type,
        curr_lim=50,
        testing=False,
        motor_ids=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
    ):
        self.hand = Hand(hand_type)
        self.curr_lim = curr_lim
        self.testing = testing
        self.motor_ids = motor_ids
        self.data_save_path = data_save_path

    def find_bound(self, motor_id):
        # Add cases for specific motors here
        # index, raise curr_lim
        # check to see if other fingers need raising
        # should we run three times? maybe more? to make sure it actually does it right
        # use present position
        t = 2
        if motor_id in [4, 5]:
            if motor_id == 4:
                self.curr_lim = 250
            else:
                self.curr_lim = 200
            t = 5
        if self.testing:
            print("------------ MOTOR ", motor_id, " ------------")
        if self.hand.hand_type == "right":
            start_pos = 100
            f = 1
        elif self.hand.hand_type == "left":
            start_pos = 4000
            f = -1
        l_bound = 100
        u_bound = 4000
        pos = np.array([start_pos] * 11)
        cur = 1000000
        while abs(u_bound - l_bound) > 10 or f * cur > self.curr_lim:
            com_pos = (u_bound + l_bound) // 2 - 1
            # if self.testing: print(u_bound, "    ", l_bound, "         ", com_pos)
            pos[motor_id - 1] = com_pos
            self.hand.set_pos(pos)
            time.sleep(t)
            cur = self.hand.read_single_cur(motor_id)
            pres_pos = self.hand.read_pos()[motor_id - 1]
            if self.testing:
                print(
                    u_bound,
                    "    ",
                    l_bound,
                    "         ",
                    com_pos,
                    "         ",
                    pres_pos,
                )
            if self.testing:
                print(cur)
            if f * cur < self.curr_lim:
                if self.hand.hand_type == "right":
                    l_bound = pres_pos + 1
                    u_bound -= 1
                else:
                    u_bound = pres_pos - 1
                    l_bound += 1
            else:
                if self.hand.hand_type == "right":
                    u_bound = pres_pos + 1
                else:
                    l_bound = pres_pos - 1
        return pres_pos

    def find_curled_tensioned(self):
        if self.hand.hand_type == "right":
            f = 1
        else:
            f = -1
        curled = np.array([0] * len(self.motor_ids))
        for i in range(len(curled)):
            curled[i] = self.find_bound(self.motor_ids[i])
        tensioned = [x - f * 1100 for x in curled]
        return curled, tensioned

    def save_motor_limits(self):
        curled, _ = self.find_curled_tensioned()
        np.save(self.data_save_path, curled)


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
        "--testing",
        type=bool,
        default=True,
        help="Enable testing mode with debug prints",
    )
    parser.add_argument(
        "--curr-lim", type=int, default=50, help="Current limit for calibration"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    repo_root = get_repo_root()
    if not os.path.exists(f"{repo_root}/motor_limits"):
        os.makedirs(f"{repo_root}/motor_limits")
    calibrator = HandCalibrator(
        data_save_path=f"{repo_root}/motor_limits/{args.hand_type}_motor_limits.npy",
        hand_type=args.hand_type,
        curr_lim=args.curr_lim,
        testing=args.testing,
    )
    calibrator.save_motor_limits()
