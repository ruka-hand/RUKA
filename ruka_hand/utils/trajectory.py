import time

import numpy as np


def move_to_pos(
    curr_pos,
    des_pos,
    hand,
    data_saver=None,
    traj_len=50,
    sleep_time=0.01,
):
    if traj_len == 1:
        trajectory = [des_pos]
    else:
        trajectory = np.linspace(curr_pos, des_pos, traj_len)[
            1:
        ]  # Don't include the first pose

    for hand_pos in trajectory:
        hand.set_pos(hand_pos)
        time.sleep(sleep_time)

        if not data_saver is None:
            data_saver.save_single_row(hand, hand_pos)

    return True, hand_pos
