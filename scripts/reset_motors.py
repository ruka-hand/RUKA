import argparse
import time

from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

parser = argparse.ArgumentParser(description="Teleop robot hands.")
parser.add_argument(
    "-ht",
    "--hand_type",
    type=str,
    help="Hand you'd like to teleoperate",
    default="left",
)
args = parser.parse_args()
hand = Hand(args.hand_type)
while True:
    curr_pos = hand.read_pos()
    time.sleep(0.5)
    print(f"curr_pos: {curr_pos}, des_pos: {hand.tensioned_pos}")
    test_pos = hand.tensioned_pos
    try:
        move_to_pos(curr_pos=curr_pos, des_pos=test_pos, hand=hand, traj_len=50)
    except:
        break


hand.close()
