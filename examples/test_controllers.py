# This example shows how to load the controller and use it on different examples (robot/human keypoints)
import os

import numpy as np

from ruka_hand.control.operator import RUKAOperator
from ruka_hand.utils.file_ops import get_repo_root
from ruka_hand.utils.timer import FrequencyTimer


def load_keypoints(test_type: str, hand_type: str):
    repo_root = get_repo_root()
    examples_dir = "ruka_data/osfstorage/examples"
    keypoints = np.load(
        os.path.join(repo_root, examples_dir, f"{test_type}_examples_{hand_type}.npy")
    )
    return keypoints


def run_controller(keypoints: np.ndarray, hand_type: str):

    timer = FrequencyTimer(10)

    # Initialize the RUKAOperator
    operator = RUKAOperator(hand_type=hand_type, moving_average_limit=2)

    # Run the controller
    for keypoint in keypoints:
        timer.start_loop()
        operator.step(keypoint)
        timer.end_loop()


if __name__ == "__main__":
    hand_type = "right"

    keypoints = load_keypoints("human", hand_type)
    run_controller(keypoints, hand_type)
