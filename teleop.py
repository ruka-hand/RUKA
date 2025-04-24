import argparse

from ruka_hand.teleoperation.manus_teleoperator import ManusTeleoperator
from ruka_hand.teleoperation.oculus_teleoperator import OculusTeleoperator
from ruka_hand.utils.constants import HOST, OCULUS_LEFT_PORT, OCULUS_RIGHT_PORT

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Teleop robot hands.")
    parser.add_argument(
        "-ht",
        "--hand_type",
        type=str,
        help="Hand you'd like to teleoperate",
        default="",
    )

    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        help="Mode you'd like to teleoperate",
        default="manus",
    )

    args = parser.parse_args()

    if args.mode == "manus":
        manus_teleoperator = ManusTeleoperator(
            hand_names=[args.hand_type],
            frequency=50,
            record=False,
        )
        manus_teleoperator.run()
    elif args.mode == "oculus":
        oculus_teleoperator = OculusTeleoperator(
            HOST,
            OCULUS_LEFT_PORT,
            OCULUS_RIGHT_PORT,
            90,
            hands=[args.hand_type],
        )
        oculus_teleoperator.run()
