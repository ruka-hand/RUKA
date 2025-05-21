import os
from copy import deepcopy as copy

import numpy as np

from ruka_hand.utils.constants import (
    FINGER_NAMES_TO_MOTOR_IDS,
    MOTOR_RANGES_LEFT,
    MOTOR_RANGES_RIGHT,
    USB_PORTS,
)
from ruka_hand.utils.dynamixel_util import *
from ruka_hand.utils.file_ops import get_repo_root

"""port: The Dynamixel device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
"""
# PORT = "/dev/tty.usbserial-FT8ISQ5P"
# PORT = "/dev/ttyUSB0"
# PORT = "/dev/ttyUSB0"

# PID Gains
MCP_D_GAIN = 1000
MCP_I_GAIN = 120
MCP_P_GAIN = 450

DIP_PIP_D_GAIN = 960
DIP_PIP_I_GAIN = 100
DIP_PIP_P_GAIN = 500


class Hand:
    """Robot Hand class.
    Initializes dynamixel client, sets motor ids and initial motor settings.
    """

    def __init__(self, hand_type="right"):
        self.motors = motors = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.DIP_PIP_motors = [4, 6, 9, 11]
        self.MCP_motors = [5, 7, 8, 10]
        # self.motors = motors = [1]
        self.port = USB_PORTS[hand_type]
        self.dxl_client = DynamixelClient(motors, self.port)
        self.dxl_client.connect()

        self.fingers_dict = FINGER_NAMES_TO_MOTOR_IDS

        # Paramaters for initalization
        self.hand_type = hand_type
        self.curr_lim = 700
        self.temp_lim = 60
        self.goal_velocity = 400
        self.operating_mode = 5  # 5: current-based position control

        # These should be the values you get from running robot_hand.utils.calibration and python -m robot_hand.utils.tension respectively
        repo_root = get_repo_root()
        if hand_type == "right":
            if os.path.exists(f"{repo_root}/motor_limits/right_motor_limits.npy"):
                self.curled_bound = np.load(
                    f"{repo_root}/motor_limits/right_motor_limits.npy"
                )
            else:
                self.curled_bound = np.ones(11) * MOTOR_RANGES_RIGHT
            self.tensioned_pos = self.curled_bound - MOTOR_RANGES_RIGHT

            self.min_lim, self.max_lim = self.tensioned_pos, self.curled_bound
        elif hand_type == "left":
            if os.path.exists(f"{repo_root}/motor_limits/left_motor_limits.npy"):
                self.curled_bound = np.load(
                    f"{repo_root}/motor_limits/left_motor_limits.npy"
                )
            else:
                self.curled_bound = 4000 - np.ones(11) * MOTOR_RANGES_LEFT
            self.tensioned_pos = self.curled_bound + MOTOR_RANGES_LEFT

            self.min_lim, self.max_lim = self.curled_bound, self.tensioned_pos

        self.init_pos = copy(self.tensioned_pos)
        self._commanded_pos = copy(self.tensioned_pos)

        # Initialization settings of dxl_client
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.operating_mode,
            ADDR_OPERATING_MODE,
            LEN_OPERATING_MODE,
        )  # Set all motors to current-based position control mode
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.temp_lim,
            ADDR_TEMP_LIMIT,
            LEN_TEMP_LIMIT,
        )  # Set Temp limit
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.curr_lim,
            ADDR_CURRENT_LIMIT,
            LEN_CURRENT_LIMIT,
        )  # Set Current limit
        self.dxl_client.sync_write(
            FINGER_NAMES_TO_MOTOR_IDS["Thumb"],
            np.ones(len(motors)) * 700,
            ADDR_CURRENT_LIMIT,
            LEN_CURRENT_LIMIT,
        )  # Set thumb specific current limit
        self.dxl_client.sync_write(
            motors,
            np.ones(len(motors)) * self.goal_velocity,
            ADDR_GOAL_VELOCITY,
            LEN_GOAL_VELOCITY,
        )  # Set Goal Velocity

        # PID Gains for DIP + PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_P_GAIN,
            ADDR_POSITION_P_GAIN,
            LEN_POSITION_P_GAIN,
        )  # Set P gain for DIP and PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_I_GAIN,
            ADDR_POSITION_I_GAIN,
            LEN_POSITION_I_GAIN,
        )  # Set I gain for DIP and PIP motors
        self.dxl_client.sync_write(
            self.DIP_PIP_motors,
            np.ones(len(motors)) * DIP_PIP_D_GAIN,
            ADDR_POSITION_D_GAIN,
            LEN_POSITION_D_GAIN,
        )  # Set D gain for DIP and PIP motors

        # PID Gains for MCP motors
        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_P_GAIN,
            ADDR_POSITION_P_GAIN,
            LEN_POSITION_P_GAIN,
        )  # Set P gain for MCP motors
        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_I_GAIN,
            ADDR_POSITION_I_GAIN,
            LEN_POSITION_I_GAIN,
        )  # Set I gain for MCP motors
        self.dxl_client.sync_write(
            self.MCP_motors,
            np.ones(len(motors)) * MCP_D_GAIN,
            ADDR_POSITION_D_GAIN,
            LEN_POSITION_D_GAIN,
        )  # Set D gain for MCP motors

        # Enable Torque
        self.dxl_client.set_torque_enabled(True, -1, 0.05)

        # self.dxl_client.set_pos(self.init_pos)

        self.data_recording_functions = {
            "actual_hand_state": self.get_hand_state,
            "commanded_hand_state": self.get_commanded_hand_state,
        }

    def close(self):
        self.dxl_client.disconnect()

    # read any given address for the given motors
    def read_any(self, addr: int, size: int):
        return self.dxl_client.sync_read(self.motors, addr, size)

    # read position
    def read_pos(self):
        # print(f"in read_pos")
        curr_pos = self.dxl_client.read_pos()
        while curr_pos is None:
            curr_pos = self.dxl_client.read_pos()
            time.sleep(0.001)

        return curr_pos

    # read velocity
    def read_vel(self):
        # print(f"in read_vel")
        return self.dxl_client.read_vel()

    # read current
    def read_cur(self):
        return self.dxl_client.read_cur()

    # set pose
    def set_pos(self, pos):
        self._commanded_pos = pos
        self.dxl_client.set_pos(pos)
        return

    def read_temp(self):
        self.dxl_client.sync_read(self.motors, 146, 1)

    @property
    def commanded_pos(self):
        return self._commanded_pos

    @property
    def actual_pos(self):
        pos = self.read_pos()
        while any(item is None for item in pos):
            pos = self.read_pos()
            time.sleep(0.0001)
        return pos

    def get_hand_state(self):

        motor_state = dict(
            position=np.array(self.actual_pos, dtype=np.float32),
            commanded_position=np.array(self.commanded_pos, dtype=np.float32),
            velocity=np.array(self.read_vel(), dtype=np.float32),
            timestamp=time.time(),
        )
        return motor_state

    def get_commanded_hand_state(self):

        motor_state = dict(
            position=np.array(self.commanded_pos, dtype=np.float32),
            timestamp=time.time(),
        )
        return motor_state

    def read_single_cur(self, motor_id):
        cur = self.dxl_client.read_single_cur(motor_id)
        return cur
