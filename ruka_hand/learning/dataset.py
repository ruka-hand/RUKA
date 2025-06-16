# Cleaner dataset
import os
from copy import deepcopy as copy
from datetime import datetime

import numpy as np
import torch
import torch.utils.data as data
from tqdm import tqdm

from ruka_hand.utils.constants import *
from ruka_hand.utils.data import handle_normalization, preprocess


class AllHandDataMotor(data.Dataset):
    def __init__(
        self,
        data_dir,
        finger_name,
        model_type="inverse",
        input_type="fingertips",
        motor_data_type="present",
        subsample=False,
        obs_horizon=1,
        pred_horizon=1,
        predict_residual=False,
        state_as_input=False,
        fingertip_mean_std_norm=False,
    ):
        print(
            f"{data_dir}/{motor_data_type}_positions.npy -- {data_dir}/{input_type}.npy"
        )

        if not (
            os.path.exists(f"{data_dir}/{input_type}.npy")
            and os.path.exists(f"{data_dir}/{motor_data_type}_positions.npy")
        ):
            preprocess([data_dir])

        self.motor_positions = np.load(f"{data_dir}/{motor_data_type}_positions.npy")
        self.input_data = np.load(f"{data_dir}/{input_type}.npy")

        print(f"finger_name: {finger_name} - data_dir: {data_dir}")
        if not finger_name is None:
            self.manus_ids = FINGER_NAMES_TO_MANUS_IDS[finger_name]
            self.input_data = self.input_data[:, self.manus_ids, :]

            self.motor_ids = FINGER_NAMES_TO_MOTOR_IDS[finger_name]
            self.motor_positions = self.motor_positions[:, self.motor_ids]

        print(
            f"PRE SAMPLING SHAPES: {self.input_data.shape} | {self.motor_positions.shape}"
        )
        if subsample:

            self.input_data, self.motor_positions = self._subsample_waiting(
                input_data=self.input_data,
                motor_data=self.motor_positions,
                data_dir=data_dir,
                finger_name=finger_name,
            )
            print(
                f"POST SAMPLING SHAPES: {self.input_data.shape} | {self.motor_positions.shape}"
            )

        self.motor_positions = torch.FloatTensor(self.motor_positions)
        self.data_dir = data_dir
        self.input_data = torch.FloatTensor(self.input_data)
        self.predict_residual = predict_residual
        self.state_as_input = state_as_input
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.model_type = model_type
        self.input_type = input_type

        self._calculate_stats(fingertip_mean_std_norm)

    def __len__(self):
        return self.input_data.shape[0] - self.obs_horizon - self.pred_horizon + 1

    def _subsample_waiting(self, input_data, motor_data, data_dir, finger_name):
        # Load the commanded motor positions
        motor_ids = FINGER_NAMES_TO_MOTOR_IDS[finger_name]
        commanded_motor_positions = np.load(f"{data_dir}/commanded_positions.npy")[
            :, motor_ids
        ]
        print("SUBSAMBLING WAITING ELEMENTS")
        pbar = tqdm(total=commanded_motor_positions.shape[0])
        i, indices = 1, []
        while i < commanded_motor_positions.shape[0] - 1:
            # Find the places where the motor positions are the same then find the last element where it's the same
            prev_val = commanded_motor_positions[i - 1]
            curr_val = commanded_motor_positions[i]
            nex_val = commanded_motor_positions[i + 1]

            if (prev_val == curr_val).all() and (nex_val != curr_val).any():
                indices.append(i)
            i += 1
            pbar.update(1)

        indices = np.array(indices)
        pbar.close()

        return input_data[indices, :], motor_data[indices, :]

    def _calculate_stats(self, fingertip_mean_std_norm=False):

        self.stats = {}

        if os.path.exists(f"{self.data_dir}/max_motor_lims.npy"):
            motor_max = torch.FloatTensor(
                np.load(f"{self.data_dir}/max_motor_lims.npy")[self.motor_ids]
            )
            motor_min = torch.FloatTensor(
                np.load(f"{self.data_dir}/min_motor_lims.npy")[self.motor_ids]
            )
        else:
            motor_min, motor_max = (
                self.motor_positions.min(dim=0)[0],
                self.motor_positions.max(dim=0)[0],
            )
        self.stats["motor"] = [motor_min, motor_max]

        # Read the fingertip stats from the input values
        if fingertip_mean_std_norm:
            self.stats["input"] = [
                torch.FloatTensor(self.input_data.mean(dim=0)),
                torch.FloatTensor(self.input_data.std(dim=0)),
            ]
        else:
            self.stats["input"] = [
                torch.FloatTensor(self.input_data.min(dim=0)[0]),
                torch.FloatTensor(self.input_data.max(dim=0)[0]),
            ]

        self.fingertip_mean_std_norm = fingertip_mean_std_norm
        print(
            f"INPUT STATS: {self.stats['input']} | MOTOR STATS: {self.stats['motor']}"
        )

    def get_stats(self):

        input_stats = copy(self.stats["input"])
        motor_stats = copy(self.stats["motor"])

        return {
            "input": input_stats,
            "motor": motor_stats,
        }

    def __getitem__(self, i):

        curr_input_data = handle_normalization(
            self.input_data[i : i + self.obs_horizon],
            stats=self.stats["input"],
            normalize=True,
            mean_std=self.fingertip_mean_std_norm,
        )

        curr_motor_pos = handle_normalization(
            self.motor_positions[i : i + self.obs_horizon],
            stats=self.stats["motor"],
            normalize=True,
            mean_std=self.fingertip_mean_std_norm,
        )

        next_input_data = handle_normalization(
            self.input_data[
                i + self.obs_horizon : i + self.obs_horizon + self.pred_horizon
            ],
            stats=self.stats["input"],
            normalize=True,
        )
        pre_intput_data = handle_normalization(
            self.input_data[
                i + self.obs_horizon - self.pred_horizon : i + self.obs_horizon
            ],
            stats=self.stats["input"],
            normalize=True,
        )
        next_motor_pos = handle_normalization(
            self.motor_positions[
                i + self.obs_horizon : i + self.obs_horizon + self.pred_horizon
            ],
            stats=self.stats["motor"],
            normalize=True,
        )
        prev_motor_pos = handle_normalization(
            self.motor_positions[
                i + self.obs_horizon - self.pred_horizon : i + self.obs_horizon
            ],
            stats=self.stats["motor"],
            normalize=True,
        )
        

        if self.model_type == "forward":  # input is the motors
            if self.state_as_input:
                input_data = torch.concat([curr_motor_pos, curr_input_data], dim=-1)
            else:
                input_data = curr_motor_pos

            if self.predict_residual:
                output_data = next_input_data - pre_intput_data
            else:
                output_data = next_input_data

        else:  # input is the fingertips
            if self.state_as_input:
                input_data = torch.concat([curr_input_data, curr_motor_pos], dim=-1)
            else:
                input_data = curr_input_data

            if self.predict_residual:
                output_data = next_motor_pos - prev_motor_pos
            else:
                output_data = next_motor_pos

        return input_data, output_data
