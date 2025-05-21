import glob
import os
import pickle
from pathlib import Path

import h5py
import hydra
import numpy as np
import torch
from omegaconf import OmegaConf

from ruka_hand.control.hand import *
from ruka_hand.utils.constants import *
from ruka_hand.utils.data import handle_normalization
from ruka_hand.utils.extract_control_table import df_controlTable
from ruka_hand.utils.file_ops import get_repo_root
from ruka_hand.utils.initialize_learner import init_learner
from ruka_hand.utils.timer import FrequencyTimer
from ruka_hand.utils.vectorops import moving_average


class HandController:
    def __init__(
        self,
        hand_type,
        frequency,
        single_move_len=1,
        device="cpu",
        record=False,
        data_save_dir=None,
    ):
        """
        finger_to_training_dir = {
            "Index": {
                "checkpoint": 'best',
                "multirun_num"None,
                "training_dir"'...'
            },
            "Middle": ...
        },
        finger_to_stats = {
            "Index": {"mean": [...], "std": [...]}
        }
        """

        learner_dict = self._set_learner_dict(hand_type)

        self.device = device
        self._load_learners(learner_dict=learner_dict)
        self._set_input_type()
        self.finger_to_stats = self._load_dataset_stats(learner_dict=learner_dict)

        self.hand = Hand(hand_type)
        self.hand_pos = self.hand.init_pos
        self.record = record
        if record:
            self._recorder_file_name = f"{data_save_dir}/ruka_data.h5"
            self.ruka_data = dict()
            # Input motor limits
            self.ruka_data["max_motor_lim"] = self.hand.max_lim
            self.ruka_data["min_motor_lim"] = self.hand.min_lim

            self._data_names = dict(
                present_position="Present Position",
            )
            self.num_datapoints = 0
            self.record_start_time = time.time()

        self.timer = FrequencyTimer(frequency * single_move_len)
        self.single_move_len = single_move_len
        self.past_observations = dict()
        self.robot_stats = torch.FloatTensor([self.hand.min_lim, self.hand.max_lim])

    def _set_learner_dict(self, hand_type):
        self.checkpoint_dir = os.path.join(get_repo_root(), CHECKPOINT_DIR)
        learner_dict = dict(
            Thumb=f"{self.checkpoint_dir}/{hand_type}_thumb",
            Index=f"{self.checkpoint_dir}/{hand_type}_index",
            Middle=f"{self.checkpoint_dir}/{hand_type}_middle",
            Ring=f"{self.checkpoint_dir}/{hand_type}_ring",
            Pinky=f"{self.checkpoint_dir}/{hand_type}_pinky",
        )

        return learner_dict

    def _load_learners(self, learner_dict):
        self.learners = {}
        self.cfgs = {}
        for key, value in learner_dict.items():
            training_dir = value
            checkpoint = "best"
            cfg = OmegaConf.load(os.path.join(training_dir, ".hydra/config.yaml"))
            model_path = Path(training_dir) / "models"

            # Load the trained model
            learner = init_learner(cfg=cfg, device=self.device)
            learner.load(model_path, training_cfg=cfg, model_type=checkpoint, device=self.device)
            learner.eval()
            learner.to(self.device)

            self.learners[key] = learner
            self.cfgs[key] = cfg

        print(f"Controller Learners: {self.learners}")

    def _set_input_type(self):
        all_input_types = np.array(
            [cfg.dataset.input_type for cfg in self.cfgs.values()]
        )
        if np.all(all_input_types == "joint_angles"):
            self.input_type = "joint_angles"
        elif np.all(all_input_types == "fingertips"):
            self.input_type = "fingertips"
        elif all_input_types[0] == "fingertips" and np.all(
            all_input_types[1:] == "joint_angles"
        ):
            self.input_type = "thumb_special"
        else:
            raise ValueError(
                f"Input type is calculated incorrectly. "
                f"Expected 'joint_angles', 'fingertips', or 'thumb_special', but got: {all_input_types}"
            )

    def _load_dataset_stats(self, learner_dict):

        finger_to_stats = {}
        for finger_name in learner_dict.keys():
            checkpoint_dir = learner_dict[finger_name]
            finger_stats = pickle.load(
                open(os.path.join(checkpoint_dir, "dataset_stats.pkl"), "rb")
            )
            finger_to_stats[finger_name] = finger_stats

        print(f"Finger to Stats: {finger_to_stats}")
        return finger_to_stats

    def reset(self):
        self.move_to_pos(
            curr_pos=self.hand.read_pos(), des_pos=self.hand.tensioned_pos, traj_len=30
        )

    def _process_input(self, input, finger_name):
        # Will look through the learner, depending on if it's sequential / residual or not
        # it will change the input accordingly
        # 1 - if it's sequential it'll add it to its own past observations and return a batch of observations
        cfg = self.cfgs[finger_name]
        if "state_as_input" in cfg.dataset and cfg.dataset.state_as_input:
            motor_ids = FINGER_NAMES_TO_MOTOR_IDS[finger_name]
            curr_motor_pos = torch.FloatTensor(self.hand.read_pos())[motor_ids]

            input = handle_normalization(
                input=input,
                stats=self.finger_to_stats[finger_name][
                    "input"
                ],  # It cannot be a forward model
                normalize=True,
                mean_std=(
                    self.cfgs[finger_name].dataset.fingertip_mean_std_norm
                    if "fingertip_mean_std_norm" in self.cfgs[finger_name].dataset
                    else False
                ),
            )

            motor_norm = handle_normalization(
                input=curr_motor_pos,
                stats=self.finger_to_stats[finger_name]["motor"],
                normalize=True,
                mean_std=False,
            )

            input = torch.cat([input, motor_norm], dim=-1)

        else:
            input = handle_normalization(
                input=input,
                stats=self.finger_to_stats[finger_name][
                    "input"
                ],  # It cannot be a forward model
                normalize=True,
                mean_std=(
                    self.cfgs[finger_name].dataset.fingertip_mean_std_norm
                    if "fingertip_mean_std_norm" in self.cfgs[finger_name].dataset
                    else False
                ),
            )

        if "obs_horizon" in cfg.dataset:  # TODO: Check if this actually works
            if not finger_name in self.past_observations:
                self.past_observations[finger_name] = input.repeat(
                    cfg.dataset.obs_horizon
                ).reshape(-1, input.shape[0])

            else:

                self.past_observations[finger_name] = torch.cat(
                    [
                        torch.roll(
                            self.past_observations[finger_name], shifts=-1, dims=0
                        )[:-1, :],
                        input.unsqueeze(0),
                    ],
                    dim=0,
                )

            input = self.past_observations[finger_name]

        return input

    def _process_output(self, output, finger_name, weighted_average=False):
        # Will look through the learner, depending on if it's residual or not
        # it will 1) change the motor output to be residual
        # 2) if there is prediction horizon, then it'll move to the first one - or like weighted average ?
        cfg = self.cfgs[finger_name]
        if "pred_horizon" in cfg.dataset:
            if weighted_average:
                pass  # TODO: Implement weighted average with respect to the prediction horizon
            else:
                output = output[0, :]

        motor_ids = FINGER_NAMES_TO_MOTOR_IDS[finger_name]
        if "predict_residual" in cfg.dataset and cfg.dataset.predict_residual:
            # Get the current motor position and add the output to that
            curr_motor_pos = np.array(self.hand.read_pos())[motor_ids]
            output = curr_motor_pos + output

        output = np.clip(output, 0, 4000)
        for i in range(len(motor_ids)):
            self.hand_pos[motor_ids[i]] = output[i]

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

    def move_to_pos(
        self,
        curr_pos,
        des_pos,
        traj_len=50,
    ):
        if traj_len == 1:
            self.hand.set_pos(des_pos)
            self.hand_pos = des_pos
            if self.record:
                self._update_ruka_data(commanded_position=des_pos)
            return

        trajectory = np.linspace(curr_pos, des_pos, traj_len)[
            1:
        ]  # Don't include the first pose

        for hand_pos in trajectory:
            self.timer.start_loop()

            self.hand.set_pos(hand_pos)
            self.hand_pos = hand_pos

            if self.record:
                self._update_ruka_data(commanded_position=hand_pos)

            self.timer.end_loop()

    def step(self, input_data, moving_average_info=None, move=True):
        # input_data: (5,3) - 5: fingers, 3: input_dim
        input_data = torch.FloatTensor(input_data)
        # times_passed = []

        for finger_name in self.learners.keys():
            learner = self.learners[finger_name]

            finger_id = FINGER_NAMES_TO_MANUS_IDS[finger_name]
            motor_ids = FINGER_NAMES_TO_MOTOR_IDS[finger_name]

            model_input = input_data[finger_id, :]  # (3)

            model_input = self._process_input(
                input=model_input, finger_name=finger_name
            )

            pred_motor_pos = learner.forward(model_input).detach().cpu()[0]

            robot_stats = torch.stack(
                [self.robot_stats[0][motor_ids], self.robot_stats[1][motor_ids]]
            )
            pred_motor_pos = handle_normalization(
                input=pred_motor_pos, stats=robot_stats, normalize=False, mean_std=False
            )

            self._process_output(
                output=pred_motor_pos, finger_name=finger_name, weighted_average=False
            )

        if not moving_average_info is None:
            self.hand_pos = moving_average(
                self.hand_pos,
                moving_average_info["queue"],
                moving_average_info["limit"],
            )

        # before_step = time.time()
        if move:
            self.move_to_pos(
                curr_pos=self.hand.read_pos(),
                des_pos=self.hand_pos,
                traj_len=self.single_move_len,
            )
        else:
            return self.hand_pos

    def move(self, wanted_pos):
        self.move_to_pos(
            curr_pos=self.hand.read_pos(),
            des_pos=wanted_pos,
            traj_len=self.single_move_len,
        )

    def _add_metadata(self, datapoints):
        self.metadata = dict(
            file_name=self._recorder_file_name,
            num_datapoints=datapoints,
            record_start_time=self.record_start_time,
            record_end_time=self.record_end_time,
            record_duration=self.record_end_time - self.record_start_time,
            record_frequency=datapoints
            / (self.record_end_time - self.record_start_time),
        )

    def _display_statistics(self, datapoints):
        print("Saving data to {}".format(self._recorder_file_name))
        print("Number of datapoints recorded: {}.".format(datapoints))
        print(
            "Data record frequency: {}.".format(
                datapoints / (self.record_end_time - self.record_start_time)
            )
        )

    def _compress_data(self, data_dict):
        self._display_statistics(self.num_datapoints)
        self._add_metadata(self.num_datapoints)

        # Writing to dataset
        print("Compressing keypoint data...")
        with h5py.File(self._recorder_file_name, "w") as file:
            # Main data
            for key in data_dict.keys():
                if key != "timestamp":
                    data_dict[key] = np.array(data_dict[key], dtype=np.float32)
                else:
                    data_dict["timestamp"] = np.array(
                        data_dict["timestamp"], dtype=np.float64
                    )

                file.create_dataset(
                    key + "s",
                    data=data_dict[key],
                    compression="gzip",
                    compression_opts=6,
                )

            # Other metadata
            file.update(self.metadata)

    def close(self):
        if self.record:
            print(f"** ROBOT SAVING DONE IN {self._recorder_file_name}")
            self.record_end_time = time.time()
            self._compress_data(data_dict=self.ruka_data)
            print("Saved manus_data data in {}.".format(self._recorder_file_name))
        self.hand.close()
