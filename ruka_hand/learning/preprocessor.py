import os
from multiprocessing import Process

import h5py
import numpy as np


class DataPreprocessor:
    def __init__(self, data_file_name, data_keys, time_difference=None):
        # For motor data data_keys would be present_position and commanded_position
        # For manus data data_keys would be fingertips and joint_angles

        self.load_file_name = f"{data_file_name}.h5"
        self.dump_file_name = f"{data_file_name}.npy"
        self.data_keys = data_keys
        self.data_file_name = data_file_name
        self.time_difference = time_difference

        self.current_id = 0
        self.indices = []

    def __repr__(self):
        return f"{self.data_file_name}_preprocessor"

    @property
    def current_timestamp(self):
        return self.data["timestamps"][self.current_id]

    def update_root(self, root):
        self.root = root
        self.indices = []
        self.load_data()
        print(
            "Updated the root of PreprocessorModule ({}) for root: {}".format(
                self, root
            )
        )
        self.reset_current_id()

    def reset_current_id(self):
        self.current_id = 0

    def update_indices(self):
        self.indices.append(self.current_id)

    def is_finished(self):
        if self.current_id >= len(self.data["timestamps"]) - 1:
            return True

        return False

    def update_next_id(self, desired_timestamp):
        self.current_id = self._get_closest_id(desired_timestamp)

    def get_next_timestamp(self):
        curr_ts = self.current_timestamp

        if self.time_difference != -1:
            desired_ts = curr_ts + self.time_difference
            next_id = self._get_closest_id(desired_ts)
            return self.data["timestamps"][next_id]
        else:
            return self.data["timestamps"][self.current_id + 1]

    def load_data(self):
        file_path = os.path.join(self.root, self.load_file_name)
        self.data = dict()
        with h5py.File(file_path, "r") as file:
            print(f"path: {file_path}, file.keys(): {file.keys()}")
            for key in self.data_keys:
                key_data = np.array(file[key])
                self.data[key] = key_data

            self.data["timestamps"] = np.array(file["timestamps"])
            if "ruka" in self.load_file_name:
                self.data["max_motor_lims"] = np.array(file["max_motor_lims"])
                self.data["min_motor_lims"] = np.array(file["min_motor_lims"])

        print(
            f"** LOADED DATA FROM {self.load_file_name} - KEYS LOADED: {self.data.keys()}"
        )

    def dump_data(self):
        print(f"-------------- dumping in {self} ------------ ")
        for key in self.data.keys():
            print(f"pre precessing {key}: {self.data[key].shape}")
            if key != "max_motor_lims" and key != "min_motor_lims":
                prep_data = self.data[key][self.indices]
                num_nan_counts = 0
                for i in range(prep_data.shape[0]):
                    if np.isnan(prep_data[i]).any():
                        num_nan_counts += 1
                np.save(f"{self.root}/{key}.npy", prep_data)
                print(
                    f"Dumped {key}.npy - Shape: {prep_data.shape} - NAN counts: {num_nan_counts}"
                )
            else:
                np.save(f"{self.root}/{key}.npy", self.data[key])
                print(f"Dumped {key}.npy - Shape: {self.data[key].shape}")

        print(f"-------------- done in {self} ------------ ")

    def _get_closest_non_nan_id(self, curr_id):
        if "manus" in self.load_file_name:
            key_name = "joint_angles"
        elif "ruka" in self.load_file_name:
            key_name = "present_positions"
        else:
            return curr_id  # NOTE: We don't check for isnan for other datas
        for i in range(curr_id, len(self.data[key_name])):

            if np.isnan(self.data[key_name][i]).any():
                continue
            else:
                break

        return i

    def _get_closest_id(self, desired_ts):
        for i in range(self.current_id, len(self.data["timestamps"])):
            if self.data["timestamps"][i] >= desired_ts:
                return self._get_closest_non_nan_id(i)

        return i


class Preprocessor:
    def __init__(
        self,
        save_dirs,
        frequency,
        module_keys=["manus", "ruka", "right_arm", "left_arm"],
    ):
        self.save_dirs = save_dirs
        self.time_difference = 1 / frequency
        self.frequency = frequency

        self._start_modules(module_keys)

    def _start_modules(self, module_keys):
        self._module_dict = dict(
            manus=DataPreprocessor(
                data_file_name="manus_data",
                data_keys=["fingertips", "joint_angles", "keypoints"],
                time_difference=self.time_difference,
            ),
            ruka=DataPreprocessor(
                data_file_name="ruka_data",
                data_keys=[
                    "present_positions",
                    "commanded_positions",
                ],
                time_difference=self.time_difference,
            ),
        )
        self.modules = dict()
        for key in module_keys:
            self.modules[key] = self._module_dict[key]

    def _preprocess_single_dir(self, save_dir):
        print(f"** PREPROCESSING ** {save_dir}")
        self._update_root(save_dir)
        self._dump_data()
        print(f"** PREPROCESSING DONE IN {save_dir} **")

    def get_processes(self):

        self.processes = []
        for (
            save_dir
        ) in self.save_dirs:  # Preprocess each directory in a different process
            self.processes.append(
                Process(target=self._preprocess_single_dir, args=(save_dir,))
            )

        return self.processes

    def _reset_indices(self):
        for module in self.modules.values():
            module.reset_current_id()

    def _update_root(self, root):
        for module in self.modules.values():
            module.update_root(root)

    def _find_latest_module(self):
        latest_ts = 0
        module_key = None
        for key, module in self.modules.items():
            current_ts = module.current_timestamp
            if current_ts > latest_ts:
                latest_ts = current_ts
                module_key = key

        return module_key

    def _dump_data(self):
        # TODO: add shorteninig part

        self._reset_indices()

        latest_key = self._find_latest_module()
        metric_timestamp = self.modules[latest_key].current_timestamp

        # Find the beginning ids for each module
        for module in self.modules.values():
            module.update_next_id(metric_timestamp)
            module.update_indices()

            print(f"{module} - ts: {module.current_timestamp}")

        # Update timestamps and ids consequitively
        while True:

            # Each module returns a 'metric' timestamp
            # We will choose the closest timestamp to the curr_ts as the metric timestamp
            # If the module is not selective (for ex touch sensors are not important for preprocesing
            # because they save data in very high frequency) they will return -1
            module_ts_diff = (
                1e3 if self.time_difference != -1 else -1e3
            )  # If time difference is -1 we'll only sync to the smallest dataset
            cand_metric_ts = metric_timestamp
            for key, module in self.modules.items():
                next_ts = module.get_next_timestamp()

                candidate_check = (
                    next_ts - metric_timestamp < module_ts_diff
                    if self.time_difference > -1
                    else next_ts - metric_timestamp > module_ts_diff
                )
                if next_ts != -1 and candidate_check:
                    module_ts_diff = next_ts - metric_timestamp
                    cand_metric_ts = next_ts

            if cand_metric_ts == metric_timestamp:  # If it hasn't changed at all
                break
            metric_timestamp = cand_metric_ts

            # Update the ids of each module
            for module in self.modules.values():
                module.update_next_id(metric_timestamp)

            # Check if the loop should be completed or not
            finished = np.array(
                [module.is_finished() for module in self.modules.values()]
            ).any()
            if finished:
                break

            # If not add update the indices array of each module
            for module in self.modules.values():
                module.update_indices()

        for module in self.modules.values():
            module.dump_data()
