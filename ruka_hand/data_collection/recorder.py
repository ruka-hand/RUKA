from abc import ABC

import h5py
import numpy as np


class Component(ABC):
    # @abstractmethod
    def stream(self):
        # raise NotImplementedError()
        pass

    def notify_component_start(self, component_name):
        print("***************************************************************")
        print("     Starting {} component".format(component_name))
        print("***************************************************************")


class Recorder(Component):
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
