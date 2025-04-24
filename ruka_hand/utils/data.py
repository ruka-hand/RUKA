import numpy as np
import torch

from ruka_hand.learning.preprocessor import Preprocessor


def handle_normalization(input, stats, normalize, mean_std=False):
    if mean_std:
        mean, std = stats[0], stats[1]
        if normalize:
            input = (input - mean) / (std + 1e-10)
        else:
            input = (input * std + mean).numpy()
    else:
        min, max = stats[0], stats[1]
        if normalize:
            if isinstance(input, np.ndarray):
                input = np.clip(input, min, max)
            else:
                input = torch.clamp(input, min, max)
            input = (input - min) / (max - min + 1e-10)
        else:
            input = input * (max - min) + min
            if not isinstance(input, np.ndarray):
                input = input.numpy()

    return input


def preprocess(save_dirs):
    preprocessor = Preprocessor(
        save_dirs=save_dirs,
        frequency=-1,
        module_keys=["manus", "ruka"],
        visualize=False,
    )

    processes = preprocessor.get_processes()
    for process in processes:
        process.start()

    for process in processes:
        process.join()
