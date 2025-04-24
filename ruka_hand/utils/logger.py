import os

import wandb
from omegaconf import DictConfig, OmegaConf


# Class for the wandb logger
class Logger:
    def __init__(self, cfg: DictConfig, exp_name: str, out_dir: str) -> None:
        # Initialize the wandb experiment
        self.wandb_logger = wandb.init(
            project=cfg.wandb_project_name,
            name=exp_name,
            dir=cfg.work_dir,
            config=OmegaConf.to_container(cfg, resolve=True),
            settings=wandb.Settings(start_method="thread"),
        )

        self.logger_file = os.path.join(out_dir, "train.log")

    def log(self, msg):
        if type(msg) is dict:
            self.wandb_logger.log(msg)

        with open(self.logger_file, "a") as f:
            f.write("{}\n".format(msg))

    def log_metrics(self, metrics, time_step, time_step_name):
        for key in metrics.keys():
            msg = {time_step_name: time_step, key: metrics[key]}
            self.log(msg)
