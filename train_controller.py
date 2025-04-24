import glob
import os

import hydra
import torch
import wandb
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf
from tqdm import tqdm

from ruka_hand.learning.dataloaders import get_dataloaders
from ruka_hand.utils.initialize_learner import init_learner
from ruka_hand.utils.logger import Logger

# TODO: Add dataset_stats.pkl dumping here!!!


class Workspace:
    def __init__(self, cfg: DictConfig) -> None:
        print(f"Workspace config: {OmegaConf.to_yaml(cfg)}")

        try:
            self.hydra_dir = (
                f"{HydraConfig.get().sweep.dir}/{HydraConfig.get().sweep.subdir}"
            )
        except:
            print(f"in exception")
            self.hydra_dir = HydraConfig.get().run.dir

        self.checkpoint_dir = os.path.join(self.hydra_dir, "models")

        # Create the checkpoint directory - it will be inside the hydra directory
        os.makedirs(
            self.checkpoint_dir, exist_ok=True
        )  # Doesn't give an error if dir exists when exist_ok is set to True
        os.makedirs(self.hydra_dir, exist_ok=True)

        # Set device and config
        self.cfg = cfg

    def train(self) -> None:
        device = torch.device(self.cfg.device)

        # It looks at the datatype type and returns the train and test loader accordingly
        train_loader, test_loader, dataset = get_dataloaders(self.cfg)

        # Initialize the learner - looks at the type of the agent to be initialized first
        self.cfg.net.input_dim = dataset[0][0].shape[-1]
        self.cfg.net.output_dim = dataset[0][1].shape[-1]
        print(
            f"OUTPUT DIM: {self.cfg.net.output_dim}, INPUT DIM: {self.cfg.net.input_dim}"
        )
        with open(f"{self.hydra_dir}/.hydra/config.yaml", "w") as f:
            OmegaConf.save(
                self.cfg, f
            )  # Save the input / output dim to the config so that you can load it directly
        learner = init_learner(cfg=self.cfg, device=device)

        best_loss = torch.inf
        pbar = tqdm(total=self.cfg.train_epochs)

        # Initialize logger (wandb)
        if self.cfg.log:
            wandb_exp_name = "-".join(self.hydra_dir.split("/")[-2:])
            print("wandb_exp_name: {}".format(wandb_exp_name))
            self.logger = Logger(self.cfg, wandb_exp_name, out_dir=self.hydra_dir)
        else:
            self.logger = None

        # Start the training
        for epoch in range(self.cfg.train_epochs):

            # Train the models for one epoch
            train_loss = learner.train_epoch(train_loader, epoch)

            pbar.set_description(
                f"Epoch {epoch}, Train loss: {train_loss:.5f}, Best loss: {best_loss:.5f}"
            )
            pbar.update(1)

            # Logging
            if self.cfg.log and epoch % self.cfg.log_frequency == 0:
                self.logger.log({"epoch": epoch, "train loss": train_loss})

            # Testing and saving the model
            if epoch % 10 == 0:
                if epoch % self.cfg.save_frequency == 0:
                    learner.save(self.checkpoint_dir, model_type=epoch)

                # Test for one epoch
                test_loss = learner.test_epoch(test_loader)

                # Save if it's the best loss
                if test_loss < best_loss:
                    best_loss = test_loss
                    learner.save(self.checkpoint_dir, model_type="best")

                # Logging
                pbar.set_description(f"Epoch {epoch}, Test loss: {test_loss:.5f}")
                if self.cfg.log:
                    self.logger.log({"epoch": epoch, "test loss": test_loss})
                    self.logger.log({"epoch": epoch, "best loss": best_loss})

        pbar.close()
        wandb.finish()


@hydra.main(
    version_base=None,
    config_path="configs",
    config_name="train_controller",
)
def main(cfg: DictConfig) -> None:
    workspace = Workspace(cfg)
    workspace.train()


if __name__ == "__main__":
    main()
