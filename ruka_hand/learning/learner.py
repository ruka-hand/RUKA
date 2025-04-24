import os
from abc import ABC

import torch


# Main class for all learner modules
class Learner(ABC):
    def to(self, device):
        self.device = device
        self.net.to(device)

    def train(self):
        self.net.train()

    def eval(self):
        self.net.eval()

    def save(self, checkpoint_dir, model_type="best"):
        torch.save(
            self.net.state_dict(),
            os.path.join(checkpoint_dir, f"model_{model_type}.pt"),
            _use_new_zipfile_serialization=False,
        )

    def load(self, checkpoint_dir, training_cfg, device=None, model_type="best"):
        return NotImplementedError

    def train_epoch(self, train_loader, **kwargs):
        return NotImplementedError

    def test_epoch(self, test_loader, **kwargs):
        return NotImplementedError
