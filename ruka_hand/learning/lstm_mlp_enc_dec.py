import os

import torch
import torch.nn as nn
from torch.nn.parallel import DistributedDataParallel as DDP

from ruka_hand.learning.learner import Learner
from ruka_hand.utils.models import create_fc


class MLPDecoder(nn.Module):
    def __init__(
        self,
        input_dim,
        output_dim,
        pred_horizon,
        hidden_dims,
        use_batchnorm,
        dropout,
        is_moco=False,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.net = create_fc(
            input_dim=input_dim,
            output_dim=output_dim * pred_horizon,
            hidden_dims=hidden_dims,
            use_batchnorm=use_batchnorm,
            dropout=dropout,
            is_moco=is_moco,
        )
        self.pred_horizon = pred_horizon

    def forward(self, x):
        flattened = x.reshape(x.shape[0], -1)
        flat_pred = self.net(flattened)
        return flat_pred.reshape(flat_pred.shape[0], self.pred_horizon, -1)


class LSTMEncoder(nn.Module):
    def __init__(
        self,
        input_dim,
        hidden_dim,
        output_dim,
        lstm_hidden_dim,
        nlayers,
        dropout,
        obs_horizon,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.embed = nn.Linear(input_dim, hidden_dim)
        self.lstm = nn.LSTM(
            hidden_dim,
            lstm_hidden_dim,
            num_layers=nlayers,
            batch_first=True,
            dropout=dropout,
        )
        self.decoder = nn.Linear(lstm_hidden_dim, output_dim)
        self.obs_horizon = obs_horizon

    def forward(self, x):
        embed = self.embed(x)
        out, _ = self.lstm(embed.reshape((-1, *embed.shape[-2:])))
        out = self.decoder(out.reshape((*x.shape[:-2], *out.shape[-2:])))
        return out[:, -1:, :]


class LSTMMLPEncDec(Learner):
    def __init__(self, cfg, rank=0, **kwargs):
        self._init_models(cfg, rank)

    def _init_models(self, cfg, rank=0):
        self.encoder = LSTMEncoder(
            input_dim=cfg.net.input_dim,
            hidden_dim=cfg.net.encoder.hidden_dim,
            output_dim=cfg.net.encoder.output_dim,
            lstm_hidden_dim=cfg.net.encoder.lstm_hidden_dim,
            nlayers=cfg.net.encoder.nlayers,
            dropout=cfg.net.encoder.dropout,
            obs_horizon=cfg.net.encoder.obs_horizon,
        )

        self.decoder = MLPDecoder(
            input_dim=cfg.net.encoder.output_dim,
            output_dim=cfg.net.output_dim,
            pred_horizon=cfg.net.decoder.pred_horizon,
            hidden_dims=cfg.net.decoder.hidden_dims,
            use_batchnorm=cfg.net.decoder.use_batchnorm,
            dropout=cfg.net.decoder.dropout,
        )

    def set_optimizer(self, cfg):
        self.optimizer = torch.optim.AdamW(
            params=list(self.encoder.parameters()) + list(self.decoder.parameters()),
            lr=cfg.optimizer.lr,
            weight_decay=cfg.optimizer.weight_decay,
        )
        self.loss_fn = torch.nn.MSELoss(reduction=cfg.learner.loss_fn_reduction)

    def set_distributed(self, rank):
        self.encoder = DDP(
            self.encoder, device_ids=[rank], output_device=rank, broadcast_buffers=False
        )
        self.decoder = DDP(
            self.decoder, device_ids=[rank], output_device=rank, broadcast_buffers=False
        )

    def to(self, device):
        self.device = device
        self.encoder.to(device)
        self.decoder.to(device)

    def train(self):
        self.encoder.train()
        self.decoder.train()

    def eval(self):
        self.encoder.eval()
        self.decoder.eval()

    def save(self, checkpoint_dir, model_type="best"):
        torch.save(
            self.encoder.state_dict(),
            os.path.join(checkpoint_dir, f"encoder_{model_type}.pt"),
            _use_new_zipfile_serialization=False,
        )

        torch.save(
            self.decoder.state_dict(),
            os.path.join(checkpoint_dir, f"decoder_{model_type}.pt"),
            _use_new_zipfile_serialization=False,
        )

    def load(self, checkpoint_dir, training_cfg, device=None, model_type="best"):
        # Init models
        self._init_models(training_cfg)

        # Get the weigths
        encoder_state_dict = torch.load(
            os.path.join(checkpoint_dir, f"encoder_{model_type}.pt"),
            map_location=device,
        )
        decoder_state_dict = torch.load(
            os.path.join(checkpoint_dir, f"decoder_{model_type}.pt"),
            map_location=device,
        )

        self.encoder.load_state_dict(encoder_state_dict)
        self.decoder.load_state_dict(decoder_state_dict)

        num_params = sum(p.numel() for p in self.encoder.parameters()) + sum(
            p.numel() for p in self.decoder.parameters()
        )
        print(f"Loaded LSTM MLP Dec Enc Learner - Total parameters: {num_params}")

    def train_epoch(self, train_loader, epoch, **kwargs):
        self.train()

        # Save the train loss
        train_loss = 0.0

        # Training loop
        for batch in train_loader:
            input_data, output_data = [x.to(self.device) for x in batch]
            self.optimizer.zero_grad()

            # Get the loss by the model
            input_states = self.encoder(input_data)
            pred_output = self.decoder(input_states)
            # print(f"pred_output: {pred_output.shape}, input_data: {input_data.shape}, output_data: {output_data.shape}")
            loss = self.loss_fn(output_data, pred_output)
            train_loss += loss.item()

            # Backprop
            loss.backward()
            self.optimizer.step()

        return train_loss / len(train_loader)

    def test_epoch(self, test_loader, **kwargs):
        self.eval()

        # Save the train loss
        test_loss = 0.0

        # Training loop
        for batch in test_loader:
            input_data, output_data = [x.to(self.device) for x in batch]
            self.optimizer.zero_grad()

            # Get the loss by the model
            with torch.no_grad():
                input_states = self.encoder(input_data)
                pred_output = self.decoder(input_states)
            loss = self.loss_fn(output_data, pred_output)
            test_loss += loss.item()

        return test_loss / len(test_loader)

    def forward(self, input_data):

        input_data = input_data.to(self.device)
        if len(input_data.shape) == 2:
            input_data = input_data.unsqueeze(0)

        # Get the loss by the model
        input_states = self.encoder(input_data)
        pred_output = self.decoder(input_states)

        return pred_output
