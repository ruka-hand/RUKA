from ruka_hand.utils.file_ops import load_function


def init_learner(cfg, device, rank=0):
    fn = load_function(f"ruka_hand.utils.initialize_learner.init_{cfg.learner.name}")
    return fn(cfg, device, rank)


def init_lstm_mlp_enc_dec(cfg, device, rank=0, **kwargs):
    from ruka_hand.learning.lstm_mlp_enc_dec import LSTMMLPEncDec

    learner = LSTMMLPEncDec(cfg=cfg, rank=rank)
    learner.to(device)
    learner.set_optimizer(cfg)

    return learner
