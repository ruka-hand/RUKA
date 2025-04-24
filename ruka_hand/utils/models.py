from torch import nn


def create_fc(
    input_dim,
    output_dim,
    hidden_dims,
    use_batchnorm=False,
    dropout=None,
    is_moco=False,
):
    if hidden_dims is None:
        return nn.Sequential(*[nn.Linear(input_dim, output_dim)])

    layers = [nn.Linear(input_dim, hidden_dims[0]), nn.ReLU()]

    if use_batchnorm:
        layers.append(nn.BatchNorm1d(hidden_dims[0]))

    if dropout is not None:
        layers.append(nn.Dropout(p=dropout))

    for idx in range(len(hidden_dims) - 1):
        layers.append(nn.Linear(hidden_dims[idx], hidden_dims[idx + 1]))
        layers.append(nn.ReLU())

        if use_batchnorm:
            layers.append(nn.BatchNorm1d(hidden_dims[idx + 1]))

        if dropout is not None:
            layers.append(nn.Dropout(p=dropout))

    layers.append(nn.Linear(hidden_dims[-1], output_dim))
    if is_moco:
        layers.append(nn.BatchNorm1d(hidden_dims[-1], affine=False))
    return nn.Sequential(*layers)
