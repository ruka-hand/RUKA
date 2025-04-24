import hydra
import torch
import torch.utils.data as data


# Script to return dataloaders
def get_dataloaders(cfg):

    if "all_data_directories" in cfg:
        return get_dataloaders_from_multiple_datasets(cfg)

    train_dset = hydra.utils.instantiate(cfg.dataset)

    if "test_dataset" in cfg:
        test_dset = hydra.utils.instantiate(cfg.test_dataset)
    else:
        train_dset_size = int(len(train_dset) * cfg.train_dset_split)
        test_dset_size = len(train_dset) - train_dset_size

        # Random split the train and validation datasets
        train_dset, test_dset = data.random_split(
            train_dset,
            [train_dset_size, test_dset_size],
            generator=torch.Generator().manual_seed(cfg.seed),
        )

    return return_dataloaders_given_dsets(cfg, train_dset, test_dset)


def get_dataloaders_from_multiple_datasets(cfg):

    train_datasets = []
    for data_dir in cfg.all_data_directories:
        train_datasets.append(hydra.utils.instantiate(cfg.dataset, data_dir=data_dir))
    train_dset = data.ConcatDataset(train_datasets)

    if "test_dataset" in cfg:
        test_dset = hydra.utils.instantiate(cfg.test_dataset)
    else:
        train_dset_size = int(len(train_dset) * cfg.train_dset_split)
        test_dset_size = len(train_dset) - train_dset_size

        # Random split the train and validation datasets
        train_dset, test_dset = data.random_split(
            train_dset,
            [train_dset_size, test_dset_size],
            generator=torch.Generator().manual_seed(cfg.seed),
        )

    return return_dataloaders_given_dsets(cfg, train_dset, test_dset)


def return_dataloaders_given_dsets(cfg, train_dset, test_dset):
    train_loader = data.DataLoader(
        train_dset,
        batch_size=cfg.batch_size,
        shuffle=True,
        num_workers=cfg.num_workers,
    )
    test_loader = data.DataLoader(
        test_dset,
        batch_size=cfg.batch_size,
        shuffle=False,
        num_workers=cfg.num_workers,
    )

    return train_loader, test_loader, train_dset
