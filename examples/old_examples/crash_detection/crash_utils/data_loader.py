# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import print_function, division

import os

import numpy as np
import pandas as pd
import torch
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms

PYTORCH_TRANSFORM = transforms.Compose(
    [
        transforms.RandomResizedCrop(
            size=(256, 256), scale=(0.9, 1.0), ratio=(0.8, 1.2), interpolation=2
        ),
        transforms.Resize(size=(224, 224)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.2, hue=0.2),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ]
)


class DroneCrashDataset(Dataset):
    """ Drone Crash dataset"""

    def __init__(
        self,
        csv_file,
        root_dir,
        train=True,
        train_fraction=0.8,
        transform=PYTORCH_TRANSFORM,
    ):
        """
        """
        self.root_dir = root_dir
        self.csv_file = os.path.join(self.root_dir, csv_file)
        self.frame = pd.read_csv(self.csv_file, header=None)
        n_total = len(self.frame)
        drop_fraction = 1.0 - train_fraction
        n_train = int(n_total * drop_fraction)
        n_test = n_total - n_train
        if train == True:
            self.frame = self.frame.drop(
                list(np.linspace(0, n_train - 1, num=n_train, dtype=np.int64))
            )
        else:
            self.frame = self.frame.drop(
                list(np.linspace(n_train, n_total - 1, num=n_test, dtype=np.int64))
            )

        self.transform = transform

    def __len__(self):
        return len(self.frame)

    def __getitem__(self, idx):
        img_name = os.path.join(self.root_dir, self.frame.iloc[idx, 0])
        image = Image.open(img_name)
        label = self.frame.iloc[idx, 1]
        sample = {
            "image": image,
            "label": torch.Tensor([label]),
        }
        if self.transform:
            sample["image"] = self.transform(sample["image"])

        return sample
