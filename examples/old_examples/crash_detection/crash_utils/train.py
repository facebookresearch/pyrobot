# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse
import copy
import os
import pickle
import time

import numpy as np
import torch
from IPython import embed
from tensorboardX import SummaryWriter
from torch import nn, optim
from torch.optim import lr_scheduler
from torch.utils.data import DataLoader

from data_loader import DroneCrashDataset
from model import CrashDetectorNet

use_gpu = torch.cuda.is_available()

ROOT_DIR = "/home/senthil/projects/drone/control_data"
BATCH_SIZE = 128
lr = 0.0004
momentum = 0.9


class Trainer(object):
    def __init__(self, csv_filename, train_f=0.8, **kwargs):
        D_train = DroneCrashDataset(
            csv_filename, ROOT_DIR, train_fraction=train_f, train=True
        )
        D_test = DroneCrashDataset(
            csv_filename, ROOT_DIR, train_fraction=train_f, train=False
        )
        self.train_dataloader = DataLoader(
            D_train, batch_size=BATCH_SIZE, shuffle=True, num_workers=2
        )
        self.test_dataloader = DataLoader(
            D_test, batch_size=BATCH_SIZE, shuffle=True, num_workers=2
        )
        self.create_model()
        params = []
        for key, value in dict(self.model.named_parameters()).items():
            if value.requires_grad:
                params += [{"params": [value]}]
        self.optimizer = optim.Adam(params, lr=lr)
        self.optimizer.zero_grad()
        self.lr_scheduler = lr_scheduler.StepLR(self.optimizer, step_size=7, gamma=0.1)
        Wneg = 0.8
        self.class_weight = torch.FloatTensor([Wneg, 1 - Wneg])
        if use_gpu:
            self.class_weight = self.class_weight.cuda()
        self.criterion = nn.CrossEntropyLoss(weight=self.class_weight)

    def create_model(self):
        self.model = CrashDetectorNet()
        if use_gpu:
            self.model = self.model.cuda()

    def make_cuda_batch(self, batch):
        if use_gpu:
            batch["image"] = batch["image"].cuda()
            batch["label"] = batch["label"].long().cuda().squeeze()
        return batch

    def train_model(self, num_epochs=25, save_path=None, save_every=10, log_dir=None):
        writer = SummaryWriter(log_dir)
        since = time.time()
        best_model_wts = copy.deepcopy(self.model.state_dict())
        best_acc = 0.0
        metrics = {}
        phases = ["test", "train"]
        for phase in phases:
            metrics[phase] = {}
            for metric in ["loss", "accuracy"]:
                metrics[phase][metric] = []

        for epoch in range(num_epochs):
            print("Epoch {}/{}".format(epoch, num_epochs - 1))
            print("-" * 10)
            for phase in phases:
                if phase == "train":
                    dataloader = self.train_dataloader
                    self.lr_scheduler.step()
                    self.model.train(True)
                    n_cycles = 30
                else:
                    dataloader = self.test_dataloader
                    self.model.train(False)
                    n_cycles = 5

                running_loss = 0.0
                running_accuracy = 0
                loss_stats = np.array([])
                acc_stats = np.array([])
                batch_losses = []
                for batch_id, sample_batched in enumerate(dataloader):
                    sample_batched = self.make_cuda_batch(sample_batched)
                    if batch_id % 5 == 0:
                        print(
                            "[{}] Epoch: {}, Batch: {}/{}".format(
                                phase, epoch, batch_id + 1, n_cycles
                            )
                        )
                    inputs, labels = sample_batched["image"], sample_batched["label"]
                    outputs = self.model(x=inputs)
                    loss = self.criterion(outputs, labels)
                    conf = (outputs[:, 1] > 0.5) == labels.byte()
                    accuracy = conf.float().mean()
                    if phase in ["train"]:
                        loss.backward()
                        self.optimizer.step()
                        self.optimizer.zero_grad()
                    loss_stats = np.concatenate([loss_stats, [loss.cpu().data.numpy()]])
                    acc_stats = np.concatenate(
                        [acc_stats, [accuracy.cpu().data.numpy()]]
                    )
                    running_loss += loss.item()
                    running_accuracy += accuracy.item()
                    if (batch_id + 1) % n_cycles == 0:
                        break
                epoch_loss = running_loss / n_cycles
                epoch_acc = running_accuracy / n_cycles
                metrics[phase]["loss"].append(epoch_loss)
                metrics[phase]["accuracy"].append(epoch_acc)
                print(
                    "{} Loss: {:.4f} Acc: {:.4f}".format(phase, epoch_loss, epoch_acc)
                )
                writer.add_scalar("{}/Loss".format(phase), epoch_loss, epoch)
                writer.add_scalar("{}/Accuracy".format(phase), epoch_acc, epoch)
                if phase == "test" and epoch_acc > best_acc:
                    print("NEW BEST MODEL FOUND!!")
                    best_acc = epoch_acc
                    best_model_wts = copy.deepcopy(self.model.state_dict())
            if epoch % save_every == 0 and epoch > 0:
                torch.save(self.model.cpu(), save_path + ".{}".format(epoch))
                if use_gpu:
                    self.model.cuda()
            if epoch == num_epochs - 1:
                torch.save(self.model.cpu(), save_path)
                if use_gpu:
                    self.model.cuda()
        writer.close()
        self.model.load_state_dict(best_model_wts)
        return self.model, metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", default="./models/")
    parser.add_argument("--log_dir", default="./logs/")
    parser.add_argument("--ID", default="1_2")
    parser.add_argument(
        "--csv_filename", default="control_data_0_0_0#1_0_control_data_1_1_0#2_0.csv"
    )
    parser.add_argument("--n_epochs", type=int, default=100)
    parser.add_argument("--save_every", type=int, default=10)
    parser.add_argument("--train_f", type=float, default=0.8)
    args = parser.parse_args()
    model_dir = args.model_dir
    csv_filename = args.csv_filename
    log_dir = args.log_dir
    n_epochs = args.n_epochs
    save_every = args.save_every
    ID = args.ID
    train_f = args.train_f
    model_dir = "{}/{}_EPS_{}".format(model_dir, ID, n_epochs)
    log_dir = "{}/{}_EPS_{}".format(log_dir, ID, n_epochs)
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)
    save_path = os.path.join(model_dir, "checkpoint.pth")
    metric_path = os.path.join(model_dir, "training_info.p")
    T = Trainer(csv_filename=csv_filename, train_f=train_f)
    M, metrics = T.train_model(
        num_epochs=n_epochs, save_path=save_path, save_every=save_every, log_dir=log_dir
    )
    torch.save(M.cpu(), save_path + ".bst")
    pickle.dump(metrics, open(metric_path, "wb"))
    embed()


if __name__ == "__main__":
    main()
