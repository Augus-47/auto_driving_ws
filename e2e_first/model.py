#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import torch
import torch.nn as nn


class E2ESteeringNet(nn.Module):
    """
    Input:
      image:    [B, 3, H, W]
      velocity: [B, 1]

    Output:
      steering_norm: [B, 1], range roughly [-1, 1]
    """

    def __init__(self):
        super().__init__()

        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.BatchNorm2d(24),
            nn.ReLU(),

            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.BatchNorm2d(36),
            nn.ReLU(),

            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(),

            nn.Conv2d(48, 64, kernel_size=3, stride=2),
            nn.BatchNorm2d(64),
            nn.ReLU(),

            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),

            nn.AdaptiveAvgPool2d((1, 1))
        )

        self.velocity_mlp = nn.Sequential(
            nn.Linear(1, 16),
            nn.ReLU(),
            nn.Linear(16, 16),
            nn.ReLU()
        )

        self.head = nn.Sequential(
            nn.Linear(64 + 16, 128),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(128, 64),
            nn.ReLU(),

            nn.Linear(64, 1),
            nn.Tanh()
        )

    def forward(self, image, velocity):
        img_feat = self.cnn(image)
        img_feat = img_feat.flatten(1)

        vel_feat = self.velocity_mlp(velocity)

        feat = torch.cat([img_feat, vel_feat], dim=1)
        steering_norm = self.head(feat)

        return steering_norm

