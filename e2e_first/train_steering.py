#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn

from model import E2ESteeringNet


class E2EDataset(Dataset):
    def __init__(
        self,
        dataset_dirs,
        width=320,
        height=180,
        max_steer_deg=30.0,
        max_speed=1.0,
        crop_top_ratio=0.25,
        crop_bottom_ratio=0.0,
    ):
        self.samples = []
        self.width = width
        self.height = height
        self.max_steer_deg = max_steer_deg
        self.max_speed = max_speed
        self.crop_top_ratio = crop_top_ratio
        self.crop_bottom_ratio = crop_bottom_ratio

        for ds_dir in dataset_dirs:
            ds_dir = Path(ds_dir)
            labels_path = ds_dir / "labels.csv"

            if not labels_path.exists():
                raise FileNotFoundError(f"labels.csv not found: {labels_path}")

            with open(labels_path, "r") as f:
                reader = csv.DictReader(f)

                for row in reader:
                    image_path = ds_dir / row["image_path"]

                    if not image_path.exists():
                        continue

                    velocity = float(row["velocity"])

                    if "steering_norm_clipped" in row:
                        steering_norm = float(row["steering_norm_clipped"])
                        steering_norm = max(-1.0, min(1.0, steering_norm))

                    elif "steering_norm" in row:
                        steering_norm = float(row["steering_norm"])
                        steering_norm = max(-1.0, min(1.0, steering_norm))

                    elif "steering_deg" in row:
                        steering_deg = float(row["steering_deg"])
                        steering_norm = steering_deg / max_steer_deg
                        steering_norm = max(-1.0, min(1.0, steering_norm))

                    elif "steering" in row:
                        steering_deg = float(row["steering"])
                        steering_norm = steering_deg / max_steer_deg
                        steering_norm = max(-1.0, min(1.0, steering_norm))

                    else:
                        raise KeyError(
                            "labels.csv must contain one of: "
                            "steering_norm_clipped, steering_norm, steering_deg, steering"
                        )

                    self.samples.append({
                        "image_path": str(image_path),
                        "velocity": velocity,
                        "steering_norm": steering_norm,
                    })

        if len(self.samples) == 0:
            raise RuntimeError("No valid samples found.")

        print(f"Loaded {len(self.samples)} samples from {dataset_dirs}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        item = self.samples[idx]

        image_bgr = cv2.imread(item["image_path"], cv2.IMREAD_COLOR)

        if image_bgr is None:
            raise RuntimeError(f"Failed to read image: {item['image_path']}")

        h, w = image_bgr.shape[:2]

        y1 = int(h * self.crop_top_ratio)
        y2 = int(h * (1.0 - self.crop_bottom_ratio))

        if y2 <= y1:
            y1 = 0
            y2 = h

        image_bgr = image_bgr[y1:y2, :]
        image_bgr = cv2.resize(image_bgr, (self.width, self.height))

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        image = image_rgb.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))

        velocity = item["velocity"] / self.max_speed
        velocity = max(-1.0, min(1.0, velocity))

        steering_norm = item["steering_norm"]

        image = torch.from_numpy(image).float()
        velocity = torch.tensor([velocity], dtype=torch.float32)
        steering = torch.tensor([steering_norm], dtype=torch.float32)

        return image, velocity, steering


def train_one_epoch(model, loader, optimizer, criterion, device):
    model.train()

    total_loss = 0.0
    total_count = 0

    for image, velocity, steering_gt in loader:
        image = image.to(device)
        velocity = velocity.to(device)
        steering_gt = steering_gt.to(device)

        steering_pred = model(image, velocity)
        loss = criterion(steering_pred, steering_gt)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        batch_size = image.size(0)
        total_loss += loss.item() * batch_size
        total_count += batch_size

    return total_loss / max(total_count, 1)


@torch.no_grad()
def evaluate(model, loader, criterion, device, max_steer_deg):
    model.eval()

    total_loss = 0.0
    total_count = 0
    total_abs_error_deg = 0.0

    for image, velocity, steering_gt in loader:
        image = image.to(device)
        velocity = velocity.to(device)
        steering_gt = steering_gt.to(device)

        steering_pred = model(image, velocity)
        loss = criterion(steering_pred, steering_gt)

        abs_error_deg = torch.abs(
            (steering_pred - steering_gt) * max_steer_deg
        )

        batch_size = image.size(0)
        total_loss += loss.item() * batch_size
        total_abs_error_deg += abs_error_deg.sum().item()
        total_count += batch_size

    avg_loss = total_loss / max(total_count, 1)
    mae_deg = total_abs_error_deg / max(total_count, 1)

    return avg_loss, mae_deg


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--train-dirs",
        nargs="+",
        required=True,
        help="Training dataset dirs"
    )

    parser.add_argument(
        "--val-dirs",
        nargs="+",
        required=True,
        help="Validation dataset dirs"
    )

    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-4)

    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=180)

    parser.add_argument("--max-steer-deg", type=float, default=30.0)
    parser.add_argument("--max-speed", type=float, default=1.0)

    parser.add_argument("--crop-top-ratio", type=float, default=0.25)
    parser.add_argument("--crop-bottom-ratio", type=float, default=0.0)

    parser.add_argument("--num-workers", type=int, default=4)

    parser.add_argument(
        "--save-dir",
        default="checkpoints",
        help="Checkpoint save directory"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    train_set = E2EDataset(
        dataset_dirs=args.train_dirs,
        width=args.width,
        height=args.height,
        max_steer_deg=args.max_steer_deg,
        max_speed=args.max_speed,
        crop_top_ratio=args.crop_top_ratio,
        crop_bottom_ratio=args.crop_bottom_ratio,
    )

    val_set = E2EDataset(
        dataset_dirs=args.val_dirs,
        width=args.width,
        height=args.height,
        max_steer_deg=args.max_steer_deg,
        max_speed=args.max_speed,
        crop_top_ratio=args.crop_top_ratio,
        crop_bottom_ratio=args.crop_bottom_ratio,
    )

    pin_memory = True if device.type == "cuda" else False

    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
        drop_last=False,
    )

    val_loader = DataLoader(
        val_set,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
        drop_last=False,
    )

    model = E2ESteeringNet().to(device)

    criterion = nn.SmoothL1Loss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    best_val_loss = 1e9

    for epoch in range(1, args.epochs + 1):
        train_loss = train_one_epoch(
            model=model,
            loader=train_loader,
            optimizer=optimizer,
            criterion=criterion,
            device=device,
        )

        val_loss, val_mae_deg = evaluate(
            model=model,
            loader=val_loader,
            criterion=criterion,
            device=device,
            max_steer_deg=args.max_steer_deg,
        )

        print(
            f"Epoch [{epoch:03d}/{args.epochs}] "
            f"train_loss={train_loss:.6f} "
            f"val_loss={val_loss:.6f} "
            f"val_mae={val_mae_deg:.3f} deg"
        )

        ckpt = {
            "model_state": model.state_dict(),
            "width": args.width,
            "height": args.height,
            "max_steer_deg": args.max_steer_deg,
            "max_speed": args.max_speed,
            "crop_top_ratio": args.crop_top_ratio,
            "crop_bottom_ratio": args.crop_bottom_ratio,
        }

        torch.save(ckpt, save_dir / "last_model.pt")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(ckpt, save_dir / "best_model.pt")
            print(f"Saved best model: {save_dir / 'best_model.pt'}")


if __name__ == "__main__":
    main()

