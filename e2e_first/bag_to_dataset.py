#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
from pathlib import Path
from bisect import bisect_left

import cv2
from cv_bridge import CvBridge

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def clamp(x, low, high):
    return max(low, min(high, x))


def open_bag_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_topic_type_map(reader):
    topic_types = reader.get_all_topics_and_types()
    return {topic.name: topic.type for topic in topic_types}


def find_nearest_vehicle(vehicle_times, vehicle_data, target_time_ns, max_dt_ns):
    idx = bisect_left(vehicle_times, target_time_ns)

    best = None
    best_dt = None

    for j in [idx - 1, idx]:
        if j < 0 or j >= len(vehicle_times):
            continue

        dt = abs(vehicle_times[j] - target_time_ns)

        if best is None or dt < best_dt:
            best = vehicle_data[j]
            best_dt = dt

    if best is None:
        return None

    if best_dt > max_dt_ns:
        return None

    return best


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        required=True,
        help="Input rosbag directory, e.g. dataset/raw/e2e_corridor_001"
    )

    parser.add_argument(
        "--out",
        required=True,
        help="Output dataset directory, e.g. dataset/processed/e2e_corridor_001"
    )

    parser.add_argument(
        "--image-topic",
        default="/image_raw"
    )

    parser.add_argument(
        "--vehicle-topic",
        default="/vehicle_info"
    )

    parser.add_argument(
        "--label-delay",
        type=float,
        default=0.10,
        help="Use vehicle state at image_time + label_delay as label, seconds"
    )

    parser.add_argument(
        "--max-sync-dt",
        type=float,
        default=0.05,
        help="Max allowed time difference between image and vehicle label, seconds"
    )

    parser.add_argument(
        "--max-steer-deg",
        type=float,
        default=30.0,
        help="Max steering angle in degrees, used for normalization"
    )

    parser.add_argument(
        "--steering-unit",
        choices=["deg", "rad"],
        default="deg",
        help="Unit of msg.steering in /vehicle_info"
    )

    parser.add_argument(
        "--save-every-n",
        type=int,
        default=1,
        help="Save every N valid image frames"
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=90
    )

    args = parser.parse_args()

    bag_path = Path(args.bag)
    out_dir = Path(args.out)
    images_dir = out_dir / "images"
    labels_path = out_dir / "labels.csv"

    out_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)

    label_delay_ns = int(args.label_delay * 1e9)
    max_sync_dt_ns = int(args.max_sync_dt * 1e9)

    print(f"Input bag: {bag_path}")
    print(f"Output dir: {out_dir}")
    print(f"Image topic: {args.image_topic}")
    print(f"Vehicle topic: {args.vehicle_topic}")
    print(f"Label delay: {args.label_delay:.3f}s")
    print(f"Max sync dt: {args.max_sync_dt:.3f}s")
    print(f"Steering unit: {args.steering_unit}")

    # First pass: read vehicle_info.
    reader = open_bag_reader(bag_path)
    topic_type_map = get_topic_type_map(reader)

    if args.image_topic not in topic_type_map:
        raise RuntimeError(f"Image topic not found in bag: {args.image_topic}")

    if args.vehicle_topic not in topic_type_map:
        raise RuntimeError(f"Vehicle topic not found in bag: {args.vehicle_topic}")

    image_msg_type = get_message(topic_type_map[args.image_topic])
    vehicle_msg_type = get_message(topic_type_map[args.vehicle_topic])

    vehicle_times = []
    vehicle_data = []

    print("Reading vehicle_info messages...")

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic != args.vehicle_topic:
            continue

        msg = deserialize_message(data, vehicle_msg_type)

        velocity = float(msg.velocity)
        steering = float(msg.steering)

        if args.steering_unit == "rad":
            steering_deg = steering * 180.0 / 3.141592653589793
        else:
            steering_deg = steering

        vehicle_times.append(int(t))
        vehicle_data.append({
            "time_ns": int(t),
            "velocity": velocity,
            "steering_deg": steering_deg,
        })

    if len(vehicle_times) == 0:
        raise RuntimeError("No vehicle_info messages found.")

    print(f"Loaded vehicle_info count: {len(vehicle_times)}")

    # Second pass: read images and match labels.
    reader = open_bag_reader(bag_path)
    bridge = CvBridge()

    saved_count = 0
    skipped_no_label = 0
    skipped_decode = 0
    image_count = 0
    valid_count = 0

    print("Converting images...")

    with open(labels_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "image_path",
                "timestamp_ns",
                "label_timestamp_ns",
                "velocity",
                "steering_deg",
                "steering_norm",
                "steering_norm_clipped",
            ]
        )
        writer.writeheader()

        while reader.has_next():
            topic, data, t = reader.read_next()

            if topic != args.image_topic:
                continue

            image_count += 1

            target_time_ns = int(t) + label_delay_ns

            label = find_nearest_vehicle(
                vehicle_times=vehicle_times,
                vehicle_data=vehicle_data,
                target_time_ns=target_time_ns,
                max_dt_ns=max_sync_dt_ns
            )

            if label is None:
                skipped_no_label += 1
                continue

            valid_count += 1

            if valid_count % args.save_every_n != 0:
                continue

            try:
                image_msg = deserialize_message(data, image_msg_type)
                image_bgr = bridge.imgmsg_to_cv2(
                    image_msg,
                    desired_encoding="bgr8"
                )
            except Exception as e:
                print(f"Image decode failed at t={t}: {e}")
                skipped_decode += 1
                continue

            image_name = f"{saved_count:06d}.jpg"
            image_rel_path = f"images/{image_name}"
            image_abs_path = images_dir / image_name

            ok = cv2.imwrite(
                str(image_abs_path),
                image_bgr,
                [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )

            if not ok:
                print(f"Failed to write image: {image_abs_path}")
                skipped_decode += 1
                continue

            steering_norm = label["steering_deg"] / args.max_steer_deg
            steering_norm_clipped = clamp(steering_norm, -1.0, 1.0)

            writer.writerow({
                "image_path": image_rel_path,
                "timestamp_ns": int(t),
                "label_timestamp_ns": int(label["time_ns"]),
                "velocity": float(label["velocity"]),
                "steering_deg": float(label["steering_deg"]),
                "steering_norm": float(steering_norm),
                "steering_norm_clipped": float(steering_norm_clipped),
            })

            saved_count += 1

            if saved_count % 500 == 0:
                print(f"Saved {saved_count} samples...")

    print("Done.")
    print(f"Image messages: {image_count}")
    print(f"Valid matched images: {valid_count}")
    print(f"Saved samples: {saved_count}")
    print(f"Skipped no label: {skipped_no_label}")
    print(f"Skipped decode/write: {skipped_decode}")
    print(f"Labels saved to: {labels_path}")


if __name__ == "__main__":
    main()

