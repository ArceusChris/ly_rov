#!/usr/bin/env python3
import argparse
import os
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


ENCODINGS = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "mono8": (np.uint8, 1),
    "8UC1": (np.uint8, 1),
    "8UC3": (np.uint8, 3),
    "8UC4": (np.uint8, 4),
    "bgra8": (np.uint8, 4),
    "rgba8": (np.uint8, 4),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
}


def stamp_to_datetime(msg: Image) -> datetime:
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return datetime.now()
    return datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1_000_000_000)


def image_msg_to_cv2(msg: Image):
    if msg.encoding not in ENCODINGS:
        raise ValueError(f"unsupported encoding: {msg.encoding}")

    dtype, channels = ENCODINGS[msg.encoding]
    item_size = np.dtype(dtype).itemsize
    row_items = msg.step // item_size
    expected_items = row_items * msg.height
    array = np.frombuffer(msg.data, dtype=dtype, count=expected_items)

    if channels == 1:
        image = array.reshape((msg.height, row_items))[:, : msg.width]
    else:
        image = array.reshape((msg.height, row_items // channels, channels))[
            :, : msg.width, :
        ]

    if msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif msg.encoding == "rgba8":
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)

    return np.ascontiguousarray(image)


class ImageSaver(Node):
    def __init__(self, args):
        super().__init__("save_image_raw")
        self.topic = args.topic
        self.output_dir = Path(args.output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.ext = args.ext if args.ext.startswith(".") else f".{args.ext}"
        self.min_interval = max(0.0, args.interval)
        self.max_count = max(0, args.max_count)
        self.saved_count = 0
        self.received_count = 0
        self.last_save_time = 0.0
        self.last_endpoint_print = 0.0
        self.done = False

        reliability = (
            QoSReliabilityPolicy.RELIABLE
            if args.qos == "reliable"
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
        )
        self.sub = self.create_subscription(Image, self.topic, self.on_image, qos)
        self.timer = self.create_timer(2.0, self.on_timer)
        self.get_logger().info(
            f"saving {self.topic} images to {self.output_dir}{os.sep} as *{self.ext}, "
            f"qos={args.qos}"
        )

    def on_timer(self):
        if self.received_count > 0:
            return
        publishers = self.count_publishers(self.topic)
        details = self.publisher_details()
        self.get_logger().warn(
            f"waiting for images on {self.topic}; publishers={publishers}; "
            f"{details}"
        )

    def publisher_details(self) -> str:
        infos = self.get_publishers_info_by_topic(self.topic)
        if not infos:
            return "no publisher endpoint details; check camera node and ROS_DOMAIN_ID"

        parts = []
        for info in infos:
            qos = info.qos_profile
            parts.append(
                "node="
                f"{info.node_namespace.rstrip('/')}/{info.node_name} "
                f"type={info.topic_type} "
                f"reliability={qos.reliability.name} "
                f"durability={qos.durability.name}"
            )
        return "; ".join(parts)

    def on_image(self, msg: Image):
        self.received_count += 1
        now = self.get_clock().now().nanoseconds / 1_000_000_000
        if self.min_interval > 0.0 and now - self.last_save_time < self.min_interval:
            return

        try:
            image = image_msg_to_cv2(msg)
        except Exception as exc:
            self.get_logger().error(
                "failed to convert image "
                f"encoding={msg.encoding} width={msg.width} height={msg.height} "
                f"step={msg.step} data_len={len(msg.data)}: {exc}"
            )
            return

        timestamp = stamp_to_datetime(msg).strftime("%Y%m%d_%H%M%S_%f")
        filename = self.output_dir / f"{timestamp}_{self.saved_count:06d}{self.ext}"
        if not cv2.imwrite(str(filename), image):
            self.get_logger().error(f"failed to save image: {filename}")
            return

        self.saved_count += 1
        self.last_save_time = now
        self.get_logger().info(f"saved {filename}")

        if self.max_count > 0 and self.saved_count >= self.max_count:
            self.get_logger().info(f"saved {self.saved_count} images, exiting")
            self.done = True


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save sensor_msgs/Image frames from a ROS2 image topic."
    )
    parser.add_argument("--topic", default="/image_raw", help="image topic name")
    parser.add_argument(
        "--output-dir",
        default="image_raw_capture",
        help="directory used to save images",
    )
    parser.add_argument("--ext", default=".jpg", help="image extension: .jpg or .png")
    parser.add_argument(
        "--qos",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="subscription reliability QoS",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="minimum seconds between saved frames; 0 saves every frame",
    )
    parser.add_argument(
        "--max-count",
        type=int,
        default=0,
        help="stop after this many images; 0 means unlimited",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ImageSaver(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
