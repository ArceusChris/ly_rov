#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import Capture, PerceptionTargets, Target
from rclpy.node import Node


class RemoteMaskMjpegBridge(Node):
    def __init__(self):
        super().__init__("remote_mask_mjpeg_bridge")
        self.input_url = self.declare_parameter(
            "input_url",
            "http://192.168.127.11:8090/mask.mjpg",
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic",
            "remote_pipe_segmentation",
        ).value
        self.threshold = int(self.declare_parameter("threshold", 127).value)
        self.mask_label = float(self.declare_parameter("mask_label", 1).value)
        self.max_fps = float(self.declare_parameter("max_fps", 15.0).value)
        self.reconnect_delay = float(self.declare_parameter("reconnect_delay", 1.0).value)

        self.publisher = self.create_publisher(PerceptionTargets, self.output_topic, 10)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"bridging remote mask {self.input_url} -> {self.output_topic}"
        )

    def destroy_node(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        super().destroy_node()

    def capture_loop(self):
        delay = 0.0 if self.max_fps <= 0.0 else 1.0 / self.max_fps
        while rclpy.ok() and not self.stop_event.is_set():
            cap = cv2.VideoCapture(self.input_url)
            if not cap.isOpened():
                self.get_logger().warn(f"failed to open remote mask stream: {self.input_url}")
                time.sleep(self.reconnect_delay)
                continue

            self.get_logger().info(f"reading remote mask stream: {self.input_url}")
            while rclpy.ok() and not self.stop_event.is_set():
                started = time.monotonic()
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.get_logger().warn("remote mask stream ended, reconnecting")
                    break
                self.publish_mask(frame)
                if delay > 0.0:
                    elapsed = time.monotonic() - started
                    if elapsed < delay:
                        time.sleep(delay - elapsed)
            cap.release()
            time.sleep(self.reconnect_delay)

    def publish_mask(self, frame):
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        binary = (gray > self.threshold).astype(np.float32) * self.mask_label

        msg = PerceptionTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "remote_mask"

        capture = Capture()
        capture.img.height = int(binary.shape[0])
        capture.img.width = int(binary.shape[1])
        capture.img.step = 1
        capture.features = binary.ravel().tolist()

        target = Target()
        target.type = "pipe"
        target.captures.append(capture)
        msg.targets.append(target)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RemoteMaskMjpegBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
