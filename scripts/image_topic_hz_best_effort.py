#!/usr/bin/env python3
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class ImageTopicHz(Node):
    def __init__(self, topic: str, window: int):
        super().__init__("image_topic_hz_best_effort")
        self.topic = topic
        self.times = deque(maxlen=max(2, window))
        self.last_print = time.monotonic()
        self.last_wait_print = self.last_print
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f"measuring {topic} with BEST_EFFORT QoS")

    def on_timer(self):
        now = time.monotonic()
        if self.times or now - self.last_wait_print < 2.0:
            return
        self.last_wait_print = now
        publishers = self.count_publishers(self.topic)
        print(
            f"waiting for images on {self.topic}; discovered publishers: {publishers}",
            flush=True,
        )

    def on_image(self, msg: Image):
        del msg
        now = time.monotonic()
        self.times.append(now)
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        if len(self.times) < 2:
            return
        elapsed = self.times[-1] - self.times[0]
        if elapsed <= 0.0:
            return
        rate = (len(self.times) - 1) / elapsed
        print(f"average rate: {rate:.3f} Hz window: {len(self.times)}", flush=True)


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
    window = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    rclpy.init()
    node = ImageTopicHz(topic, window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
