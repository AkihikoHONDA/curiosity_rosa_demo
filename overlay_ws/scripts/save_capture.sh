#!/usr/bin/env bash
set -eo pipefail

python3 - <<'PY'
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

output_dir = os.environ.get("CAPTURE_OUT_DIR", "/workspace/overlay_ws/artifacts")
os.makedirs(output_dir, exist_ok=True)

stamp = time.strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(output_dir, f"capture_{stamp}.ppm")

class CaptureOnce(Node):
    def __init__(self) -> None:
        super().__init__("capture_once")
        self._done = False
        self.create_subscription(Image, "/capture/image_raw", self._on_msg, 10)

    def _on_msg(self, msg: Image) -> None:
        if self._done:
            return
        self._done = True
        try:
            if msg.encoding not in ("rgb8", "bgr8"):
                raise RuntimeError(f"unsupported encoding: {msg.encoding}")
            width = msg.width
            height = msg.height
            row_len = msg.step
            pixel_len = width * 3
            data = bytes(msg.data)
            if row_len != pixel_len:
                buf = bytearray()
                for y in range(height):
                    start = y * row_len
                    buf.extend(data[start : start + pixel_len])
                data = bytes(buf)
            if msg.encoding == "bgr8":
                buf = bytearray(data)
                for i in range(0, len(buf), 3):
                    buf[i], buf[i + 2] = buf[i + 2], buf[i]
                data = bytes(buf)
            header = f"P6\n{width} {height}\n255\n".encode("ascii")
            with open(out_path, "wb") as f:
                f.write(header)
                f.write(data)
            self.get_logger().info(f"saved: {out_path}")
        finally:
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = CaptureOnce()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == "__main__":
    main()
PY