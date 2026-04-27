#!/usr/bin/env python3
"""
UDP -> ROS2 bridge for HX711 force readings.

Runs on your laptop as a normal ROS2 node.
Listens for UDP packets from the Pi's hx711_udp_publisher.py and
publishes them on /gripper/force as a geometry_msgs/WrenchStamped.

Applies a short rolling-median filter on the raw counts to kill any
single-sample electrical spikes that slip past the Pi-side outlier
filter. Real contact events last many samples at 20 Hz so they pass
through unaffected.

How to run (on the laptop, with your ROS2 env sourced):
    python3 force_bridge_node.py

To calibrate (see README block at bottom), set these parameters
at launch or at runtime:
    force_in_newtons = (raw_median - offset) / slope

Example:
    ros2 run ... force_bridge_node --ros-args \\
        -p offset:=-128000.0 -p slope:=420.0 -p median_window:=3
"""

import json
import socket
import statistics
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


# ---------- CONFIG ----------
UDP_HOST = "0.0.0.0"   # listen on all laptop interfaces
UDP_PORT = 9870        # must match hx711_udp_publisher.py
# ----------------------------


class ForceBridge(Node):
    def __init__(self):
        super().__init__("force_bridge")

        # Calibration parameters (defaults = pass-through raw counts)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("slope",  1.0)
        self.declare_parameter("frame_id", "gripper")

        # Rolling-median window on raw counts. 3 = kills single-sample
        # spikes with ~100 ms added latency at 20 Hz. Set to 1 to disable.
        self.declare_parameter("median_window", 3)

        self._median_window = max(1, int(self.get_parameter("median_window").value))
        self._raw_buf = deque(maxlen=self._median_window)
        self._stamp_buf = deque(maxlen=self._median_window)

        self.pub = self.create_publisher(WrenchStamped, "/gripper/force", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.settimeout(0.5)

        self.running = True
        self.thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"force_bridge listening on UDP {UDP_HOST}:{UDP_PORT}, "
            f"publishing /gripper/force "
            f"(median_window={self._median_window})"
        )

    def _rx_loop(self):
        while self.running and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                pkt = json.loads(data.decode("utf-8"))
            except ValueError:
                continue

            if not pkt.get("valid", False):
                continue

            raw_in = float(pkt.get("raw", 0))
            pi_stamp = pkt.get("stamp")

            # Push into rolling window
            self._raw_buf.append(raw_in)
            self._stamp_buf.append(pi_stamp)

            # Wait until we have a full window before publishing.
            # This is the one-time startup delay (N-1 samples = ~100 ms for N=3).
            if len(self._raw_buf) < self._median_window:
                continue

            # Median on raw counts, then apply calibration.
            # Using middle-sample timestamp keeps header.stamp aligned with
            # the physical sample that the median most likely represents.
            raw_med = statistics.median(self._raw_buf)
            mid_idx = self._median_window // 2
            stamp_for_msg = list(self._stamp_buf)[mid_idx]

            offset = self.get_parameter("offset").value
            slope  = self.get_parameter("slope").value
            force  = (raw_med - offset) / slope if slope != 0.0 else 0.0

            msg = WrenchStamped()

            if isinstance(stamp_for_msg, (int, float)) and stamp_for_msg > 0:
                msg.header.stamp.sec     = int(stamp_for_msg)
                msg.header.stamp.nanosec = int((stamp_for_msg - int(stamp_for_msg)) * 1e9)
            else:
                msg.header.stamp = self.get_clock().now().to_msg()

            msg.header.frame_id = self.get_parameter("frame_id").value
            # Gripper presses forward along body-frame +X
            msg.wrench.force.x = force
            self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ForceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------
# CALIBRATION NOTES
# ---------------------------------------------------------------------
# Before /gripper/force means anything in Newtons, you need two numbers:
#
#   offset : raw HX711 value when no load is applied (tare)
#   slope  : how many raw counts per Newton of force
#
# How to calibrate (do this out of the water, on a bench):
#   1. Launch both hx711_udp_publisher.py (on Pi) and this node.
#   2. With no load:
#          ros2 topic echo /gripper/force
#      Note the force.x value. That is your "offset" (divided by 1.0
#      since slope=1). Set offset parameter to that raw number.
#   3. Hang a known weight from the load cell (e.g. a 1 kg mass => 9.81 N).
#      Note the new raw reading, compute:
#          slope = (raw_loaded - offset) / force_in_newtons
#      Set the slope parameter.
#   4. Verify: with the weight, force.x should now read ~9.81.
#
# You can set the parameters at runtime:
#   ros2 param set /force_bridge offset <number>
#   ros2 param set /force_bridge slope  <number>
#   ros2 param set /force_bridge median_window <int>   # requires restart
# ---------------------------------------------------------------------