#!/usr/bin/env python3
"""Read-only odom check: wait up to 20 seconds, then measure a 10-second window."""
import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('stm32_odom_build_check')
received = []
stamps = []
def on_odom(msg):
    received.append(time.monotonic())
    stamps.append(msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
sub = node.create_subscription(Odometry, '/odomfromSTM32', on_odom, 10)
start = time.monotonic()
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if received and now - received[0] >= 10:
            break
        if not received and now - start >= 20:
            raise SystemExit('FAIL: no /odomfromSTM32 within 20 seconds')
    result = {
        'messages': len(received),
        'rate_hz': (len(received)-1)/(received[-1]-received[0]) if len(received)>1 else 0,
        'max_gap_s': max((b-a for a,b in zip(received, received[1:])), default=0),
        'stamp_advanced': len(set(stamps)) > 1,
    }
    print(json.dumps(result, indent=2))
    if len(received) < 2 or not result['stamp_advanced']:
        raise SystemExit('FAIL: odom stream did not advance')
finally:
    node.destroy_node()
    rclpy.shutdown()
