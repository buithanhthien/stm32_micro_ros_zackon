#!/usr/bin/env python3
"""Offline only: summarize Twist/Odometry and export a shared receive-time CSV.

Usage after sourcing ROS: python3 tools/analyze-motion-bag.py BAG OUTPUT_PREFIX
Wheel speeds reconstructed from odometry are NOT independent encoder samples.
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('output_prefix')
    parser.add_argument('--wheel-base', type=float, default=0.292)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {'/cmd_vel': 'geometry_msgs/msg/Twist',
              '/odomfromSTM32': 'nav_msgs/msg/Odometry'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in wanted and t.type == wanted[t.name]}
    rows = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        twist = msg if topic == '/cmd_vel' else msg.twist.twist
        rows[topic].append((stamp, twist.linear.x, twist.angular.z))
    start = min((r[0] for data in rows.values() for r in data), default=0)
    summary = {'bag': args.bag, 'clock': 'bag receive time; no MCU synchronization',
               'topics': {}, 'cmd_zero_transitions_s': []}
    for topic, data in rows.items():
        data.sort()
        gaps = [(b[0]-a[0])/1e9 for a, b in zip(data, data[1:])]
        result = {'count': len(data), 'max_gap_s': max(gaps, default=None)}
        if data:
            result.update(v_range=[min(r[1] for r in data), max(r[1] for r in data)],
                          w_range=[min(r[2] for r in data), max(r[2] for r in data)],
                          first_s=(data[0][0]-start)/1e9, last_s=(data[-1][0]-start)/1e9)
        if topic == '/cmd_vel':
            result['exact_zero_count'] = sum(v == 0 and w == 0 for _, v, w in data)
            result['counts_per_receive_second'] = dict(sorted(Counter(
                (t-start)//1_000_000_000 for t, _, _ in data).items()))
            summary['cmd_zero_transitions_s'] = [
                (b[0]-start)/1e9 for a, b in zip(data, data[1:])
                if (a[1] != 0 or a[2] != 0) and b[1] == 0 and b[2] == 0]
        summary['topics'][topic] = result
    prefix = Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with prefix.with_suffix('.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['receive_ns', 'elapsed_s', 'topic', 'v_mps', 'w_rad_s',
                         'left_mps', 'right_mps'])
        for t, topic, v, w in sorted((t, topic, v, w) for topic, data in rows.items()
                                      for t, v, w in data):
            writer.writerow([t, (t-start)/1e9, topic, v, w,
                             v-w*args.wheel_base/2, v+w*args.wheel_base/2])
    prefix.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
