#!/usr/bin/python3

import argparse
import importlib
import sys
import time
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

def talker(message_pkg, message_name, number_of_cycles, namespace):
    import rclpy

    time.sleep(0.1)

    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)

    rclpy.init(args=[])

    node = rclpy.create_node('talker', namespace=namespace)

    lifespan = Duration(seconds=0.5)

    qos_profile = QoSProfile(
        depth=10,
        # Guaranteed delivery is needed to send messages to late-joining subscription.
        reliability=QoSReliabilityPolicy.RELIABLE,
        # Store messages on the publisher so that they can be affected by Lifespan.
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=lifespan)

    chatter_pub = node.create_publisher(
        msg_mod, 'test/message/' + message_name, qos_profile)

    cycle_count = 0
    print('talker: beginning loop')
    msgs = []
    try:
        msgs = [msg_mod(header=Header(stamp=Time(sec=1, nanosec=0)))]
    except Exception as e:
        msgs = [msg_mod()]

    while rclpy.ok() and cycle_count < number_of_cycles:
        msg_count = 0
        for msg in msgs:
            chatter_pub.publish(msg)
            msg_count += 1
            print('publishing message #%d' % msg_count)
            time.sleep(0.1)
        cycle_count += 1
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_pkg', help='name of the ROS package')
    parser.add_argument('message_name', help='name of the ROS message')
    parser.add_argument('namespace', help='namespace of the ROS node')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=1,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        talker(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            namespace=args.namespace,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
