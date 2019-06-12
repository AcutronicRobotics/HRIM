#!/usr/bin/python3

import argparse
import functools
import importlib
import sys
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

def listener_cb(msg, received_messages, expected_msgs):
    known_msg = False
    msg_repr = repr(msg)
    for num, exp in expected_msgs:
        if msg_repr == exp:
            print('received message #{} of {}'.format(num + 1, len(expected_msgs)))
            known_msg = True
            already_received = False
            for rmsg in received_messages:
                if rmsg == msg_repr:
                    already_received = True
                    break

            if not already_received:
                received_messages.append(msg_repr)
            break
    if known_msg is False:
        raise RuntimeError('received unexpected message %r' % msg)


def listener(message_pkg, message_name, namespace):
    import rclpy

    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)

    rclpy.init(args=[])

    node = rclpy.create_node('listener', namespace=namespace)

    received_messages = []
    a = msg_mod( header=Header() )
    expected_msgs = [(i, repr(msg)) for i, msg in enumerate([msg_mod(header=Header(stamp=Time(sec=1, nanosec=0))),
            msg_mod(header=Header(stamp=Time(sec=2, nanosec=0)))])]

    chatter_callback = functools.partial(
        listener_cb, received_messages=received_messages, expected_msgs=expected_msgs)

    lifespan = Duration(seconds=0.5)

    qos_profile = QoSProfile(
        depth=10,
        # Guaranteed delivery is needed to send messages to late-joining subscription.
        reliability=QoSReliabilityPolicy.RELIABLE,
        # Store messages on the publisher so that they can be affected by Lifespan.
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=lifespan)

    node.create_subscription(
        msg_mod, 'test/message/' + message_name, chatter_callback, qos_profile)

    spin_count = 1
    print('subscriber: beginning loop')
    while (rclpy.ok() and len(received_messages) != len(expected_msgs)):
        rclpy.spin_once(node)
        spin_count += 1
        print('spin_count: ' + str(spin_count))
    node.destroy_node()
    rclpy.shutdown()

    assert len(received_messages) == len(expected_msgs),\
        'Should have received {} {} messages from talker'.format(len(expected_msgs), message_name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_pkg', help='name of the ROS package')
    parser.add_argument('message_name', help='name of the ROS message')
    parser.add_argument('namespace', help='namespace of the ROS node')
    args = parser.parse_args()
    try:
        listener(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            namespace=args.namespace
        )
    except KeyboardInterrupt:
        print('subscriber stopped cleanly')
    except BaseException:
        print('exception in subscriber:', file=sys.stderr)
        raise
