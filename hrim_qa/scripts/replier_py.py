#!/usr/bin/python3

import argparse
import functools
import importlib
import sys


def replier_callback(request, response, srv_fixtures):
    for req, resp in srv_fixtures:
        if request.__repr__() == req.__repr__():
            print('received request #%d of %d' %
                  (srv_fixtures.index([req, resp]) + 1, len(srv_fixtures)))
            return resp


def replier(service_pkg, service_name, number_of_cycles, namespace):
    import rclpy

    module = importlib.import_module(service_pkg + '.srv')
    srv_mod = getattr(module, service_name)

    rclpy.init(args=[])

    node = rclpy.create_node('replier', namespace=namespace)

    req = srv_mod.Request()
    resp = srv_mod.Response()

    srv_fixtures = [[req, resp]]

    chatter_callback = functools.partial(
        replier_callback, srv_fixtures=srv_fixtures)

    node.create_service(
        srv_mod, 'test/service/' + service_name, chatter_callback)

    spin_count = 0
    print('replier: beginning loop')
    while rclpy.ok() and spin_count < number_of_cycles:
        rclpy.spin_once(node, timeout_sec=2)
        spin_count += 1
        # print('spin_count: ' + str(spin_count))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('service_pkg', help='name of the ROS package')
    parser.add_argument('service_name', help='name of the ROS message')
    parser.add_argument('namespace', help='namespace of the ROS node')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=1,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        replier(
            service_pkg=args.service_pkg,
            service_name=args.service_name,
            number_of_cycles=args.number_of_cycles,
            namespace=args.namespace
        )
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
