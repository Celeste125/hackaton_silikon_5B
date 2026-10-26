#!/usr/bin/env python3
"""
Send commands to the valve controller.  (Run on the Jetson.)

Usage:
  ros2 run silo_controller valve_cmd <valve> <0-100>       open the valve that many percent (0 = closed)
  ros2 run silo_controller valve_cmd <valve> open          open the valve
  ros2 run silo_controller valve_cmd <valve> close         close the valve
  ros2 run silo_controller valve_cmd <valve> jog <deg>     move by a small step, e.g. jog 5 / jog -2
  ros2 run silo_controller valve_cmd <valve> goto <deg>    move to an angle (default limits -60..240)
  ros2 run silo_controller valve_cmd <valve> set-closed    save current position as CLOSED
  ros2 run silo_controller valve_cmd <valve> set-open      save current position as OPEN
  ros2 run silo_controller valve_cmd <valve> calibrate     interactive mode (recommended)

Example:
  ros2 run silo_controller valve_cmd 2 calibrate
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

CALIBRATE_HELP = """
Calibration mode - type a command and press Enter:
  +   /  -        step +1 / -1 degree
  ++  /  --       step +5 / -5 degrees
  <number>        step by that amount, e.g. 3 or -0.5
  g <deg>         go to an angle, e.g. g 170
  c               save current position as CLOSED
  o               save current position as OPEN
  open / close    test the saved positions
  p <0-100>       open by a percentage, e.g. p 50
  h               show this help
  q               quit
"""


class NoListener(Exception):
    pass


def is_number(text: str) -> bool:
    try:
        float(text)
        return True
    except ValueError:
        return False


def wait_for_subscriber(node, pub, topic, timeout=5.0):
    """Wait until the valve controller listens, so a single message is not lost."""
    deadline = time.monotonic() + timeout
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f'Nobody is listening on /{topic}. Is the valve controller running for this valve, '
            f'and is ROS_DOMAIN_ID the same on both machines?')
        raise NoListener
    time.sleep(0.2)  # let both sides finish matching


def calibrate(node, topic, calib_topic):
    bool_pub = node.create_publisher(Bool, topic, 10)
    calib_pub = node.create_publisher(String, calib_topic, 10)
    percent_pub = node.create_publisher(Float32, f'{topic}/percent', 10)

    latest = {'angle': None}
    node.create_subscription(Float32, f'{topic}/angle',
                             lambda m: latest.__setitem__('angle', m.data), 10)

    wait_for_subscriber(node, calib_pub, calib_topic)
    wait_for_subscriber(node, bool_pub, topic)
    print(CALIBRATE_HELP)

    steps = {'+': 1, '-': -1, '++': 5, '--': -5}

    while True:
        try:
            line = input(f'{topic}> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue
        if line in ('q', 'quit', 'exit'):
            break
        if line in ('h', 'help', '?'):
            print(CALIBRATE_HELP)
            continue

        latest['angle'] = None
        if line in ('open', 'close'):
            bool_pub.publish(Bool(data=line == 'open'))
        elif line.startswith('p '):
            try:
                percent_pub.publish(Float32(data=float(line[2:])))
            except ValueError:
                print('  usage: p <0-100>, e.g. p 50')
                continue
        elif line == 'c':
            calib_pub.publish(String(data='set_closed'))
        elif line == 'o':
            calib_pub.publish(String(data='set_open'))
        elif line in steps:
            calib_pub.publish(String(data=f'jog {steps[line]}'))
        elif line.startswith('g '):
            try:
                calib_pub.publish(String(data=f'goto {float(line[2:])}'))
            except ValueError:
                print('  usage: g <deg>, e.g. g 170')
                continue
        else:
            try:
                calib_pub.publish(String(data=f'jog {float(line)}'))
            except ValueError:
                print('  unknown command, type h for help')
                continue

        # Show the angle the controller reports back
        deadline = time.monotonic() + 1.0
        while latest['angle'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if latest['angle'] is not None:
            saved = {'c': '  (saved as CLOSED)', 'o': '  (saved as OPEN)'}.get(line, '')
            print(f"  -> {latest['angle']:g} deg{saved}")
        elif line in ('open', 'close') or line.startswith('p '):
            print('  (already there)')
        else:
            print('  (no feedback - check the controller log on the Pi)')


def main():
    args = sys.argv[1:]
    if len(args) < 2 or not args[0].isdigit():
        print(__doc__)
        sys.exit(1)

    topic = f'valve{args[0]}'
    calib_topic = f'{topic}/calib'
    action = args[1].lower()

    rclpy.init()
    node = Node('valve_cmd')
    code = 0
    try:
        if is_number(action) and len(args) == 2:
            percent = float(action)
            if not 0 <= percent <= 100:
                print('Percentage must be between 0 and 100')
                raise SystemExit(1)
            pub = node.create_publisher(Float32, f'{topic}/percent', 10)
            wait_for_subscriber(node, pub, f'{topic}/percent')
            pub.publish(Float32(data=percent))
            node.get_logger().info(f'Sent {percent:g}% to /{topic}/percent')
            time.sleep(0.5)

        elif action in ('open', 'close') and len(args) == 2:
            pub = node.create_publisher(Bool, topic, 10)
            wait_for_subscriber(node, pub, topic)
            pub.publish(Bool(data=action == 'open'))
            node.get_logger().info(f'Sent {action.upper()} to /{topic}')
            time.sleep(0.5)

        elif action in ('jog', 'goto') and len(args) == 3:
            value = float(args[2])
            pub = node.create_publisher(String, calib_topic, 10)
            wait_for_subscriber(node, pub, calib_topic)
            pub.publish(String(data=f'{action} {value}'))
            node.get_logger().info(f'Sent "{action} {value:g}" to /{calib_topic}')
            time.sleep(0.5)

        elif action in ('set-closed', 'set-open') and len(args) == 2:
            pub = node.create_publisher(String, calib_topic, 10)
            wait_for_subscriber(node, pub, calib_topic)
            pub.publish(String(data=action.replace('-', '_')))
            node.get_logger().info(f'Sent "{action}" to /{calib_topic}')
            time.sleep(0.5)

        elif action == 'calibrate' and len(args) == 2:
            calibrate(node, topic, calib_topic)

        else:
            print(__doc__)
            code = 1
    except NoListener:
        code = 1
    except ValueError:
        print('Angle must be a number, e.g. jog 5 or goto 170')
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
