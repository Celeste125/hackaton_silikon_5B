"""One-time load cell calibration: ros2 run mixer_esp_driver calibrate

Run it while esp_driver_node is running: it reads /loadcell/raw, writes the
calibration file and tells the driver to reload it.
"""

import time
from statistics import median

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from mixer_esp_driver import config
from mixer_esp_driver.calibration import (
    calibration_path, cell_weights, compute_calibration, save_calibration,
)

SETTLE_S = 1.5  # wait after each step: lets the load settle, drops queued old samples


class Calibrator(Node):
    def __init__(self):
        super().__init__('loadcell_calibrate')
        self.samples = None  # list per cell while collecting, else None
        self.create_subscription(Int32MultiArray, 'loadcell/raw', self.on_raw, 10)
        self.reload_client = self.create_client(Trigger, 'loadcell/reload_calibration')

    def on_raw(self, msg):
        if self.samples is not None and len(msg.data) == len(self.samples):
            for s, raw in zip(self.samples, msg.data):
                s.append(raw)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collect(self, n):
        """Median of the next n samples of every cell (after settling)."""
        self.spin_for(SETTLE_S)
        self.samples = [[] for _ in range(config.NUM_CELLS)]
        deadline = time.monotonic() + n * 0.2 + 2.0
        while len(self.samples[0]) < n:
            if time.monotonic() > deadline:
                self.samples = None
                raise TimeoutError(
                    'No data on /loadcell/raw. Is esp_driver_node running and the ESP connected?'
                )
            rclpy.spin_once(self, timeout_sec=0.1)
        medians = [median(s) for s in self.samples]
        self.samples = None
        return medians

    def reload_driver(self):
        if not self.reload_client.wait_for_service(timeout_sec=2.0):
            print('   driver not found, restart esp_driver_node to use the new calibration')
            return
        future = self.reload_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        ok = future.result() is not None and future.result().success
        print('   driver reloaded the calibration' if ok else
              '   reload failed, restart esp_driver_node')


def ask_float(prompt, default):
    while True:
        text = input(prompt).strip()
        if not text:
            return default
        try:
            return float(text)
        except ValueError:
            print('   not a number, try again')


def run(node):
    print(f'Calibrating {config.NUM_CELLS} load cells.')
    input('1) Mount the EMPTY container (nothing inside). Press Enter...')
    print('   measuring zero...')
    offsets = node.collect(config.CALIBRATION_SAMPLES)

    known_kg = ask_float('2) Known weight in kg [1.0]: ', 1.0)
    input(f'3) Put {known_kg} kg in the CENTER of the container. Press Enter...')
    print('   measuring...')
    loaded = node.collect(config.CALIBRATION_SAMPLES)

    try:
        cal, shares = compute_calibration(offsets, loaded, known_kg)
    except ValueError as e:
        print(f'   {e}')
        return
    for i, (share, sign) in enumerate(zip(shares, cal['signs'])):
        note = ''
        if share < 0.05:
            note = '  <- barely loaded, check wiring/mounting'
        elif sign < 0:
            note = '  (reads backwards, flipped in software)'
        print(f'   cell {i}: {share:5.1%} of the load{note}')

    save_calibration(cal)
    print(f'   saved to {calibration_path()}')
    node.reload_driver()

    print(f'4) Check: move the weight to the corners, total should stay '
          f'~{known_kg} kg. Ctrl+C to quit.')
    while True:
        kg = cell_weights(node.collect(5), cal)
        cells_text = ' '.join(f'{k:6.3f}' for k in kg)
        print(f'\r   total {sum(kg):7.3f} kg   cells {cells_text}  ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()
    try:
        run(node)
    except TimeoutError as e:
        print(f'\n   {e}')
    except (KeyboardInterrupt, ExternalShutdownException, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
