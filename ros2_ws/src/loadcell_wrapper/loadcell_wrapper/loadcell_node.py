"""Publish the weight in the mixer container from the 4 load cells.

Topics:
  /loadcell/weight  std_msgs/Float32           total kg (sum of all cells)
  /loadcell/cells   std_msgs/Float32MultiArray kg per cell, for debugging
Service:
  /loadcell/tare    std_srvs/Trigger           zero with the current contents

Pins and settings: config.py. Calibrate once first:
  ros2 run loadcell_wrapper calibrate
"""

import threading
import time
from collections import deque
from statistics import median

import lgpio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger

from loadcell_wrapper import config
from loadcell_wrapper.calibrate import cell_weights, load_calibration, save_calibration
from loadcell_wrapper.hx711 import open_cells, read_medians

STALE_AFTER_S = 1.0


class LoadCellNode(Node):
    def __init__(self, cal):
        super().__init__('loadcell_node')
        self.cal = cal
        self.handle, self.cells = open_cells()

        self.weight_pub = self.create_publisher(Float32, 'loadcell/weight', 10)
        self.cells_pub = self.create_publisher(Float32MultiArray, 'loadcell/cells', 10)
        self.create_service(Trigger, 'loadcell/tare', self.on_tare)

        # GPIO is only touched by the reader thread; tare requests are handed
        # over through _tare_done and answered via _tare_result.
        self._lock = threading.Lock()
        self._tare_done = None
        self._tare_result = (False, '')
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Reading {len(self.cells)} load cells, publishing /loadcell/weight'
        )

    def on_tare(self, request, response):
        done = threading.Event()
        with self._lock:
            self._tare_done = done
        if not done.wait(timeout=config.CALIBRATION_SAMPLES * 0.2 + 5.0):
            response.success, response.message = False, 'tare timed out'
        else:
            response.success, response.message = self._tare_result
        return response

    def _tare(self):
        try:
            self.cal['offsets'] = read_medians(self.cells, config.CALIBRATION_SAMPLES)
            save_calibration(config.CALIBRATION_FILE, self.cal)
            self._tare_result = (True, 'tared')
            self.get_logger().info('Tared')
        except TimeoutError as e:
            self._tare_result = (False, str(e))

    def _read_loop(self):
        n = len(self.cells)
        windows = [deque(maxlen=config.FILTER_WINDOW) for _ in range(n)]
        fresh = [False] * n
        last_sample = [time.monotonic()] * n

        while self._running:
            with self._lock:
                tare_done, self._tare_done = self._tare_done, None
            if tare_done is not None:
                self._tare()
                tare_done.set()
                for w in windows:
                    w.clear()
                fresh = [False] * n
                last_sample = [time.monotonic()] * n
                continue

            now = time.monotonic()
            for i, cell in enumerate(self.cells):
                if cell.is_ready():
                    windows[i].append(cell.read_raw())
                    fresh[i] = True
                    last_sample[i] = now
                elif now - last_sample[i] > STALE_AFTER_S:
                    self.get_logger().warn(
                        f'cell {i} (DT GPIO{cell.dt}) not responding, check wiring',
                        throttle_duration_sec=5.0,
                    )

            # Publish once every cell has a new sample (~10 Hz). A dead cell
            # stops publishing rather than reporting a wrong total.
            if all(fresh):
                kg = cell_weights([median(w) for w in windows], self.cal)
                self.weight_pub.publish(Float32(data=float(sum(kg))))
                self.cells_pub.publish(Float32MultiArray(data=[float(k) for k in kg]))
                fresh = [False] * n

            time.sleep(0.002)

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        lgpio.gpiochip_close(self.handle)


def main(args=None):
    cal = load_calibration(config.CALIBRATION_FILE)
    if cal is None:
        print(f'No calibration at {config.CALIBRATION_FILE}.\n'
              f'Run first: ros2 run loadcell_wrapper calibrate')
        return

    rclpy.init(args=args)
    node = LoadCellNode(cal)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
