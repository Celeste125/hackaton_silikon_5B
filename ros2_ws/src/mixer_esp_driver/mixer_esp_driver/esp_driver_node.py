"""ROS 2 driver for the mixer ESP32 (firmware: mixer_esp32/).

Load cells
  /loadcell/weight  std_msgs/Float32           total kg in the container
  /loadcell/cells   std_msgs/Float32MultiArray kg per cell, for debugging
  /loadcell/raw     std_msgs/Int32MultiArray   raw HX711 counts (calibrate uses this)
  /loadcell/tare    std_srvs/Trigger           zero with the current contents
  /loadcell/reload_calibration  std_srvs/Trigger  reread the calibration file
Motors
  /mixer/motor_0, /mixer/motor_1  std_msgs/Float32, -1.0 .. 1.0 (negative = reverse)
  e.g. ros2 topic pub -r 5 /mixer/motor_0 std_msgs/Float32 "data: 0.5"

Settings: config.py. Calibrate once (with this node running):
  ros2 run mixer_esp_driver calibrate
"""

import threading
import time
from collections import deque
from statistics import median

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

from mixer_esp_driver import config
from mixer_esp_driver.calibration import cell_weights, load_calibration, save_calibration


class EspDriverNode(Node):
    def __init__(self):
        super().__init__('esp_driver_node')
        self.cal = None
        self._load_calibration()

        self.weight_pub = self.create_publisher(Float32, 'loadcell/weight', 10)
        self.cells_pub = self.create_publisher(Float32MultiArray, 'loadcell/cells', 10)
        self.raw_pub = self.create_publisher(Int32MultiArray, 'loadcell/raw', 10)
        self.create_service(Trigger, 'loadcell/tare', self.on_tare)
        self.create_service(Trigger, 'loadcell/reload_calibration', self.on_reload)

        self.speeds = [0.0] * config.NUM_MOTORS
        self.last_cmd = [None] * config.NUM_MOTORS
        for i in range(config.NUM_MOTORS):
            self.create_subscription(
                Float32, f'mixer/motor_{i}',
                lambda msg, i=i: self.on_motor_cmd(i, msg), 10,
            )
        self.create_timer(1.0 / config.MOTOR_SEND_HZ, self.send_motors)

        self.windows = [deque(maxlen=config.FILTER_WINDOW) for _ in range(config.NUM_CELLS)]
        # Tare: the reader thread collects samples into _tare_samples until
        # CALIBRATION_SAMPLES are in, then sets _tare_done.
        self._lock = threading.Lock()
        self._tare_samples = None
        self._tare_done = None

        self.ser = None
        self._write_lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    # ---- calibration ---------------------------------------------------------

    def _load_calibration(self):
        try:
            self.cal = load_calibration()
        except (ValueError, KeyError) as e:
            self.cal = None
            self.get_logger().error(f'Bad calibration file: {e}')
            return False
        if self.cal is None:
            self.get_logger().warn(
                f'No calibration at {config.CALIBRATION_FILE}: only /loadcell/raw '
                f'is published. Run: ros2 run mixer_esp_driver calibrate'
            )
            return False
        self.get_logger().info(f'Loaded calibration from {config.CALIBRATION_FILE}')
        return True

    def on_reload(self, request, response):
        response.success = self._load_calibration()
        response.message = 'reloaded' if response.success else 'no valid calibration file'
        return response

    def on_tare(self, request, response):
        if self.cal is None:
            response.success, response.message = False, 'not calibrated yet'
            return response
        done = threading.Event()
        with self._lock:
            self._tare_samples = [[] for _ in range(config.NUM_CELLS)]
            self._tare_done = done
        if not done.wait(timeout=config.CALIBRATION_SAMPLES * 0.2 + 5.0):
            with self._lock:
                self._tare_samples = self._tare_done = None
            response.success, response.message = False, 'tare timed out, no data from ESP?'
            return response
        save_calibration(self.cal)
        self.get_logger().info('Tared')
        response.success, response.message = True, 'tared'
        return response

    # ---- serial --------------------------------------------------------------

    def _open_serial(self):
        ser = serial.Serial()
        ser.port = config.SERIAL_PORT
        ser.baudrate = config.BAUD
        ser.timeout = 0.5
        # Keep DTR/RTS released: on ESP32 DevKits they drive the reset and
        # boot pins, and toggling them can reset the ESP or hold it in reset.
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    def _serial_loop(self):
        while self._running:
            if self.ser is None:
                try:
                    self.ser = self._open_serial()
                    self.get_logger().info(f'Opened {config.SERIAL_PORT}')
                except serial.SerialException as e:
                    self.get_logger().warn(
                        f'Cannot open {config.SERIAL_PORT}: {e}', throttle_duration_sec=10.0
                    )
                    time.sleep(2.0)
                    continue
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial error, reconnecting: {e}')
                self._close_serial()
                continue
            if not line:
                continue
            try:
                self._handle_line(line.decode('ascii', errors='ignore').strip())
            except Exception as e:
                # On Ctrl+C ROS shuts down before close() stops this thread,
                # so the last publish can fail. That's expected, just exit.
                if not self._running or not rclpy.ok():
                    break
                self.get_logger().error(f'Error handling "{line!r}": {e}')

    def _close_serial(self):
        with self._write_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except (serial.SerialException, OSError):
                    pass
                self.ser = None

    def _write(self, text):
        with self._write_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(text.encode('ascii'))
            except (serial.SerialException, OSError):
                pass  # the reader thread notices and reconnects

    def _handle_line(self, line):
        if line.startswith('L '):
            try:
                raws = [int(v) for v in line[2:].split()]
            except ValueError:
                return  # garbled line, e.g. right after the ESP boots
            if len(raws) != config.NUM_CELLS:
                self.get_logger().error(
                    f'ESP sends {len(raws)} cells but NUM_CELLS is {config.NUM_CELLS}',
                    throttle_duration_sec=10.0,
                )
                return
            self._handle_sample(raws)
        elif line.startswith('I '):
            self.get_logger().info(f'ESP: {line[2:]}')
        elif line.startswith('E '):
            self.get_logger().warn(f'ESP: {line[2:]}')

    def _handle_sample(self, raws):
        self.raw_pub.publish(Int32MultiArray(data=raws))

        with self._lock:
            if self._tare_samples is not None:
                for s, raw in zip(self._tare_samples, raws):
                    s.append(raw)
                if len(self._tare_samples[0]) >= config.CALIBRATION_SAMPLES:
                    self.cal['offsets'] = [median(s) for s in self._tare_samples]
                    for w in self.windows:
                        w.clear()
                    self._tare_done.set()
                    self._tare_samples = self._tare_done = None
                return

        for w, raw in zip(self.windows, raws):
            w.append(raw)
        cal = self.cal
        if cal is None:
            return
        kg = cell_weights([median(w) for w in self.windows], cal)
        self.weight_pub.publish(Float32(data=float(sum(kg))))
        self.cells_pub.publish(Float32MultiArray(data=[float(k) for k in kg]))

    # ---- motors --------------------------------------------------------------

    def on_motor_cmd(self, i, msg):
        self.speeds[i] = max(-1.0, min(1.0, msg.data))
        self.last_cmd[i] = time.monotonic()

    def send_motors(self):
        # Sent every tick, also when nothing changed: that keeps the ESP's
        # 500 ms watchdog fed. If this node dies, the ESP stops the motors.
        now = time.monotonic()
        for i in range(config.NUM_MOTORS):
            stale = self.last_cmd[i] is not None and now - self.last_cmd[i] > config.COMMAND_TIMEOUT_S
            if config.COMMAND_TIMEOUT_S > 0 and stale and self.speeds[i] != 0.0:
                self.speeds[i] = 0.0
                self.get_logger().warn(
                    f'motor {i}: no command for {config.COMMAND_TIMEOUT_S}s, stopped'
                )
        self._write('M ' + ' '.join(f'{s:.3f}' for s in self.speeds) + '\n')

    def close(self):
        self._write('M ' + ' '.join(['0'] * config.NUM_MOTORS) + '\n')
        self._running = False
        self._thread.join(timeout=1.0)
        self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = EspDriverNode()
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
