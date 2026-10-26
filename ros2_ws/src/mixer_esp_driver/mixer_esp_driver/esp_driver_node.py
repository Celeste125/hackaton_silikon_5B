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
  Speed changes are ramped here (config.RAMP_PER_S); the ESP applies every
  speed instantly.
  /mixer/shake  std_msgs/Float32MultiArray [fwd_ms, rev_ms, speed0, speed1]
                Unjam pattern: +speed for fwd_ms, -speed for rev_ms, repeat,
                no ramp. Overrides /mixer/motor_* while it is fresh; publish
                an empty array to stop it.
  e.g. ros2 topic pub -r 5 /mixer/shake std_msgs/msg/Float32MultiArray "data: [150, 150, 1.0, 1.0]"

Door servo
  /door/position  std_msgs/Float32  0.0 = fully open, 1.0 = fully closed
  /door/raw_us    std_msgs/Float32  pulse width in us, for finding the end
                  positions (jog it, then save). Overrides /door/position
                  until the next /door/position message.
  /door/save_open, /door/save_closed  std_srvs/Trigger  store the current
                  pulse width as the open / closed end (door_calibration.json)
  Calibrate: ros2 topic pub --once /door/raw_us std_msgs/Float32 "data: 1500"
             (adjust until the door is fully open) ros2 service call /door/save_open std_srvs/srv/Trigger
             same for closed. Then: ros2 topic pub --once /door/position std_msgs/Float32 "data: 1.0"

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
from mixer_esp_driver.calibration import (
    calibration_path, cell_weights, door_position_to_us, load_calibration,
    load_door_calibration, save_calibration, save_door_calibration,
)


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

        self.speeds = [0.0] * config.NUM_MOTORS  # targets from the topics
        self.output = [0.0] * config.NUM_MOTORS  # what the ESP was last sent
        self.last_cmd = [None] * config.NUM_MOTORS
        for i in range(config.NUM_MOTORS):
            self.create_subscription(
                Float32, f'mixer/motor_{i}',
                lambda msg, i=i: self.on_motor_cmd(i, msg), 10,
            )
        self.pattern = None  # (fwd_ms, rev_ms, [speeds]) while shaking
        self.pattern_start = 0.0
        self.last_pattern = None
        self.create_subscription(Float32MultiArray, 'mixer/shake', self.on_shake, 10)
        self._last_tick = time.monotonic()
        self._last_send = 0.0
        self.create_timer(1.0 / config.MOTOR_LOOP_HZ, self.update_motors)

        # Door servo. _servo_us is None until the first command, so the servo
        # is not driven (and doesn't jump) before someone asks for a position.
        try:
            self.door_cal = load_door_calibration()
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'Bad door calibration file: {e}, using defaults')
            self.door_cal = {
                'open_us': config.SERVO_DEFAULT_OPEN_US,
                'closed_us': config.SERVO_DEFAULT_CLOSED_US,
            }
        self._servo_us = None
        self.create_subscription(Float32, 'door/position', self.on_door_position, 10)
        self.create_subscription(Float32, 'door/raw_us', self.on_door_raw, 10)
        self.create_service(Trigger, 'door/save_open', self.on_save_open)
        self.create_service(Trigger, 'door/save_closed', self.on_save_closed)

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
                f'No calibration at {calibration_path()}: only /loadcell/raw '
                f'is published. Run: ros2 run mixer_esp_driver calibrate'
            )
            return False
        self.get_logger().info(f'Loaded calibration from {calibration_path()}')
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

    def on_shake(self, msg):
        if not msg.data:
            self.pattern = None
            return
        if len(msg.data) != 2 + config.NUM_MOTORS:
            self.get_logger().error(
                f'mixer/shake needs [fwd_ms, rev_ms] + {config.NUM_MOTORS} speeds',
                throttle_duration_sec=5.0,
            )
            return
        fwd_ms, rev_ms = (max(0.0, min(10000.0, v)) for v in msg.data[:2])
        if fwd_ms + rev_ms < 20:
            self.get_logger().error('mixer/shake: fwd_ms + rev_ms must be >= 20',
                                    throttle_duration_sec=5.0)
            return
        pattern = (fwd_ms, rev_ms, [max(-1.0, min(1.0, v)) for v in msg.data[2:]])
        now = time.monotonic()
        # The web panel resends the same pattern every 100 ms; keep its phase,
        # or it would restart before ever reaching the reverse half.
        if pattern != self.pattern:
            self.pattern_start = now
        self.pattern = pattern
        self.last_pattern = now
        # Back to standstill, not the old speeds, when the pattern ends.
        self.speeds = [0.0] * config.NUM_MOTORS

    def _shake_output(self, now):
        fwd_ms, rev_ms, speeds = self.pattern
        phase_ms = ((now - self.pattern_start) * 1000.0) % (fwd_ms + rev_ms)
        sign = 1.0 if phase_ms < fwd_ms else -1.0
        return [sign * s for s in speeds]

    def _ramp_output(self, dt):
        if config.RAMP_PER_S <= 0:
            return list(self.speeds)
        step = config.RAMP_PER_S * dt
        return [o + max(-step, min(step, t - o)) for o, t in zip(self.output, self.speeds)]

    def update_motors(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now

        if (self.pattern is not None and config.COMMAND_TIMEOUT_S > 0
                and now - self.last_pattern > config.COMMAND_TIMEOUT_S):
            self.pattern = None
            self.get_logger().warn(f'shake: no command for {config.COMMAND_TIMEOUT_S}s, stopped')
        for i in range(config.NUM_MOTORS):
            stale = self.last_cmd[i] is not None and now - self.last_cmd[i] > config.COMMAND_TIMEOUT_S
            if config.COMMAND_TIMEOUT_S > 0 and stale and self.speeds[i] != 0.0:
                self.speeds[i] = 0.0
                self.get_logger().warn(
                    f'motor {i}: no command for {config.COMMAND_TIMEOUT_S}s, stopped'
                )

        # Shake flips direction instantly; everything else ramps, also out
        # of a shake back to standstill.
        output = self._shake_output(now) if self.pattern is not None else self._ramp_output(dt)
        changed = output != self.output
        self.output = output
        # Resent at MOTOR_SEND_HZ also when nothing changed: that keeps the
        # ESP's 500 ms watchdog fed. If this node dies, the ESP stops the motors.
        keepalive = now - self._last_send >= 1.0 / config.MOTOR_SEND_HZ
        if changed or keepalive:
            self._write('M ' + ' '.join(f'{s:.3f}' for s in output) + '\n')
            self._last_send = now
        # Resent too, so the servo recovers if the ESP resets.
        if keepalive and self._servo_us is not None:
            self._write(f'S {self._servo_us:.0f}\n')

    # ---- door servo ----------------------------------------------------------

    def on_door_position(self, msg):
        self._servo_us = door_position_to_us(msg.data, self.door_cal)
        self._write(f'S {self._servo_us:.0f}\n')

    def on_door_raw(self, msg):
        self._servo_us = float(msg.data)
        self._write(f'S {self._servo_us:.0f}\n')

    def _save_door_end(self, key, response):
        if self._servo_us is None:
            response.success = False
            response.message = 'no servo position yet, publish /door/raw_us first'
            return response
        self.door_cal[key] = self._servo_us
        save_door_calibration(self.door_cal)
        self.get_logger().info(f'door {key} = {self._servo_us:.0f}')
        response.success, response.message = True, f'{key} = {self._servo_us:.0f}'
        return response

    def on_save_open(self, request, response):
        return self._save_door_end('open_us', response)

    def on_save_closed(self, request, response):
        return self._save_door_end('closed_us', response)

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
