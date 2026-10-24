#!/usr/bin/env python3
"""
ROS 2 valve controller
Raspberry Pi 5 + PCA9685 + FITEC FS5106B servos

Topics per valve (example for /valve2):
  /valve2/percent std_msgs/Float32  how far to open: 0 = closed ... 100 = fully open
                                    (e.g. 50 = half open; values outside 0-100 are capped)
  /valve2         std_msgs/Bool     true = open (100 %), false = closed (0 %)
  /valve2/calib   std_msgs/String   calibration commands:
                                      "jog <deg>"   move relative, e.g. "jog 5", "jog -2"
                                      "goto <deg>"  move to an absolute angle
                                      "set_closed"  save current position as CLOSED
                                      "set_open"    save current position as OPEN
  /valve2/angle   std_msgs/Float32  (published) current commanded angle, for feedback

Angles: 0-180 maps to min_pulse_us-max_pulse_us (900-2100 us, the rated range
of the FITEC FS5106B, about 120 degrees of real rotation). The same scale
continues beyond that in both directions without any limit (limit_angles =
false). Only the PCA9685 itself caps the pulse (0 to one full 20 ms period).
Note: hobby servos normally only respond to roughly 500-2500 us
(angle -60..240); far outside that they ignore the signal or push into
their end stop. To re-enable limits: -p limit_angles:=true.

Calibration file:
  Positions saved with set_closed / set_open are stored in `calib_file`
  (JSON, keyed by PCA9685 channel) and loaded at startup. Saved values take
  priority over the open_angle / closed_angle parameters. Mount a folder at
  /data in Docker so the file survives restarts.

Servo behaviour:
  hold = false (default): after each move the PWM signal is switched off after
    `move_time_s`, so the servo goes quiet. The valve stays put by friction.
  hold = true: the servo keeps actively holding its position.
  On exit the valves are closed (if close_on_exit) and the PCA9685 is put to sleep.

Parameters (override with --ros-args -p name:=value):
  i2c_bus        int        I2C bus number               (default 1)
  i2c_address    int        PCA9685 address              (default 0x40 = 64)
  channels       int[]      PCA9685 channel per valve    (default [13, 14, 15])
  topics         string[]   topic per valve              (default [valve1, valve2, valve3])
  open_angle     int[]      angle for "open"             (default [140, 115, 55])
  closed_angle   int[]      angle for "closed"           (default [285, 230, 230])
  only           int        use only valve N (1-based), 0 = all   (default 0)
  min_pulse_us   int        pulse at angle 0             (default 900)
  max_pulse_us   int        pulse at angle 180           (default 2100)
  limit_angles   bool       enforce min_angle/max_angle  (default false = unlimited)
  min_angle      double     lowest angle if limited      (default -60  = 500 us)
  max_angle      double     highest angle if limited     (default 240  = 2500 us)
  hold           bool       keep holding after a move    (default false)
  move_time_s    double     time to finish a move        (default 1.0)
  close_on_start bool       close valves at startup      (default true)
  close_on_exit  bool       close valves on shutdown     (default true)
  timeout_s      double     close all if no command for this long, 0 = off (default 0.0)
  calib_file     string     calibration file, "" = off   (default /data/valve_calibration.json)
"""

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from smbus2 import SMBus
from std_msgs.msg import Bool, Float32, String


class PCA9685:
    """Minimal PCA9685 driver over smbus2."""

    MODE1 = 0x00
    MODE2 = 0x01
    LED0_ON_L = 0x06
    ALL_LED_ON_L = 0xFA
    PRESCALE = 0xFE
    OSC_HZ = 25_000_000  # internal oscillator

    def __init__(self, bus: int = 1, address: int = 0x40, freq_hz: float = 50.0):
        self.bus = SMBus(bus)
        self.addr = address

        # Switch every output off first, so channels this node does not use
        # (or old values from a previous run) don't start moving servos.
        self.all_off()

        self.bus.write_byte_data(self.addr, self.MODE2, 0x04)  # totem-pole outputs
        self.bus.write_byte_data(self.addr, self.MODE1, 0x00)  # normal mode
        time.sleep(0.005)

        prescale = round(self.OSC_HZ / (4096 * freq_hz)) - 1
        old = self.bus.read_byte_data(self.addr, self.MODE1) & 0x7F
        self.bus.write_byte_data(self.addr, self.MODE1, old | 0x10)  # sleep to set prescaler
        self.bus.write_byte_data(self.addr, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, self.MODE1, old)          # wake
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, self.MODE1, old | 0xA0)  # restart + auto-increment

        actual_freq = self.OSC_HZ / (4096 * (prescale + 1))
        self.period_us = 1_000_000 / actual_freq

    def set_pulse_us(self, channel: int, pulse_us: float) -> None:
        ticks = int(round(pulse_us * 4096 / self.period_us))
        ticks = max(0, min(4095, ticks))
        reg = self.LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, ticks & 0xFF, ticks >> 8])

    def release(self, channel: int) -> None:
        """Stop pulses on one channel (servo goes limp and quiet)."""
        reg = self.LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(self.addr, reg, [0, 0, 0, 0x10])

    def all_off(self) -> None:
        """Stop pulses on all 16 channels."""
        for offset, value in enumerate((0, 0, 0, 0x10)):
            self.bus.write_byte_data(self.addr, self.ALL_LED_ON_L + offset, value)

    def sleep(self) -> None:
        """All outputs off and oscillator stopped (same as after power-up)."""
        self.all_off()
        mode1 = self.bus.read_byte_data(self.addr, self.MODE1)
        self.bus.write_byte_data(self.addr, self.MODE1, (mode1 & 0x7F) | 0x10)

    def close(self) -> None:
        self.bus.close()


class ValveController(Node):
    def __init__(self):
        super().__init__('valve_controller')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('channels', [13, 14, 15])
        self.declare_parameter('topics', ['valve1', 'valve2', 'valve3'])
        # Calibrated positions (measured): valve1 ch13, valve2 ch14, valve3 ch15
        self.declare_parameter('open_angle', [140, 115, 55])
        self.declare_parameter('closed_angle', [285, 230, 230])
        self.declare_parameter('only', 0)
        self.declare_parameter('min_pulse_us', 900)
        self.declare_parameter('max_pulse_us', 2100)
        self.declare_parameter('limit_angles', False)
        self.declare_parameter('min_angle', -60.0)
        self.declare_parameter('max_angle', 240.0)
        self.declare_parameter('hold', False)
        self.declare_parameter('move_time_s', 1.0)
        self.declare_parameter('close_on_start', True)
        self.declare_parameter('close_on_exit', True)
        self.declare_parameter('timeout_s', 0.0)
        self.declare_parameter('calib_file', '/data/valve_calibration.json')

        get = lambda name: self.get_parameter(name).value
        self.channels = list(get('channels'))
        self.topics = list(get('topics'))
        self.open_angle = [float(a) for a in get('open_angle')]
        self.closed_angle = [float(a) for a in get('closed_angle')]
        self.min_us = get('min_pulse_us')
        self.max_us = get('max_pulse_us')
        self.limit_angles = bool(get('limit_angles'))
        self.angle_min = float(get('min_angle'))
        self.angle_max = float(get('max_angle'))
        self.hold = bool(get('hold'))
        self.move_time_s = float(get('move_time_s'))
        self.close_on_exit = bool(get('close_on_exit'))
        self.timeout_s = float(get('timeout_s'))
        self.calib_file = get('calib_file')

        n = len(self.channels)
        if not (len(self.topics) == len(self.open_angle) == len(self.closed_angle) == n):
            raise ValueError('channels, topics, open_angle and closed_angle must have the same length')

        only = int(get('only'))
        if only:
            if not 1 <= only <= n:
                raise ValueError(f'only must be between 1 and {n}')
            k = only - 1
            self.channels = [self.channels[k]]
            self.topics = [self.topics[k]]
            self.open_angle = [self.open_angle[k]]
            self.closed_angle = [self.closed_angle[k]]
            n = 1

        self.load_calibration()

        self.pca = PCA9685(get('i2c_bus'), get('i2c_address'))
        self.get_logger().info(
            f"PCA9685 ready on bus {get('i2c_bus')}, address 0x{get('i2c_address'):02X}, "
            f"pulse {self.min_us}-{self.max_us} us, "
            + (f"angle limits {self.angle_min:g}..{self.angle_max:g}"
               if self.limit_angles else "angle limits OFF (unlimited)")
            + f", hold={self.hold}")

        self.state = [None] * n          # percent open (0-100), None = unknown / calibrating
        self.angle = [None] * n          # last commanded angle
        self.release_timers = [None] * n
        self.angle_pubs = []
        self.last_cmd = time.monotonic()
        self.timed_out = False

        for i, topic in enumerate(self.topics):
            self.angle_pubs.append(self.create_publisher(Float32, f'{topic}/angle', 10))

        if get('close_on_start'):
            for i in range(n):
                self.set_valve(i, False)

        for i, topic in enumerate(self.topics):
            self.create_subscription(Float32, f'{topic}/percent', lambda msg, i=i: self.on_percent(i, msg), 10)
            self.create_subscription(Bool, topic, lambda msg, i=i: self.on_command(i, msg), 10)
            self.create_subscription(String, f'{topic}/calib', lambda msg, i=i: self.on_calib(i, msg), 10)
            self.get_logger().info(
                f'/{topic} -> channel {self.channels[i]}  '
                f'(0% = closed {self.closed_angle[i]:g} deg, 100% = open {self.open_angle[i]:g} deg)')

        if self.timeout_s > 0:
            self.create_timer(0.1, self.watchdog)
            self.get_logger().info(f'Fail-safe: close all valves after {self.timeout_s:.1f} s without commands')

    # ---------- calibration file ----------

    def load_calibration(self) -> None:
        if not self.calib_file:
            return
        try:
            with open(self.calib_file) as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().info(f'No calibration file yet ({self.calib_file}), using parameters')
            return
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'Could not read {self.calib_file}: {e}; using parameters')
            return

        for i, ch in enumerate(self.channels):
            entry = data.get(str(ch), {})
            if 'open' in entry:
                self.open_angle[i] = float(entry['open'])
            if 'closed' in entry:
                self.closed_angle[i] = float(entry['closed'])
            if entry:
                self.get_logger().info(f'Loaded calibration for channel {ch}: {entry}')

    def save_calibration(self) -> bool:
        if not self.calib_file:
            self.get_logger().warn('calib_file is empty: position kept until restart, not saved')
            return False
        try:
            data = {}
            if os.path.exists(self.calib_file):
                with open(self.calib_file) as f:
                    data = json.load(f)
            for i, ch in enumerate(self.channels):
                data[str(ch)] = {'open': self.open_angle[i], 'closed': self.closed_angle[i]}
            folder = os.path.dirname(self.calib_file)
            if folder:
                os.makedirs(folder, exist_ok=True)
            tmp = self.calib_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self.calib_file)
            return True
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'Could not save {self.calib_file}: {e} (kept until restart)')
            return False

    # ---------- servo movement ----------

    def angle_to_us(self, angle: float) -> float:
        angle = self.clamp(angle)
        pulse = self.min_us + (self.max_us - self.min_us) * angle / 180.0
        return max(0.0, pulse)  # the PCA9685 caps the upper end at one full period

    def clamp(self, angle: float) -> float:
        if not self.limit_angles:
            return float(angle)
        return max(self.angle_min, min(self.angle_max, float(angle)))

    def move_to(self, i: int, angle: float) -> None:
        if self.limit_angles and not self.angle_min <= float(angle) <= self.angle_max:
            self.get_logger().warn(
                f'/{self.topics[i]}: {float(angle):g} deg is outside the allowed '
                f'{self.angle_min:g}..{self.angle_max:g}, limited')
        angle = self.clamp(angle)
        self.pca.set_pulse_us(self.channels[i], self.angle_to_us(angle))
        self.angle[i] = angle
        self.angle_pubs[i].publish(Float32(data=angle))
        if not self.hold:
            self.schedule_release(i)

    def set_percent(self, i: int, percent: float) -> None:
        """0 = fully closed, 100 = fully open, anything in between opens partly."""
        percent = max(0.0, min(100.0, float(percent)))
        # Without hold, repeated identical commands are ignored, so a rover that
        # publishes continuously does not keep re-energising a quiet servo.
        if not self.hold and self.state[i] == percent:
            return
        closed, open_ = self.closed_angle[i], self.open_angle[i]
        angle = closed + (open_ - closed) * percent / 100.0
        changed = self.state[i] != percent
        self.state[i] = percent
        self.move_to(i, angle)
        if changed:
            label = {0.0: 'CLOSED', 100.0: 'OPEN'}.get(percent, f'{percent:g}% open')
            self.get_logger().info(f'/{self.topics[i]} {label} ({angle:g} deg)')

    def set_valve(self, i: int, open_: bool) -> None:
        self.set_percent(i, 100.0 if open_ else 0.0)

    def schedule_release(self, i: int) -> None:
        old = self.release_timers[i]
        if old is not None:
            old.cancel()
            self.destroy_timer(old)

        timer = None

        def on_release():
            timer.cancel()
            self.pca.release(self.channels[i])

        timer = self.create_timer(self.move_time_s, on_release)
        self.release_timers[i] = timer

    # ---------- callbacks ----------

    def command_received(self) -> None:
        self.last_cmd = time.monotonic()
        if self.timed_out:
            self.timed_out = False
            self.get_logger().info('Commands received again, fail-safe cleared')

    def on_command(self, i: int, msg: Bool) -> None:
        self.command_received()
        self.set_valve(i, msg.data)

    def on_percent(self, i: int, msg: Float32) -> None:
        value = float(msg.data)
        if value != value:  # NaN
            self.get_logger().warn(f'/{self.topics[i]}/percent: ignoring NaN')
            return
        if not 0.0 <= value <= 100.0:
            self.get_logger().warn(f'/{self.topics[i]}/percent: {value:g} is outside 0-100, capped')
        self.command_received()
        self.set_percent(i, value)

    def on_calib(self, i: int, msg: String) -> None:
        name = f'/{self.topics[i]}'
        parts = msg.data.strip().lower().split()
        if not parts:
            return
        cmd = parts[0]
        try:
            if cmd == 'jog' and len(parts) == 2:
                base = self.angle[i] if self.angle[i] is not None else self.closed_angle[i]
                self.state[i] = None
                self.move_to(i, base + float(parts[1]))
                self.get_logger().info(f'{name} jog {float(parts[1]):+g} -> {self.angle[i]:g} deg')
            elif cmd == 'goto' and len(parts) == 2:
                self.state[i] = None
                self.move_to(i, float(parts[1]))
                self.get_logger().info(f'{name} goto -> {self.angle[i]:g} deg')
            elif cmd in ('set_closed', 'set_open') and len(parts) == 1:
                if self.angle[i] is None:
                    self.get_logger().warn(f'{name}: move the servo first (jog/goto), then save')
                    return
                if cmd == 'set_closed':
                    self.closed_angle[i] = self.angle[i]
                    self.state[i] = 0.0
                    what = 'CLOSED'
                else:
                    self.open_angle[i] = self.angle[i]
                    self.state[i] = 100.0
                    what = 'OPEN'
                saved = self.save_calibration()
                self.get_logger().info(
                    f"{name} {what} position set to {self.angle[i]:g} deg"
                    f"{' and saved' if saved else ''}")
                self.angle_pubs[i].publish(Float32(data=self.angle[i]))
            else:
                self.get_logger().warn(f'{name}: unknown calib command "{msg.data}"')
        except ValueError:
            self.get_logger().warn(f'{name}: bad number in "{msg.data}"')

    def watchdog(self) -> None:
        if not self.timed_out and time.monotonic() - self.last_cmd > self.timeout_s:
            self.timed_out = True
            self.get_logger().warn('No valve commands received, closing all valves')
            for i in range(len(self.channels)):
                self.set_valve(i, False)

    def shutdown(self) -> None:
        try:
            if self.close_on_exit:
                for i, ch in enumerate(self.channels):
                    self.pca.set_pulse_us(ch, self.angle_to_us(self.closed_angle[i]))
                time.sleep(self.move_time_s)  # let the servos reach closed
        finally:
            try:
                self.pca.sleep()  # no pulses at all while the node is not running
            finally:
                self.pca.close()


def main():
    rclpy.init()
    node = ValveController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
