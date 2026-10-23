"""Drive the 2 mixer motors through BTS7960 (IBT-2) boards.

Topics (std_msgs/Float32, -1.0 .. 1.0, negative = reverse):
  /mixer/motor_0
  /mixer/motor_1
Example:
  ros2 topic pub -r 5 /mixer/motor_0 std_msgs/Float32 "data: 0.5"
"""

import glob

import lgpio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

# ---- Wiring: edit here, then colcon build --------------------------------
# Each BTS7960 board: VCC -> Pi 5V (or 3.3V if it doesn't react reliably),
# GND -> Pi GND (required), R_IS/L_IS unconnected,
# B+/B- -> motor battery, M+/M- -> motor.
# BCM GPIO numbers (not physical pin numbers).
MOTORS = [
    {   # motor 0 -> /mixer/motor_0
        'rpwm': 12,    # RPWM -> GPIO12 (pin 32), forward PWM
        'lpwm': 16,    # LPWM -> GPIO16 (pin 36), reverse PWM
        'en': None,    # R_EN + L_EN: None = both tied to Pi 3.3V, or a GPIO
    },
    {   # motor 1 -> /mixer/motor_1
        'rpwm': 20,    # RPWM -> GPIO20 (pin 38), forward PWM
        'lpwm': 21,    # LPWM -> GPIO21 (pin 40), reverse PWM
        'en': None,    # R_EN + L_EN: None = both tied to Pi 3.3V, or a GPIO
    },
]

# /dev/gpiochipN with the header pins. None = auto-detect (Pi 3/4/5).
GPIOCHIP = None

PWM_FREQ_HZ = 2000

# Safety: a motor stops if it gets no command for this long (e.g. the
# commanding node crashed). Keep publishing at a few Hz. 0 = never stop.
COMMAND_TIMEOUT_S = 1.0
# ---------------------------------------------------------------------------


def open_header_chip(chip=None):
    """Open the gpiochip that carries the 40-pin header."""
    if chip is not None:
        return lgpio.gpiochip_open(chip)
    for path in sorted(glob.glob('/dev/gpiochip*')):
        handle = lgpio.gpiochip_open(int(path[len('/dev/gpiochip'):]))
        label = lgpio.gpio_get_chip_info(handle)[3]
        if any(name in label for name in ('rp1', 'bcm2711', 'bcm2835')):
            return handle
        lgpio.gpiochip_close(handle)
    raise RuntimeError(
        'No Raspberry Pi GPIO chip found. In Docker, pass --device /dev/gpiochipN.'
    )


class Motor:
    def __init__(self, handle, rpwm, lpwm, en=None):
        self.handle = handle
        self.rpwm = rpwm
        self.lpwm = lpwm
        self.en = en
        self.speed = 0.0
        lgpio.gpio_claim_output(handle, rpwm, 0)
        lgpio.gpio_claim_output(handle, lpwm, 0)
        if en is not None:
            lgpio.gpio_claim_output(handle, en, 1)

    def _pwm(self, pin, duty):
        if duty <= 0:
            lgpio.tx_pwm(self.handle, pin, 0, 0)
            lgpio.gpio_write(self.handle, pin, 0)
        else:
            lgpio.tx_pwm(self.handle, pin, PWM_FREQ_HZ, duty)

    def set(self, speed):
        speed = max(-1.0, min(1.0, speed))
        if speed == self.speed:
            return  # restarting PWM on every message would cause glitches
        duty = abs(speed) * 100.0
        # Turn the active side off first so both are never driven at once.
        if speed >= 0:
            self._pwm(self.lpwm, 0)
            self._pwm(self.rpwm, duty)
        else:
            self._pwm(self.rpwm, 0)
            self._pwm(self.lpwm, duty)
        self.speed = speed

    def close(self):
        self._pwm(self.rpwm, 0)
        self._pwm(self.lpwm, 0)
        if self.en is not None:
            lgpio.gpio_write(self.handle, self.en, 0)


class MixerMotorNode(Node):
    def __init__(self):
        super().__init__('mixer_motor_node')
        self.handle = open_header_chip(GPIOCHIP)
        self.motors = [Motor(self.handle, **m) for m in MOTORS]
        self.last_cmd = [None] * len(self.motors)

        for i in range(len(self.motors)):
            self.create_subscription(
                Float32, f'mixer/motor_{i}',
                lambda msg, i=i: self.on_cmd(i, msg), 10,
            )
        if COMMAND_TIMEOUT_S > 0:
            self.create_timer(0.1, self.check_timeout)
        self.get_logger().info(
            f'{len(self.motors)} motors ready on /mixer/motor_0..{len(self.motors) - 1}'
        )

    def on_cmd(self, i, msg):
        self.motors[i].set(msg.data)
        self.last_cmd[i] = self.get_clock().now()

    def check_timeout(self):
        now = self.get_clock().now()
        for i, motor in enumerate(self.motors):
            stamp = self.last_cmd[i]
            if stamp is None or motor.speed == 0.0:
                continue
            if (now - stamp).nanoseconds * 1e-9 > COMMAND_TIMEOUT_S:
                motor.set(0.0)
                self.get_logger().warn(
                    f'motor {i}: no command for {COMMAND_TIMEOUT_S}s, stopped'
                )

    def close(self):
        for motor in self.motors:
            motor.close()
        lgpio.gpiochip_close(self.handle)


def main(args=None):
    rclpy.init(args=args)
    node = MixerMotorNode()
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
