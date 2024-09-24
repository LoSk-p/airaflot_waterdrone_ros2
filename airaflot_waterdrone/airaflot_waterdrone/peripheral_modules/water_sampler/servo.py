import rclpy
import time

from rclpy.node import Node
import RPi.GPIO as GPIO

from std_srvs.srv import Trigger

from ..config_wiring import WATER_SAMPLER_SERVO_PIN
from ...const_names import CLOSE_SERVO_SERVICE_NAME, OPEN_AND_STOP_SERVO_SERVICE_NAME

NODE_NAME = "water_sampler_servo"

CLOSE_POSITION = 10
OPEN_POSITION = 6
OPEN_SERVO_DELAY = 0.5


class WaterSamplerServoNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rate = self.create_rate(OPEN_SERVO_DELAY)
        self.pwm = self._setup_gpio()
        self.service = self.create_service(
            Trigger, CLOSE_SERVO_SERVICE_NAME, self.close_servo
        )
        self.service = self.create_service(
            Trigger, OPEN_AND_STOP_SERVO_SERVICE_NAME, self.open_and_stop_servo
        )
        self.get_logger().info("Water Sampler Servo is ready")

    def open_and_stop_servo(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Start open servo")
        try:
            self.pwm.start(OPEN_POSITION)
            time.sleep(0.5)
            self.pwm.stop()
            response.success = True
            response.message = "ok"
        except Exception as e:
            self.get_logger().error(f"Open servo service failed with error {e}")
            response.success = False
            response.message = f"Open servo service failed with error {e}"
        finally:
            return response

    def close_servo(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Start close servo")
        try:
            self.pwm.start(CLOSE_POSITION)
            response.success = True
            response.message = "ok"
        except Exception as e:
            self.get_logger().error(f"Open servo service failed with error {e}")
            response.success = False
            response.message = f"Open servo service failed with error {e}"
        finally:
            return response

    def _setup_gpio(self) -> GPIO.PWM:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(WATER_SAMPLER_SERVO_PIN, GPIO.OUT)
        return GPIO.PWM(WATER_SAMPLER_SERVO_PIN, 50)


def main():
    try:
        rclpy.init()
        minimal_service = WaterSamplerServoNode()
        rclpy.spin(minimal_service)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
