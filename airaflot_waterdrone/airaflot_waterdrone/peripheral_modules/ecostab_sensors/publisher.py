import rclpy
from rclpy.node import Node
import typing as tp

from airaflot_msgs.msg import EcostabSensors

from .sensors import Sensor, pHSensor, ConductivitySensor
from ...const_names import ECOSTAB_SENSORS_TOPIC_NAME

NODE_NAME = "ecostab_sensors"

class EcostabSensorsNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.sensors: tp.List[Sensor] = [pHSensor(), ConductivitySensor()]
        self.publisher_ = self.create_publisher(EcostabSensors, ECOSTAB_SENSORS_TOPIC_NAME, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = EcostabSensors()
        for sensor in self.sensors:
            msg = sensor.fetch(msg)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = EcostabSensorsNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()