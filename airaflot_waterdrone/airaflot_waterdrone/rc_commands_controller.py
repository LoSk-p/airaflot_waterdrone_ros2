import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mavros_msgs.msg import RCIn

NODE_NAME = "rc_command_controller"
RC_IN_TOPIC_NAME = "/mavros/rc/in"
RC_COMMANDS_CHANNEL_INDEX = 9 - 1

class RCCommandsController(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            RCIn,
            RC_IN_TOPIC_NAME,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: RCIn):
        self.get_logger().info(f"Channels: {msg.channels}, channel 9: {msg.channels[8]}")


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = RCCommandsController()

        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()