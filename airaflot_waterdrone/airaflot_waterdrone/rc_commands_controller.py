import typing as tp
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mavros_msgs.msg import RCIn
from airaflot_msgs.srv import WaterSampler

from .config_channels import WATER_SAMPLER_CHANNEL_NUMBER, TASK_1_CHANNEL, TASK_2_CHANNEL, TASK_3_CHANNEL, TASK_4_CHANNEL, CHANNELS_DIFF
from .const_names import RUN_WATER_SAMPLER_SERVICE_NAME

NODE_NAME = "rc_command_controller"
RC_IN_TOPIC_NAME = "/mavros/rc/in"
WATER_SAMPLER_CHANNEL_INDEX = WATER_SAMPLER_CHANNEL_NUMBER - 1

class RCCommandsController(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.subscription = self.create_subscription(
            RCIn,
            RC_IN_TOPIC_NAME,
            self.listener_callback,
            10)
        self.water_sampler_service_client = self.create_client(
            WaterSampler, RUN_WATER_SAMPLER_SERVICE_NAME
        )
        self.subscription  # prevent unused variable warning
        self.current_channel_value = 1000

    def listener_callback(self, msg: RCIn):
        self.get_logger().debug(f"Channels: {msg.channels}")
        if len(msg.channels) > 0:
            self.get_logger().debug(f"Channel 9: {msg.channels[WATER_SAMPLER_CHANNEL_INDEX]}")
            if abs(msg.channels[WATER_SAMPLER_CHANNEL_INDEX] - self.current_channel_value) > CHANNELS_DIFF:
                task_mode = self._get_task_mode(msg.channels[WATER_SAMPLER_CHANNEL_INDEX])
                if task_mode is not None:
                    self.get_logger().info(f"Got command to run Water Sampler service with mode {task_mode}")
                    request = WaterSampler.Request()
                    request.mode = task_mode
                    self.water_sampler_service_client.call_async(request)
            self.current_channel_value = msg.channels[WATER_SAMPLER_CHANNEL_INDEX]

    def _get_task_mode(self, channel_value: int) -> tp.Optional[int]:
        if abs(channel_value - TASK_1_CHANNEL) < 10:
            return 1
        if abs(channel_value - TASK_2_CHANNEL) < 10:
            return 2
        if abs(channel_value - TASK_3_CHANNEL) < 10:
            return 3
        if abs(channel_value - TASK_4_CHANNEL) < 10:
            return 4
        return None


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = RCCommandsController()

        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()