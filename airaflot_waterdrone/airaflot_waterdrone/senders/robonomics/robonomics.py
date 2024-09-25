from rclpy.node import Node
import json

from airaflot_msgs.msg import DataToSend
from std_msgs.msg import String

from .helpers import Robonomics, IPFS, read_formatted_data
from .config import USE_FILE_SAVER_WITH_IPFS
from ...const_names import DATA_TO_SEND_TOPIC_NAME, FILE_FINISHED_TOPIC_NAME

NODE_NAME = "robonomics"

class RobonomicsNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ### Topic Subscribers ###

        if USE_FILE_SAVER_WITH_IPFS:
            self.data_to_send_subscription = self.create_subscription(
                DataToSend, DATA_TO_SEND_TOPIC_NAME, self._data_to_send_callback, 10
            )
            self.data_to_send_subscription
        else:
            self.file_finished_subscription = self.create_subscription(
                String, FILE_FINISHED_TOPIC_NAME, self._file_finished_callback, 10
            )
            self.file_finished_subscription

        ### Other Setup ###

        self._robonomics = Robonomics()

    def _data_to_send_callback(self, data: DataToSend) -> None:
        formatted_data = self._format_data_to_string(data)
        self._robonomics.record_datalog(formatted_data)

    def _format_data_to_string(self, data: DataToSend) -> str:
        data_json = json.loads(data.sensors_data)
        data_json["timestamp"] = data.timestamp
        data_json["gps"] = (data.latitude, data.longitude)
        return json.dumps(data_json)

    def _file_finished_callback(self, data: String) -> None:
        formatted_data = read_formatted_data(data.data, self._robonomics.get_public_key())
        ipfs_hash = IPFS.add_json(formatted_data, self._logger)
        self._robonomics.record_datalog(ipfs_hash)