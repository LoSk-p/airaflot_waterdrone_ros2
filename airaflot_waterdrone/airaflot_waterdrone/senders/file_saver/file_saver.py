import typing as tp
import time
import json
import os
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from datetime import datetime, date, timedelta

from airaflot_msgs.msg import DataToSend
from std_msgs.msg import String

from ...const_names import (
    DATA_TO_SEND_TOPIC_NAME,
    FILE_FINISHED_TOPIC_NAME,
)
from .config import MAX_MEASUREMENTS_COUNT, NEW_FILE_TIMEOUT, FILE_NAME_PREFIX, STORE_FOLDER_NAME, STORE_FILES_PATH
from .filename_util import Filename

NODE_NAME = "file_saver"


class FileSaver(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        ### Topic Subscribers ###

        self.data_subscription = self.create_subscription(
            DataToSend, DATA_TO_SEND_TOPIC_NAME, self._write_data, 10
        )
        self.data_subscription  # prevent unused variable warning

        ### Topic Publishers ###

        self.publisher = self.create_publisher(String, FILE_FINISHED_TOPIC_NAME, 10)

        ### Other Setup ###

        self._current_date: str = str(date.today())
        self._create_folders()
        self._current_filename: Filename = self._create_new_file()

    def _write_data(self, data: DataToSend) -> None:
        self._check_current_file_and_create_new_if_need()
        json_new_data = self._format_data_to_json(data)
        file_data = self._get_data_from_file()
        file_data["measurements"].append(json_new_data)
        self._write_data_to_file(file_data)

    def _format_data_to_json(self, data: DataToSend) -> tp.Dict:
        json_data = json.loads(data.sensors_data)
        json_data["gps"] = (data.latitude, data.longitude)
        json_data["timestamp"] = data.timestamp
        return json_data
    
    def _check_current_file_and_create_new_if_need(self) -> None:
        self._publish_info_about_finished_file()
        time_since_creation = datetime.now() - self._current_filename.get_date()
        file_data = self._get_data_from_file()
        if time_since_creation > NEW_FILE_TIMEOUT or len(file_data["measurements"]) > MAX_MEASUREMENTS_COUNT:
            self._current_filename = self._create_new_file()

    def _publish_info_about_finished_file(self) -> None:
        msg = String()
        msg.data = self._current_filename.to_str()
        self.publisher.publish(msg)

    def _get_data_from_file(self) -> tp.Dict:
        with open(self._current_filename.to_str(), "r") as f:
            file_data = json.load(f)
        return file_data

    def _write_data_to_file(self, data: tp.Dict) -> tp.Dict:
        with open(self._current_filename.to_str(), "w") as f:
            json.dump(data, f)
    
    def _create_new_file(self) -> str:
        filename = Filename.create_new(self._current_date)
        with open(filename.to_str(), "w") as f:
            json.dump({"measurements": []}, f)
        self.get_logger().info(f"New file {filename.to_str()} was created")
        return filename
    
    def _create_folders(self) -> None:
        for folder in Filename.get_folders_to_create(self._current_date):
            self._create_folder_if_not_exists(folder)

    def _create_folder_if_not_exists(self, path: str) -> None:
        if not os.path.exists(path):
            os.mkdir(path)
            self.get_logger().info(f"Folder {path} was created")


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = FileSaver()

        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
