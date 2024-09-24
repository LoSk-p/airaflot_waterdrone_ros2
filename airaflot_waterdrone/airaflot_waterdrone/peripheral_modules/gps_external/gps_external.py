import serial
import rclpy
from datetime import datetime, date

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from airaflot_msgs.msg import NMEAGPGGA

from ..config_wiring import GPS_EXTERNAL_PORT
from ...const_names import GPS_EXTERNAL_DATA_TOPIC_NAME

NODE_NAME = "gps_external"

START_COMMAND = "log com1 gpgga ontime 0.1\r\n".encode("ascii")
TIMER_PERIOD = 0.05


class EchoSounder(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.serial = serial.Serial(GPS_EXTERNAL_PORT, 115200)

        ### Topic Publishers ###

        self.publisher = self.create_publisher(
            NMEAGPGGA, GPS_EXTERNAL_DATA_TOPIC_NAME, 10
        )

        ### Other Setup ###

        self._send_start_request()
        self.timer = self.create_timer(TIMER_PERIOD, self.publish_data)

    def publish_data(self) -> None:
        self.get_logger().debug(f"In waiting: {self.serial.in_waiting}")
        if self.serial.in_waiting > 7:
            nmea_string = self.serial.readline().decode()
            self.get_logger().debug(f"New string: {nmea_string}")
            if nmea_string.startswith("$GPGGA"):
                nmea_message = self._parse_nmea_string(nmea_string)
                self.publisher.publish(nmea_message)

    def _parse_nmea_string(self, nmea_string: str) -> NMEAGPGGA:
        nmea_string = nmea_string.split(",")
        nmea_message = NMEAGPGGA()
        nmea_message.timestamp = self._get_timestamp(nmea_string[1])
        nmea_message.latitude = float(nmea_string[2])
        nmea_message.latitude_dir = nmea_string[3]
        nmea_message.longitude = float(nmea_string[4])
        nmea_message.longitude_dir = nmea_string[5]
        nmea_message.altitude = float(nmea_string[9])
        return nmea_message

    def _get_timestamp(self, utc_time_string: str) -> float:
        today = date.today()
        gps_datetime = datetime(
            today.year,
            today.month,
            today.day,
            int(utc_time_string[0:2]),
            int(utc_time_string[2:4]),
            int(utc_time_string[4:6]),
            int(utc_time_string[7:9]),
        )
        return gps_datetime.timestamp()

    def _send_start_request(self) -> None:
        try:
            self.serial.write(START_COMMAND)
        except Exception as e:
            self.get_logger().error(f"Exception in sending message: {e}")


def main():
    try:
        rclpy.init()
        minimal_service = EchoSounder()
        executor = MultiThreadedExecutor()
        rclpy.spin(minimal_service, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
