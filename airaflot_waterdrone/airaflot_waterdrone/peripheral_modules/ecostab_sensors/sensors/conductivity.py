import struct
import pymodbus.client as ModbusClient

from .sensor import Sensor
from ....config import ECOSTAB_SENSORS_PORT

from airaflot_msgs.msg import EcostabSensors

COND_REGISTER = 0x9006

class ConductivitySensor(Sensor):
    def __init__(self) -> None:
        self.client = ModbusClient.ModbusSerialClient(
            ECOSTAB_SENSORS_PORT, baudrate=9600, bytesize=8, stopbits=1
        )

    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        conductivity_res = self.client.read_holding_registers(COND_REGISTER, 2)
        data.conductivity = self._unpack_float_registers(conductivity_res)
        return data

    def _unpack_float_registers(self, registers: list):
        t = (registers[0], registers[1])
        packed_string = struct.pack("HH", *t)
        unpacked_float = struct.unpack("f", packed_string)[0]
        return unpacked_float