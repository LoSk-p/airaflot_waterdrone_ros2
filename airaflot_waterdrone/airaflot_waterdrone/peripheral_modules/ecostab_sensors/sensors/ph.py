import struct
import pymodbus.client as ModbusClient

from .sensor import Sensor
from ....config import ECOSTAB_SENSORS_PORT

from airaflot_msgs.msg import EcostabSensors

PH_REGISTER = 0
TEMPERATURE_REGISTER = 2

class pHSensor(Sensor):
    def __init__(self) -> None:
        self.client = ModbusClient.ModbusSerialClient(
            ECOSTAB_SENSORS_PORT, baudrate=9600, bytesize=8, stopbits=1
        )

    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        ph_res = self.client.read_holding_registers(PH_REGISTER, 2)
        temperature_res = self.client.read_holding_registers(TEMPERATURE_REGISTER, 2)
        data.ph = self._unpack_float_registers(ph_res)
        data.temperature = self._unpack_float_registers(temperature_res)
        return data

    def _unpack_float_registers(self, registers: list):
        t = (registers[0], registers[1])
        packed_string = struct.pack("HH", *t)
        unpacked_float = struct.unpack("f", packed_string)[0]
        return unpacked_float
