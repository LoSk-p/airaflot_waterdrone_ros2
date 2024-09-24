from .sensor import Sensor

from airaflot_msgs.msg import EcostabSensors

PH_REGISTER = 0
TEMPERATURE_REGISTER = 2

class pHSensor(Sensor):
    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        ph_res = self.client.read_holding_registers(PH_REGISTER, 2)
        temperature_res = self.client.read_holding_registers(TEMPERATURE_REGISTER, 2)
        data.ph = self._unpack_float_registers(ph_res)
        data.temperature = self._unpack_float_registers(temperature_res)
        return data
