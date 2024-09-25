from .sensor import Sensor

from airaflot_msgs.msg import EcostabSensors

OXXYGEN_REGISTER = 0

class OxxygenSensor(Sensor):
    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        oxxygen_res = self.client.read_holding_registers(OXXYGEN_REGISTER, 2)
        data.oxxygen = self._unpack_float_registers(oxxygen_res)
        return data
