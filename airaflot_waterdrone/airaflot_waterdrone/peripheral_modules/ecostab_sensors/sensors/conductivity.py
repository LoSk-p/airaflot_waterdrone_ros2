from .sensor import Sensor

from airaflot_msgs.msg import EcostabSensors

COND_REGISTER = 0x9006

class ConductivitySensor(Sensor):
    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        conductivity_res = self.client.read_holding_registers(COND_REGISTER, 2)
        data.conductivity = self._unpack_float_registers(conductivity_res)
        return data
