from .sensor import Sensor

from airaflot_msgs.msg import EcostabSensors

ORP_REGISTER = 0x9008

class ORPSensor(Sensor):
    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        orp_res = self.client.read_holding_registers(ORP_REGISTER, 2)
        data.orp = self._unpack_float_registers(orp_res)
        return data
