from abc import ABC, abstractmethod

from airaflot_msgs.msg import EcostabSensors

class Sensor(ABC):
    @abstractmethod
    def fetch(self, data: EcostabSensors) -> EcostabSensors:
        pass 