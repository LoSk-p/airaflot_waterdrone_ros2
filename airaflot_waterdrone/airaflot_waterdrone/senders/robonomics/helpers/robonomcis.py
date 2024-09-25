from robonomicsinterface import Account, Datalog
from rclpy.impl.rcutils_logger import RcutilsLogger
from uuid import getnode as get_mac
import hashlib
import time

from ..config_private import ACCOUNT_SEED

class Robonomics:
    def __init__(self, logger: RcutilsLogger) -> None:
        self._account = Account(ACCOUNT_SEED)
        self._datalog = Datalog(self._account)
        self._logger = logger
        self._public_key = self._generate_public_key()
        logger.info(f"Robonomcis account {self._account.get_address()}")

    def record_datalog(self, data: str) -> None:
        try:
            self._logger.info(f"Start creating Datalog with data: {data}")
            res = self._datalog.record(data)
            self._logger.info(f"Datalog was created with hash {res}")
        except Exception as e:
            self._logger.error(f"Can't create datalog with error: {e}")

    def get_public_key(self) -> str:
        return self._public_key

    def _generate_public_key(self) -> str:
        mac = f"{get_mac()}_{time.time()}"
        verify_key = hashlib.sha256(mac.encode("utf-8"))
        verify_key_hex = str(verify_key.hexdigest())
        self._logger.info(f"New public key: {verify_key_hex}")
        return verify_key_hex
