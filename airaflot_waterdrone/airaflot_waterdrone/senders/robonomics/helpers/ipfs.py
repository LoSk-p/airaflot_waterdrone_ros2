import pinatapy
import typing as tp
from rclpy.impl.rcutils_logger import RcutilsLogger

from ..config_private import PINATA_PRIVATE, PINATA_PUBLIC

class IPFS:
    @staticmethod
    def add_file(filepath: str, logger: RcutilsLogger) -> tp.Optional[str]:
        logger.info(f"Start adding {filepath} to Pinata")
        try:
            pinata = pinatapy.PinataPy(PINATA_PUBLIC, PINATA_PRIVATE)
            res = pinata.pin_file_to_ipfs(filepath)
        except Exception as e:
            logger.error(f"Can't add file {filepath} to Pinata with exception: {e}")
            return
        ipfs_hash = res.get("IpfsHash")
        if ipfs_hash is not None:
            logger.info(f"File {filepath} wass added to Pinata with hash {ipfs_hash}")
            return ipfs_hash
        else:
            logger.error(f"Can't add file {filepath} to Pinata with response: {res}")
        
    @staticmethod
    def add_json(json_data: tp.Dict, logger: RcutilsLogger) -> tp.Optional[str]:
        logger.info(f"Start adding json data to Pinata")
        try:
            pinata = pinatapy.PinataPy(PINATA_PUBLIC, PINATA_PRIVATE)
            res = pinata.pin_json_to_ipfs(json_data)
        except Exception as e:
            logger.error(f"Can't add json data to Pinata with exception: {e}")
            return
        ipfs_hash = res.get("IpfsHash")
        if ipfs_hash is not None:
            logger.info(f"Json data wass added to Pinata with hash {ipfs_hash}")
            return ipfs_hash
        else:
            logger.error(f"Can't add json data to Pinata with response: {res}")