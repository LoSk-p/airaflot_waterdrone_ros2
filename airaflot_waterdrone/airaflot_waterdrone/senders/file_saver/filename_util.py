from datetime import datetime
import typing as tp
from .config import MAX_MEASUREMENTS_COUNT, NEW_FILE_TIMEOUT, FILE_NAME_PREFIX, STORE_FOLDER_NAME, STORE_FILES_PATH

FileNameType = tp.TypeVar('FileNameType', bound='Filename')

class Filename:
    def __init__(self, current_datetime: datetime, folder_date: str) -> None:
        self._file_date: datetime = current_datetime
        self._folder_name: str = folder_date
        self._filename: str = self._create_filename()

    @classmethod
    def create_new(cls: tp.Type[FileNameType], folder_date: str) -> FileNameType:
        return cls(datetime.now(), folder_date)

    def to_str(self) -> str:
        return self._filename
    
    def get_date(self) -> datetime:
        return self._file_date

    @staticmethod
    def get_folders_to_create(current_date: str) -> tp.List[str]:
        folders = []
        folders.append(STORE_FILES_PATH)
        folders.append(f"{STORE_FILES_PATH}/{STORE_FOLDER_NAME}")
        folders.append(f"{STORE_FILES_PATH}/{STORE_FOLDER_NAME}/{current_date}")
        return folders
    
    def _create_filename(self) -> str:
        filename_suffix = str(self._file_date).split(" ")[1].split(".")[0].replace(":", "_")
        return f"{STORE_FILES_PATH}/{STORE_FOLDER_NAME}/{self._folder_name}/{FILE_NAME_PREFIX}{filename_suffix}.json"
