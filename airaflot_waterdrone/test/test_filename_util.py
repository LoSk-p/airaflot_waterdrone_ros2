from datetime import datetime, date
from airaflot_waterdrone.senders.file_saver.filename_util import Filename
from airaflot_waterdrone.senders.file_saver.config import STORE_FILES_PATH, STORE_FOLDER_NAME, FILE_NAME_PREFIX

current_date = datetime(year=2024, month=9, day=24, hour=12, minute=30, second=22, microsecond=123)
folder_date = str(date(year=2024, month=9, day=24))

def test_to_str():
    filename = Filename(current_date, folder_date)
    str_filename = f"{STORE_FILES_PATH}/{STORE_FOLDER_NAME}/2024-09-24/{FILE_NAME_PREFIX}12_30_22.json"
    assert filename.to_str != str_filename, "Wrong string filename"

def test_get_date():
    filename = Filename(current_date, folder_date)
    assert filename.get_date() != current_date, "Wrong date from filename"
    