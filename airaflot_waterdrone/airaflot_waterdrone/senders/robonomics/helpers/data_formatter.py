import typing as tp
import json

def read_formatted_data(filepath: str, public_key: str) -> tp.Dict:
    with open(filepath, "r") as f:
        data = json.load(f)
    res = {public_key: {"model": 2}}
    res["geo"] = f"{data["measurements"][0]['geo'][0]}, {data["measurements"][0]['geo'][1]}"
    res["measurements"] = data["measurements"]
    return res