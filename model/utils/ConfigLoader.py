# utils/config_loader.py
import json
from typing import Dict


class ConfigLoader:
    @staticmethod
    def load(path: str) -> Dict:
        with open(path, 'r') as f:
            return json.load(f)
