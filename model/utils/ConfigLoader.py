"""兼容既有编译模块使用的 JSON 配置加载接口。"""
import json
from typing import Dict


class ConfigLoader:
    @staticmethod
    def load(path: str) -> Dict:
        with open(path, 'r', encoding='utf-8-sig') as f:
            return json.load(f)
