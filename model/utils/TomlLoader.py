import tomllib
import re
import logging
import tomlkit
from typing import Dict, Any


class TomlLoader:
    """TOML文件加载器，自动处理编码和格式问题"""

    @staticmethod
    def load(path: str, *, debug: bool = False) -> Dict[str, Any]:
        """
        加载 TOML 文件并自动解决常见问题
        :param path: 文件路径
        :param debug: 是否输出调试信息
        :return: 解析后的字典
        """
        with open(path, "rb") as f:
            raw_data = f.read()

            if debug:
                logging.info(f"toml head file: {raw_data[:16].hex(' ')}")

            # 移除BOM头
            cleaned_data = TomlLoader._remove_bom(raw_data)

            # 解码内容
            content = TomlLoader._safe_decode(cleaned_data)

            # 清理开头特殊字符
            content = re.sub(r"^[\x00-\x1F]+", "", content)

            # 解析TOML
            try:
                return tomllib.loads(content)
            except tomllib.TOMLDecodeError as e:
                if debug:
                    logging.error(f"TOML load error: {e}")
                    logging.error(f"file content is : {repr(content[:100])}")
                raise

    @staticmethod
    def save(data: Dict[str, Any], path: str) -> None:
        """
        将数据保存为TOML文件，保留原有格式和注释
        :param data: 要更新的数据字典（只包含需要修改的字段）
        :param path: 文件路径
        """
        # 首先读取现有文件内容
        with open(path, 'r', encoding='utf-8') as f:
            doc = tomlkit.parse(f.read())

        # data的结构应该是 {sn: {key: value}} 或直接 {key: value}
        if isinstance(data, dict):
            # 如果是嵌套结构
            if all(isinstance(v, dict) for v in data.values()):
                for key, value in data.items():
                    if key in doc:
                        # 更新嵌套字典中的字段
                        for subkey, subvalue in value.items():
                            doc[key][subkey] = subvalue  # type: ignore
                    else:
                        # 如果键不存在，添加到文档末尾
                        doc.add(key, value)
            else:
                # 如果是单层结构，直接更新
                for key, value in data.items():
                    doc[key] = value

        # 保存回文件
        with open(path, 'w', encoding='utf-8') as f:
            f.write(tomlkit.dumps(doc))

    @staticmethod
    def _remove_bom(data: bytes) -> bytes:
        """移除各种BOM头"""
        boms = {b"\xef\xbb\xbf": 3, b"\xff\xfe": 2, b"\xfe\xff": 2, b"\xff\xfe\x00\x00": 4, b"\x00\x00\xfe\xff": 4}
        # UTF-8  # UTF-16 LE  # UTF-16 BE  # UTF-32 LE  # UTF-32 BE

        for bom, length in boms.items():
            if data.startswith(bom):
                return data[len(bom):]
        return data

    @staticmethod
    def _safe_decode(data: bytes) -> str:
        """安全解码字节数据"""
        encodings = ["utf-8", "utf-16", "gbk", "latin-1", "cp1252"]

        for encoding in encodings:
            try:
                return data.decode(encoding)
            except UnicodeDecodeError:
                continue

        # 最终回退方案
        return data.decode("utf-8", errors="ignore")
