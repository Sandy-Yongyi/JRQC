import tomllib
import re
import logging
import codecs
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
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
        # 已发布的采集 .pyd 仍请求旧名称；统一读取同目录的采数配置。
        # 不再读取或要求存在 ProcessConfig.toml，即使旧文件仍在也不使用。
        if Path(path).name.casefold() == "processconfig.toml":
            path = str(Path(path).with_name("ReadDataConfig.toml"))
        with open(path, "rb") as f:
            raw_data = f.read()

            if debug:
                logging.info(f"toml head file: {raw_data[:16].hex(' ')}")

            content = TomlLoader._decode_bytes(raw_data)

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
        target = Path(path)
        doc = tomlkit.parse(TomlLoader._decode_bytes(target.read_bytes()))

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

        # 在同一目录完成原子替换，写入失败时保留原文件。
        temporary_path = None
        try:
            with NamedTemporaryFile(
                mode="w", encoding="utf-8", newline="", dir=target.parent,
                prefix=f".{target.name}.", suffix=".tmp", delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                temporary.write(tomlkit.dumps(doc))
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_path, target)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    @staticmethod
    def _decode_bytes(data: bytes) -> str:
        """先识别完整 BOM，避免 UTF-32 被误判为 UTF-16 或 UTF-8。"""
        for bom, encoding in (
            (codecs.BOM_UTF32_LE, "utf-32"),
            (codecs.BOM_UTF32_BE, "utf-32"),
            (codecs.BOM_UTF16_LE, "utf-16"),
            (codecs.BOM_UTF16_BE, "utf-16"),
            (codecs.BOM_UTF8, "utf-8-sig"),
        ):
            if data.startswith(bom):
                return data.decode(encoding)
        return TomlLoader._safe_decode(data)

    @staticmethod
    def _remove_bom(data: bytes) -> bytes:
        """移除各种BOM头"""
        boms = {b"\xff\xfe\x00\x00": 4, b"\x00\x00\xfe\xff": 4, b"\xef\xbb\xbf": 3, b"\xff\xfe": 2, b"\xfe\xff": 2}
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
