"""项目资源路径；不依赖调用者的工作目录。"""

from pathlib import Path
import sys


PROJECT_ROOT = (
    Path(sys.executable).resolve().parent
    if getattr(sys, "frozen", False)
    else Path(__file__).resolve().parents[2]
)
CONFIG_DIR = PROJECT_ROOT / "model" / "tomls"
DATA_DIR = PROJECT_ROOT / "data"
LOG_DIR = DATA_DIR / "logs"
POINTS_DIR = DATA_DIR / "points"
TEMP_DIR = DATA_DIR / "temp"


def config_path(filename: str) -> str:
    """为既有 Python / 编译模块返回字符串形式的配置路径。"""
    return str(CONFIG_DIR / filename)
