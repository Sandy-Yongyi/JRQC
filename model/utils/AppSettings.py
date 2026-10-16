"""应用启动设置，与喷涂策略和设备参数分开管理。"""

from dataclasses import dataclass
from pathlib import Path

from model.utils.ProjectPaths import config_path
from model.utils.TomlLoader import TomlLoader


RUN_MODE_CODES = {"replay": 1, "acquisition": 2, "plot": 3}


@dataclass(frozen=True)
class AppSettings:
    run_mode: str = "replay"
    replay_file: str = "20260923_133240.txt"
    plot_file: str = "20260915_151856.txt"

    def __post_init__(self) -> None:
        if self.run_mode not in RUN_MODE_CODES:
            raise ValueError("run_mode 必须是 replay、acquisition 或 plot")
        for name in ("replay_file", "plot_file"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} 必须是非空文件名")
            if Path(value).name != value or "/" in value or "\\" in value:
                raise ValueError(f"{name} 只接受文件名，目录由 ProjectPaths 管理")

    @property
    def control_type(self) -> int:
        return RUN_MODE_CODES[self.run_mode]

    @classmethod
    def load(cls, path: str | Path | None = None) -> "AppSettings":
        # 兼容尚未包含 AppConfig.toml 的既有部署，保留重构前的运行设置。
        settings_path = Path(path) if path is not None else Path(config_path("AppConfig.toml"))
        if not settings_path.exists():
            return cls()
        return cls(**TomlLoader.load(str(settings_path)))
