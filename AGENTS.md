# 项目协作约定

先阅读 `README.md`、`CONTRIBUTING.md` 及相关 `docs/` 文档。

- 保留工作区已有修改及设备标定值。不要为通过测试随意调整正式 TOML。
- 使用 Windows x64 / CPython 3.11；优先运行 `.venv/Scripts/python.exe`。
- 本地标准验证：`python -B -m tools.check_project`（依赖本地 `tests/`、`tools/`）和 `git diff --check`。
- 所有测试默认离线，不启动主界面，不连接 PLC / 雷达，不修改正式配置。
- 新规划代码进入职责明确的 `model/motionplan` 子模块；旧运行时模块保留兼容导出。
- 路径使用 `ProjectPaths`；启动模式与回放文件使用 `AppConfig.toml`。
- 保留 `.pyd` 的文件位置、模块接口与旧配置名兼容映射；二进制模块没有源码，勿伪称已重构或重建。
- `tests/`、`tools/` 仅保留本地，不纳入后续版本管理或推送；`docs/` 继续纳入版本管理。运行日志、点云与报告进入被忽略的 `data/` 或 `outputs/`。
