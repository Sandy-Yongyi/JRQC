# JR 喷涂控制系统

面向 Windows 的 wxPython 桌面程序，读取激光点云，维护链条位置对应的帧队列，结合 PLC 实际轴位置计算喷涂与回位命令。

当前逐帧生产链路使用 `frame_by_frame` 策略。`complete_workpiece` 已在运行入口停用；其他策略的支持情况见 [架构说明](docs/architecture.md)。

## 环境与启动

使用 **Windows x64 / CPython 3.11 x64**。仓库中的 `cp311-win_amd64.pyd` 绑定这一平台与 Python ABI，不能直接换用 Python 3.12 或 Linux。

在项目根目录执行：

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -B main.py
```

已有 `.venv` 时可直接使用。需要镜像源时，在安装命令中显式添加 `--index-url`；仓库不再强制使用某个镜像。

启动设置在 [AppConfig.toml](model/tomls/AppConfig.toml)：

| `run_mode` | 行为 | 输入目录 |
| --- | --- | --- |
| `replay` | 从保存的点云向 PLC 通讯进程回放数据 | `data/points/` |
| `acquisition` | 使用雷达实时采集，向 PLC 输出控制帧 | 雷达配置 |
| `plot` | 仅查看保存的点云 | `data/temp/` |

`replay_file` 和 `plot_file` 填写文件后缀，例如 `20260923_133240.txt`，实际文件可带 `combined_`、`left_`、`right_` 等方向前缀。

**`replay` 仍会按 `PlcConfig.toml` 建立 PLC 连接。** 只查看算法结果时使用离线工具。当前工作区保留原有回放模式、本机 PLC 地址及设备标定参数；启动前按实际用途检查配置。

## 常用开发命令

以下完整检查、回归和查看器命令依赖本地保留的 `tests/`、`tools/` 目录；这两个目录不纳入后续版本管理或推送。

```powershell
# 开发依赖
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt

# 统一检查：源码 / TOML 解析、完整 flake8、完整离线测试
.\.venv\Scripts\python.exe -B -m tools.check_project --report outputs/project-check.json

# 仅解析源码与 TOML，不加载 GUI 或设备依赖
.\.venv\Scripts\python.exe -B -m tools.check_project --static-only

# 单个回归模块
.\.venv\Scripts\python.exe -B -m unittest tests.test_device_safety_windows -v

# 离线查看器的使用参数
.\.venv\Scripts\python.exe -B -m tools.frame_contour_viewer --help
.\.venv\Scripts\python.exe -B -m tools.motion_planning_viewer --help
```

完整离线测试不连接 PLC 或雷达，不启动主界面。测试的职责和隔离约定见本地 `tests/README.md`。远端 CI 仅解析正式源码与 TOML，并检查正式源码的 flake8；完整离线测试在本地执行。

## 目录职责

```text
main.py                 桌面启动入口与单实例生命周期
control/                GUI 控制、多进程调度、PLC 通讯
view/                   wxPython 窗口与对话框
model/
  motionplan/           配置合同、轮廓算法、几何约束、反馈状态机、运行协调
  utils/                配置读写、统一路径、日志、队列、心跳等支撑代码
  tomls/                应用、设备、雷达、PLC、喷涂及算法配置
  plc/                  PLC 数据结构与通讯编译模块
  dataprocess/          点云处理编译模块
  lidar/                雷达编译模块
  formats/              帧数据结构编译模块
  plot/                 点云显示编译模块
tests/                  本地离线合同与行为回归，不纳入版本管理或推送
tools/                  本地离线查看、队列回放、模拟联调与检查工具，不纳入版本管理或推送
docs/                   架构、配置、开发与重构记录，纳入版本管理
data/                   运行日志、点云和临时文件；图标资源保留在版本库
outputs/                本地报告与分析产物，不纳入版本管理
```

现有目录名和旧模块导入路径继续保留，以兼容编译模块。新增规划代码按职责放入对应子模块，不再集中追加到 `MotionFrameByFramePlanning.py`。

进一步阅读：[架构说明](docs/architecture.md) · [配置说明](docs/configuration.md) · [开发规范](CONTRIBUTING.md) · [本次重构与验证记录](docs/refactoring.md)
