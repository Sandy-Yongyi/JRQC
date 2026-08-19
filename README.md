# LiDAR 采数系统 — 参数配置使用文档

## 目录

1. [系统概述](#1-系统概述)
    - [1.1 V2.0 更新概览（相对 V1.0）](#11-v20-更新概览相对-v10)
2. [快速开始](#2-快速开始)
3. [采集策略说明](#3-采集策略说明)
4. [配置文件详解](#4-配置文件详解)
   - [SystemConfig.toml](#41-systemconfigtoml--系统总配置)
   - [LidarConfig.toml](#42-lidarconfigtoml--激光雷达配置)
   - [PlcConfig.toml](#43-plcconfigtoml--plc通信配置)
   - [ReadDataConfig.toml](#44-readdataconfigtoml--采数核心参数)
   - [ProcessConfig.toml](#45-processconfigtoml--数据处理参数)
   - [MachineConfig.toml](#46-machineconfigtoml--机器人轴配置)
   - [SprayConfig.toml](#47-sprayconfigtoml--喷涂参数)
5. [PLC 通信帧格式](#5-plc-通信帧格式)
   - [SendMovingFrameData（发送帧）](#51-sendmovingframedata发送帧)
   - [ReceiveMovingFrameData（接收帧）](#52-receivemovingframedata接收帧)
   - [帧封装协议](#53-帧封装协议)
6. [修改策略与参数指南](#6-修改策略与参数指南)
7. [进程架构说明](#7-进程架构说明)
8. [常见问题](#8-常见问题)

---

## 1. 系统概述

本系统是标准的三种类型激光采数系统，支持以下三种采集策略：

| 策略名称                      | 中文名称       | 典型应用场景                         |
|-------------------------------|----------------|--------------------------------------|
| `frame_by_frame`              | 逐帧采集发送   | 星沙、新疆大道等传送带单向采数项目   |
| `continuous_bidirectional`    | 往复连续采集   | 沙特等往返式扫描项目                 |
| `complete_workpiece`          | 完整工件采集   | 上海展会、河村、欧瑞等整工件项目     |

系统由三个主要进程组成：

```
┌─────────────────────┐   pulse_queue   ┌──────────────────────┐
│ PlcCommunicationProc│ ──────────────> │ LidarAcquisitionProc │
│  (PLC通信进程)      │                 │  (激光采集进程)      │
└─────────────────────┘                 └──────────┬───────────┘
                                                   │ raw_data_queue
                                                   ▼
                                        ┌──────────────────────┐
                                        │ DataProcessingProcess│
                                        │  (数据处理进程，     │
                                        │  仅complete_workpiece│
                                        │  策略使用)           │
                                        └──────────────────────┘
```

### 1.1 V2.0 更新概览（相对 V1.0）

本次 V2.0 主要围绕 **采数安全联锁**、**PLC 协议扩展**、**多方向激光支持** 与 **运行性能优化** 四个方向升级。

| 模块 | V1.0 | V2.0 |
|------|------|------|
| 采数安全 | 以基本采集流程为主 | 增加激光/PLC 断连检测、自动重连、停止态激光状态轮询、异常工件作废与联锁状态下发 |
| 首件处理 | 仅基础跳过逻辑 | 增加软件首启工件跳过、重连后首工件跳过、逐帧模式空帧放行保护 |
| 帧发送 | FIFO 顺序发送 | 增加 FIFO 跳变检测，输出 `repeat_count` 用于补齐缺失节拍，并同步携带 `lidar_status` |
| 激光拓扑 | 主要面向左/右侧 | 扩展支持 `left_upper`、`right_upper` 上侧激光，并新增上侧独立滤波参数与建帧规则 |
| PLC 协议 | 简单收发字段 | 升级为固定 `206` 字节载荷，新增 `ChainCountMM`、`Status`、`Operate` 与 `AxisList[32]` |
| 运动轴映射 | 轴索引逻辑分散 | 新增 `MachineAxisMap`，统一设备类型、安装方向、PLC 轴索引及限位读取 |
| 性能优化 | 逐帧模式热点较多 | 仅消费 `pulse_queue` 最新值、逐帧模式减少排序开销、建帧改为向量化计算、支持 Cython 编译 |

#### V2.0 重点新增能力

1. **激光安全状态联锁**
    - 停止状态下会周期性轮询激光状态，并通过 `raw_data_queue` 下发 `lidar_status`。
    - 采数过程中如果激光断开或返回异常状态，当前工件会被判定为无效，避免异常点云继续下游流转。
    - `frame_by_frame` 模式下，异常时会附带 `reset_queue=True`，用于通知下游清理当前缓存队列。

2. **首件与异常恢复保护**
    - 软件刚启动时，如果工件已经在扫描区中间，V2.0 会自动跳过这件不完整工件。
    - 激光或 PLC 断线重连后，系统会跳过恢复后的首件，降低恢复瞬间混入不完整数据的概率。

3. **多方向激光扩展**
    - 在原有 `left`、`right` 基础上，新增 `left_upper`、`right_upper` 配置能力。
    - 上侧激光在逐帧建帧时改为按 `X` 分层，使用 `x_threshold` 生成帧数据，适配顶部视角采数。

4. **PLC V2 协议升级**
    - 发送帧和接收帧都扩展为固定 `206` 字节有效载荷。
    - 除链条脉冲外，还可直接接收毫米计数、状态位、操作位和 32 组轴数据。
    - 发送时如果载荷不足，会自动补 `0x00` 到约定长度，便于和 PLC 固定帧协议对齐。

5. **运行性能与部署能力提升**
    - `LidarAcquisitionProcess` 在更新脉冲时只消费队列中的最新值，减少 PLC 高频数据造成的积压。
    - `frame_by_frame` 模式下不过早做整帧排序，降低热点计算。
    - 新增 `setup.py`，可通过 Cython 将核心模块编译为扩展模块，便于性能优化和部署封装。

---

## 2. 快速开始

### 修改采集策略

在 `control/MainFrameControl.py` 中，找到以下代码并修改 `strategy_name` 参数：

```python
lidar_acquisition = LidarAcquisitionProcess(
    pulse_queue=pulse_queue,
    raw_data_queue=raw_data_queue,
    viz_queue=viz_queue,
    lidar_config=lidar_config,
    config_dir=toml_path,
    strategy_name="frame_by_frame"   # ← 修改此处
)
```

可选值：

- `"frame_by_frame"` — 逐帧采集，推荐用于传送带系统
- `"continuous_bidirectional"` — 往复采集，适用于往返扫描线
- `"complete_workpiece"` — 完整工件，适用于流水线多工件系统

### 基本使用流程

```python
import multiprocessing
from control.LidarAcquisitionProcess import LidarAcquisitionProcess
from control.PlcCommunicationProcess import PlcCommunicationProcess
from model.utils.TomlLoader import TomlLoader
import os

toml_path = os.getcwd() + "\\model\\tomls"
sys_config = TomlLoader.load(f"{toml_path}\\SystemConfig.toml")

lidar_config = {
    "left":  sys_config.get("left_lidar_ids", []),
    "right": sys_config.get("right_lidar_ids", []),
    # 如需启用上侧激光，可显式追加：
    # "left_upper":  sys_config.get("left_upper_lidar_ids", []),
    # "right_upper": sys_config.get("right_upper_lidar_ids", []),
}

pulse_queue     = multiprocessing.Queue()
raw_data_queue  = multiprocessing.Queue()
viz_queue       = multiprocessing.Queue()

# 1. 启动激光采集进程
lidar_acquisition = LidarAcquisitionProcess(
    pulse_queue=pulse_queue,
    raw_data_queue=raw_data_queue,
    viz_queue=viz_queue,
    lidar_config=lidar_config,
    config_dir=toml_path,
    strategy_name="frame_by_frame",   # ← 选择策略
)
lidar_acquisition.daemon = True
lidar_acquisition.start()

# 2. 启动PLC通信进程（工厂方法自动匹配策略）
plc_handler = PlcCommunicationProcess.create(
    strategy_name=lidar_acquisition.strategy_name,
    raw_data_queue=raw_data_queue,
    pulse_queue=pulse_queue,
)
plc_handler.daemon = True
plc_handler.start()
```

---

## 3. 采集策略说明

### 3.1 `frame_by_frame` — 逐帧采集发送

- **适用场景**：传送带单向运动，需要实时逐帧向 PLC 发送点云数据
- **工作原理**：
  1. 监测链条正向运动（`moving_forward`）
  2. 每帧采集后通过 FIFO 编号累积并发送至 `raw_data_queue`
  3. 发送格式：`{"fifo": <int>, "left": AxisFrameData, "right": AxisFrameData}`
- **关键参数**：`max_fifo`、`fifo_unit_mm`、`y_threshold`、`combined_y_min`/`combined_y_max`

### 3.2 `continuous_bidirectional` — 往复连续采集

- **适用场景**：0→3000 或 3000→0 往返扫描，端点自动停止采数
- **工作原理**：
  1. 检测方向变化，整段采集
  2. 到达端点（`pulse >= max_swing_pulse` 或 `pulse <= 0`）强制结束
  3. 采集完整后一次性发送所有数据至 `raw_data_queue`
- **关键参数**：`swing_threshold`、`max_swing_pulse`、`max_scan_length`

### 3.3 `complete_workpiece` — 完整工件采集

- **适用场景**：流水线多工件，每工件完整采集后进行分区及分枪处理
- **工作原理**：
  1. 检测正向运动，按有效点云数量判断工件起止
  2. 通过链接停止逻辑（`linked_stop_distance_mm`）协调多方向
  3. 完整工件数据发送至 `raw_data_queue`，再由 `DataProcessingProcess` 处理
- **关键参数**：`start_skip_mm`、`linked_stop_distance_mm`、`max_scan_length`

---

## 4. 配置文件详解

所有配置文件位于 `model/tomls/` 目录。

---

### 4.1 `SystemConfig.toml` — 系统总配置

**功能**：定义激光雷达 ID 分组，控制哪些激光属于左侧/右侧。

```toml
left_lidar_ids  = ["1", "2"]   # 左侧激光雷达 ID 列表（对应 LidarConfig.toml 中的节号）
right_lidar_ids = ["3", "4"]   # 右侧激光雷达 ID 列表
left_upper_lidar_ids  = ["5"]   # 左上侧激光雷达 ID 列表（可选）
right_upper_lidar_ids = ["6"]   # 右上侧激光雷达 ID 列表（可选）
```

| 参数              | 类型         | 说明                                     |
|-------------------|--------------|------------------------------------------|
| `left_lidar_ids`  | `list[str]`  | 左侧激光雷达 ID 列表，可配置多个         |
| `right_lidar_ids` | `list[str]`  | 右侧激光雷达 ID 列表，可配置多个         |
| `left_upper_lidar_ids`  | `list[str]`  | 左上侧激光雷达 ID 列表，V2.0 新增       |
| `right_upper_lidar_ids` | `list[str]`  | 右上侧激光雷达 ID 列表，V2.0 新增       |

> **注意**：ID 必须与 `LidarConfig.toml` 中的节号一致，如 `"1"` 对应 `[1]` 节。

---

### 4.2 `LidarConfig.toml` — 激光雷达配置

**功能**：定义每台激光雷达的网络地址、坐标偏移、安装方向等参数。每个节对应一台激光雷达。

```toml
[1]
host                   = "192.168.1.201"  # 激光雷达 IP 地址
port                   = 2111             # 激光雷达 TCP 端口
same_origin_offset_x   = 0               # 统一原点 X 轴偏移量（mm）
same_origin_offset_y   = 3695            # 统一原点 Y 轴偏移量（mm）
diff_origin_offset_x   = 0               # 各自原点 X 轴偏移量（mm）
diff_origin_offset_y   = 3695            # 各自原点 Y 轴偏移量（mm）
type                   = "120"           # 激光雷达型号（如 "120" 表示120°扫描角）
install_orietation     = "left"          # 安装方向：'left' 或 'right'
start_angle_direction  = 1              # 起始角度方向（0=下,1=上,2=右,3=左）
start_angle            = 0              # 扫描起始角度（度）
stop_angle             = 180            # 扫描终止角度（度）
```

| 参数                    | 类型    | 说明                                                                          |
|-------------------------|---------|-------------------------------------------------------------------------------|
| `host`                  | `str`   | 激光雷达 IP 地址                                                              |
| `port`                  | `int`   | TCP 端口，通常为 2111                                                         |
| `same_origin_offset_x`  | `int`   | 统一原点模式下的 X 轴偏移（mm），用于多激光拼接                               |
| `same_origin_offset_y`  | `int`   | 统一原点模式下的 Y 轴偏移（mm）                                               |
| `diff_origin_offset_x`  | `int`   | 各自原点模式下的 X 轴偏移（mm）                                               |
| `diff_origin_offset_y`  | `int`   | 各自原点模式下的 Y 轴偏移（mm）                                               |
| `type`                  | `str`   | 激光雷达型号                                                                  |
| `install_orietation`    | `str`   | 安装方向：`"left"`、`"right"`、`"left_upper"` 或 `"right_upper"`          |
| `start_angle_direction` | `int`   | 激光 0° 角朝向：0=下方, 1=上方, 2=右边, 3=左边（面对激光正面，逆时针 0-180）|
| `start_angle`           | `int`   | 有效扫描起始角度（度）                                                        |
| `stop_angle`            | `int`   | 有效扫描终止角度（度）                                                        |

> **V2.0 说明**：当前工程已预留 6 台激光配置，其中 `[5]`、`[6]` 分别用于 `left_upper`、`right_upper` 上侧激光。

---

### 4.3 `PlcConfig.toml` — PLC 通信配置

**功能**：定义与 PLC 的通信参数，支持 TCP 和 UDP 两种方式。

```toml
[1]
tcp_ip          = "192.168.1.100"  # PLC TCP 连接 IP 地址
tcp_port        = 2005             # PLC TCP 连接端口
udp_ip          = "192.168.1.50"   # PLC UDP 连接 IP 地址
udp_port        = 6002             # PLC UDP 连接端口
connection_type = "tcp"            # 连接类型：'tcp' 或 'udp'
```

| 参数              | 类型   | 说明                                    |
|-------------------|--------|-----------------------------------------|
| `tcp_ip`          | `str`  | PLC TCP 连接目标 IP 地址                |
| `tcp_port`        | `int`  | PLC TCP 连接目标端口号                  |
| `udp_ip`          | `str`  | PLC UDP 绑定本机 IP 地址                |
| `udp_port`        | `int`  | PLC UDP 绑定本机端口号                  |
| `connection_type` | `str`  | 通信方式：`"tcp"` 或 `"udp"`            |

---

### 4.4 `ReadDataConfig.toml` — 采数核心参数

**功能**：系统最核心的配置文件，控制点云过滤、运动判断、FIFO 管理等所有采数行为。

#### 1. 点云过滤参数

```toml
combined_x_min = 900    # 整体直通滤波 X 轴最小值（mm）
combined_x_max = 4000   # 整体直通滤波 X 轴最大值（mm）
combined_y_min = 1300   # 整体直通滤波 Y 轴最小值（mm）
combined_y_max = 5600   # 整体直通滤波 Y 轴最大值（mm）
left_x_min     = 900    # 左侧直通滤波 X 轴最小值（mm）
left_x_max     = 2500   # 左侧直通滤波 X 轴最大值（mm）
left_y_min     = 1300   # 左侧直通滤波 Y 轴最小值（mm）
left_y_max     = 5600   # 左侧直通滤波 Y 轴最大值（mm）
right_x_min    = 900    # 右侧直通滤波 X 轴最小值（mm）
right_x_max    = 2500   # 右侧直通滤波 X 轴最大值（mm）
right_y_min    = 1300   # 右侧直通滤波 Y 轴最小值（mm）
right_y_max    = 5600   # 右侧直通滤波 Y 轴最大值（mm）
left_upper_x_min  = 0   # 左上侧直通滤波 X 轴最小值（mm）
left_upper_x_max  = 1600
left_upper_y_min  = 1300
left_upper_y_max  = 5600
right_upper_x_min = 0   # 右上侧直通滤波 X 轴最小值（mm）
right_upper_x_max = 1600
right_upper_y_min = 1300
right_upper_y_max = 5600
energy_percentage = 0.9
radius_threshold  = 100
radius_num        = 100
```

| 参数                | 类型    | 说明                                              |
|---------------------|---------|---------------------------------------------------|
| `combined_x_min/max`| `int`   | 统一原点模式下整体点云 X 轴有效范围（mm）         |
| `combined_y_min/max`| `int`   | 统一原点模式下整体点云 Y 轴有效范围（mm），**也作为逐帧发送的 Y 分层范围** |
| `left_x_min/max`    | `int`   | 左侧激光独立原点模式 X 轴有效范围（mm）           |
| `left_y_min/max`    | `int`   | 左侧激光独立原点模式 Y 轴有效范围（mm）           |
| `right_x_min/max`   | `int`   | 右侧激光独立原点模式 X 轴有效范围（mm）           |
| `right_y_min/max`   | `int`   | 右侧激光独立原点模式 Y 轴有效范围（mm）           |
| `left_upper_x/y_*`  | `int`   | 左上侧激光独立原点模式的有效范围，V2.0 新增       |
| `right_upper_x/y_*` | `int`   | 右上侧激光独立原点模式的有效范围，V2.0 新增       |
| `energy_percentage` | `float` | 点云能量百分比过滤阈值                            |
| `radius_threshold`  | `int`   | 有效点云半径阈值（mm）                            |
| `radius_num`        | `int`   | 半径范围内最小有效点数                            |

#### 2. 点云间隔参数

```toml
x_threshold = 10   # 上侧激光按 X 分层建帧阈值（mm）
y_threshold = 10   # 按 Y 间隔排序/建帧阈值（mm）
z_threshold = 10   # 按 Z 间隔排序阈值（mm）
```

| 参数          | 类型  | 说明                                                              |
|---------------|-------|-------------------------------------------------------------------|
| `x_threshold` | `int` | `left_upper` / `right_upper` 在逐帧模式下按 X 分层时使用的阈值    |
| `y_threshold` | `int` | Y 轴分层间隔（mm），**逐帧模式**中每层对应一个 `AxisData` 数据点 |
| `z_threshold` | `int` | Z 轴分段间隔（mm），用于 `continuous_bidirectional` 策略数据排序 |

#### 3. 数据传输参数

```toml
plc_send_max_retries = 1     # PLC 数据发送最大重试次数
max_fifo             = 29999 # PLC FIFO 最大值，超过后重置为 0
max_pulse            = 160000 # PLC 脉冲最大值，超过后重置为 0
```

| 参数                    | 类型  | 说明                                                        |
|-------------------------|-------|-------------------------------------------------------------|
| `plc_send_max_retries`  | `int` | PLC 发送失败最大重试次数                                    |
| `max_fifo`              | `int` | FIFO 计数器最大值（含），达到后回绕为 0，需与 PLC 端一致   |
| `max_pulse`             | `int` | 脉冲计数器最大值，用于归一化 Z 轴坐标                       |

#### 4. 扫描有效性参数

```toml
left_scan_points_threshold        = 10     # 左侧有效点云数量阈值
right_scan_points_threshold       = 10     # 右侧有效点云数量阈值
left_upper_scan_points_threshold  = 10     # 左上侧有效点云数量阈值
right_upper_scan_points_threshold = 10     # 右上侧有效点云数量阈值
max_scan_length                   = 10000  # 最大扫描长度（mm），超过自动结束
swing_threshold                   = 3      # 往复模式开始/结束扫描阈值（mm）
max_swing_pulse                   = 3000   # 往复模式最大扫描范围（脉冲数）
linked_stop_distance_mm           = 100    # 两方向扫描联动停止距离差值（mm）
start_skip_mm                     = 20     # 软件启动后前 N mm 内数据跳过处理（mm）
```

| 参数                           | 类型  | 说明                                                                              |
|--------------------------------|-------|-----------------------------------------------------------------------------------|
| `left_scan_points_threshold`   | `int` | 左侧激光每帧最少有效点数，少于此值不认为工件在扫描范围内                          |
| `right_scan_points_threshold`  | `int` | 右侧激光每帧最少有效点数                                                          |
| `left_upper_scan_points_threshold`  | `int` | 左上侧激光每帧最少有效点数，V2.0 新增                                         |
| `right_upper_scan_points_threshold` | `int` | 右上侧激光每帧最少有效点数，V2.0 新增                                         |
| `max_scan_length`              | `int` | 单次工件最大采集长度（mm），超过后强制结束当前工件                                |
| `swing_threshold`              | `int` | 往复模式下端点判断阈值（mm），在端点附近此范围内停止采集                          |
| `max_swing_pulse`              | `int` | 往复模式下有效脉冲范围上限（脉冲数）                                              |
| `linked_stop_distance_mm`      | `int` | 完整工件模式下，当一个方向停止后，另一方向在此距离内联动停止（mm）                |
| `start_skip_mm`                | `int` | 软件启动后首次检测到运动时，前 N 毫米内的工件数据跳过处理，防止不完整工件入队     |

#### 5. 运动控制参数

```toml
pulse_to_mm      = 5.3  # 当前工程链条脉冲转毫米系数（示例）
fifo_unit_mm     = 2    # FIFO 每递增 1 对应的链条移动距离（mm）
diff_start_pulse = 1    # 链条启动判断脉冲差值阈值
```

| 参数              | 类型    | 说明                                                                      |
|-------------------|---------|---------------------------------------------------------------------------|
| `pulse_to_mm`     | `float` | 脉冲数转毫米的换算系数（不同产线不同）：六楼实验室链条=12.4，信高=7.0    |
| `fifo_unit_mm`    | `int`   | FIFO 计数器每增加 1 所对应的链条移动距离（mm），默认 2mm/FIFO             |
| `diff_start_pulse`| `int`   | 连续 5 帧脉冲最大最小差值超过此值才判定为链条运动（防抖阈值）             |

#### 6. 连接可靠性参数

```toml
lidar_connect_max_retries = 10000  # 激光雷达最大重连次数
plc_connect_max_retries   = 10000  # PLC 最大重连次数
```

| 参数                        | 类型  | 说明                                      |
|-----------------------------|-------|-------------------------------------------|
| `lidar_connect_max_retries` | `int` | 激光雷达连接失败后的最大重连尝试次数       |
| `plc_connect_max_retries`   | `int` | PLC 连接失败后的最大重连尝试次数           |

#### 7. 调试与可视化参数

```toml
translate_data_origin = 2  # 1=统一原点（同一坐标系），2=各自原点
```

| 参数                    | 类型  | 说明                                                                                      |
|-------------------------|-------|-------------------------------------------------------------------------------------------|
| `translate_data_origin` | `int` | `1`：所有激光数据折算到统一坐标系原点；`2`：各激光使用各自坐标系，由各激光自身原点出发   |

#### 8. 设备堆栈与线程参数

```toml
queue_size      = 1500  # 工件数据队列长度
threadpool_size = 3     # 数据保存线程池大小
```

| 参数              | 类型  | 说明                                    |
|-------------------|-------|-----------------------------------------|
| `queue_size`      | `int` | 工件数据队列堆栈数量                    |
| `threadpool_size` | `int` | 异步数据保存线程池大小                  |

---

### 4.5 `ProcessConfig.toml` — 数据处理参数

**功能**：`complete_workpiece` 策略专用，控制工件分区、分枪等处理算法。

```toml
y_threshold            = 10   # 按 Y 间隔排序阈值（mm）
z_threshold            = 10   # 按 Z 间隔排序阈值（mm）
jig_threshold          = 100  # 挂具距离最上平面 Y 的起始距离（mm）
x_range                = 300  # 判断为柜体的 X 轴范围（mm），大于此值判定为柜体
y_range                = 300  # 判断为柜体的 Y 轴范围（mm）
z_range                = 300  # 判断为柜体的 Z 轴范围（mm）
find_boundary_type     = 2    # 找分区方式（1=固定6分区，2=随机分区）
y_partition_threshold  = 80   # 合并分区 Y 轴阈值（mm），小于此值不认为是独立分区
z_partition_threshold  = 200  # 合并分区 Z 轴阈值（mm）
send_data_type         = 2    # 发送数据方式（1=分帧发送，2=整方块数据发送）
find_y_plane           = 0.5  # 查找最上平面 Y 所占百分比
```

| 参数                    | 类型    | 说明                                            |
|-------------------------|---------|-------------------------------------------------|
| `jig_threshold`         | `int`   | 挂具检测起始距离（mm）                          |
| `x_range`               | `int`   | 柜体 X 方向尺寸判定阈值（mm）                   |
| `y_range`               | `int`   | 柜体 Y 方向尺寸判定阈值（mm）                   |
| `z_range`               | `int`   | 柜体 Z 方向尺寸判定阈值（mm）                   |
| `find_boundary_type`    | `int`   | `1`=固定6分区算法，`2`=随机分区算法             |
| `y_partition_threshold` | `int`   | Y 方向合并分区最小距离阈值（mm）                |
| `z_partition_threshold` | `int`   | Z 方向合并分区最小距离阈值（mm）                |
| `send_data_type`        | `int`   | `1`=分帧发送模式，`2`=整块数据发送模式          |
| `find_y_plane`          | `float` | 最上平面检测的 Y 轴百分比参数                   |

---

### 4.6 `MachineConfig.toml` — 机器人轴配置

**功能**：定义喷涂机器人各轴的参数配置，每个节对应一把喷枪/轴组。

```toml
[0]
sn                 = 0           # 序号
type               = 1           # 类型（1=2轴，2=4轴）
offset_x           = 625         # X 轴偏移量（mm）
offset_z           = 4440        # Z 轴偏移量（mm）
axis_num           = 2           # 轴数量
z_offset           = 120         # Z 轴运动偏移（mm）
y_offset           = 100         # Y 轴运动偏移（mm）
y_speed            = 300         # Y 轴运动速度（mm/s）
x_offset           = 100         # X 轴运动偏移（mm）
x_speed            = 300         # X 轴运动速度（mm/s）
total_cycles       = 2           # 喷涂总循环次数
install_orietation = "left"      # 安装方向
limit_pos          = [1200, 1450, 1000, 180]  # 各轴限位位置
axis_enable        = [1, 1]      # 各轴使能状态（1=使能，0=禁用）
x_origin_pos       = [1560, 2010] # 各 X 轴原点位置
```

---

### 4.7 `SprayConfig.toml` — 喷涂参数

**功能**：定义喷枪运动参数、速度限制和尺寸范围。

```toml
gun_distance          = 450   # 每把枪 Y 轴间距，用于分枪（mm）
x_pre_distance        = 30    # X 轴提前定位安全距离（mm）
spray_width_distance  = 80    # 喷枪宽度，用于分区计算（mm）
spray_pos_tolerance   = 10    # 喷涂伺服定位公差（mm）
rect_threshold        = 150   # 喷涂分段间隔阈值（mm），大于 150 则分 6 段
x_offset_max          = 300   # X 轴偏移最大值（mm）
y_offset_max          = 300   # Y 轴偏移最大值（mm）
z_offset_max          = 300   # Z 轴偏移最大值（mm）
x_offset_min          = 50    # X 轴偏移最小值（mm）
y_offset_min          = 50    # Y 轴偏移最小值（mm）
z_offset_min          = 50    # Z 轴偏移最小值（mm）
lidar_scan_speed_max  = 50    # 激光采数最大速度
x_speed_max           = 620   # X 轴最大运动速度（mm/s）
y_speed_max           = 590   # Y 轴最大运动速度（mm/s）
z_speed_max           = 150   # Z 轴最大运动速度（mm/s）
r_speed_max           = 150   # R 轴最大运动速度
total_cycles_max      = 20    # 喷涂次数最大值
lidar_scan_speed      = 25    # 激光采数速度
max_size_y_max        = 4000  # Y 轴最大尺寸上限（mm）
size_y_max            = 2500  # Y 轴尺寸最大值（mm）
min_size_y_min        = 500   # Y 轴最小尺寸下限（mm）
size_y_min            = 1500  # Y 轴尺寸最小值（mm）
```

---

## 5. PLC 通信帧格式

系统通过 TCP/UDP 与 PLC 交换固定格式的二进制帧。V2.0 的帧定义位于 `model/plc/MovingFrameData.py`，字段类型定义在 `model/plc/PlcFrame.py`。

### 5.1 `SendMovingFrameData`（发送帧）

系统向 PLC 发送的控制数据帧，**有效载荷固定为 206 字节**（不含帧头/帧尾）。不足长度时会自动补 `0x00`。

| 字段名       | 类型    | 字节数 | 偏移量 | 说明                           |
|--------------|---------|--------|--------|--------------------------------|
| `Enable`     | `Int32` | 4      | 0      | 使能位/设备运动位/心跳位        |
| `Gun_Cont1`  | `Int32` | 4      | 4      | 喷枪控制字 1                   |
| `Gun_Cont2`  | `Int32` | 4      | 8      | 喷枪控制字 2                   |
| `Operate`    | `Int16` | 2      | 12     | 操作指令字                     |
| `AxisList`   | `32 × AxisData` | 192 | 14 | 32 组轴控制数据，每组含 `Pos`/`Speed`/`Status` |

```python
from model.plc.MovingFrameData import SendMovingFrameData, AxisData

# 创建发送帧
frame = SendMovingFrameData(Enable=1, Gun_Cont1=0, Gun_Cont2=0, Operate=1)
frame.AxisList[0] = AxisData(Pos=100, Speed=200, Status=1)

# 序列化（原始载荷长度为 206 字节）
raw = frame.to_bytes()
assert len(raw) == SendMovingFrameData.FRAME_SIZE    # 206

# 通过 PlcCommon.send_frame() 发送
plc.send_frame(frame)
```

**V2.0 说明**：`AxisList` 使用 `Repeat(32)` 定义 32 组 `AxisData`，每组轴数据结构如下：

```python
@dataclass
class AxisData(PlcFrame):
    Pos:    Annotated[int, Int16()] = 0
    Speed:  Annotated[int, Int16()] = 0
    Status: Annotated[int, Int16()] = 0
```

### 5.2 `ReceiveMovingFrameData`（接收帧）

系统从 PLC 接收的状态数据帧，**有效载荷固定为 206 字节**（不含帧头/帧尾）。

| 字段名        | 类型    | 字节数 | 偏移量 | 说明                          |
|---------------|---------|--------|--------|-------------------------------|
| `ChainPulse`  | `Int32` | 4      | 0      | 链条当前脉冲数                |
| `ChainSpeed`  | `Int32` | 4      | 4      | 链条当前速度                  |
| `ChainCountMM`| `Int16` | 2      | 8      | 链条毫米计数                  |
| `Status`      | `Int16` | 2      | 10     | 公共状态位                    |
| `Operate`     | `Int16` | 2      | 12     | 远程操作/心跳位               |
| `AxisList`    | `32 × AxisData` | 192 | 14 | 32 组轴状态数据               |

```python
from model.plc.MovingFrameData import ReceiveMovingFrameData

# 反序列化
frame, _ = ReceiveMovingFrameData.from_bytes(payload)
print(frame.ChainPulse, frame.ChainCountMM, len(frame.AxisList))

# 通过 PlcCommon.scan() 自动接收并解析
data = plc.scan(connection_type="tcp")
pulse = data.ChainPulse   # 链条脉冲
count_mm = data.ChainCountMM  # 毫米计数
axis0 = data.AxisList[0]      # 第 1 组轴数据
```

**修改收发帧字段**：同样在 `MovingFrameData.py` 中修改，并同步检查 `FRAME_SIZE` 与 PLC 端协议是否一致。

```python
@dataclass
class ReceiveMovingFrameData(PlcFrame):
    BYTE_ORDER = '>'
    FRAME_SIZE = 206
    ChainPulse:   Annotated[int, Int32()] = 0
    ChainSpeed:   Annotated[int, Int32()] = 0
    ChainCountMM: Annotated[int, Int16()] = 0
    Status:       Annotated[int, Int16()] = 0
    Operate:      Annotated[int, Int16()] = 0
```

### 5.3 帧封装协议

所有 PLC 通信帧使用固定帧头/帧尾封装：

```
┌─────────┬──────────────┬─────────┐
│  START  │   PAYLOAD    │   END   │
│ 4 bytes │   N bytes    │ 4 bytes │
│02 02 02 02│  frame data │03 03 03 03│
└─────────┴──────────────┴─────────┘
```

| 字段      | 值                          | 字节数 |
|-----------|-----------------------------|--------|
| `START`   | `0x02 0x02 0x02 0x02`       | 4      |
| `PAYLOAD` | 帧序列化数据                | N      |
| `END`     | `0x03 0x03 0x03 0x03`       | 4      |

- V2.0 发送总包大小：`4 + 206 + 4 = 214 字节`
- V2.0 接收总包大小：`4 + 206 + 4 = 214 字节`

**PLC 读命令**（系统每次读数据时先发送此固定请求包）：

```
0x02 0x02 0x02 0x02 0x01 0x03 0x03 0x03 0x03
```

### 5.4 字段类型参考

`PlcFrame` 支持的二进制类型（定义在 `model/plc/PlcFrame.py`）：

| Python 类型注解                     | 二进制格式    | 字节数 | 说明              |
|-------------------------------------|---------------|--------|-------------------|
| `Annotated[int, Int16()]`           | 有符号 16-bit | 2      | 范围 ±32767       |
| `Annotated[int, UInt16()]`          | 无符号 16-bit | 2      | 范围 0~65535      |
| `Annotated[int, Int32()]`           | 有符号 32-bit | 4      | 范围 ±2147483647  |
| `Annotated[int, UInt32()]`          | 无符号 32-bit | 4      | 范围 0~4294967295 |
| `Annotated[float, Float32()]`       | 32-bit 浮点   | 4      | IEEE 754          |
| `Annotated[str, FixedStr(N)]`       | 定长字符串    | N      | UTF-8, `\x00` 填充|

字节序默认为**大端序（Big-Endian）**，可通过类属性修改：

```python
BYTE_ORDER = '>'  # 大端序（默认）
BYTE_ORDER = '<'  # 小端序
```

---

## 6. 修改策略与参数指南

### 6.1 切换到 `frame_by_frame`（逐帧传送带）

1. 修改 `MainFrameControl.py`：`strategy_name="frame_by_frame"`
2. 配置 `ReadDataConfig.toml`：
   - 调整 `combined_y_min`、`combined_y_max` 为实际工件 Y 轴范围
   - 设置 `y_threshold`（Y 分层粒度，影响发送数据密度）
   - 设置 `pulse_to_mm`（当前产线脉冲换算系数）
   - 设置 `fifo_unit_mm`（与 PLC FIFO 增量一致）
   - 设置 `max_fifo`（与 PLC FIFO 最大值一致）

### 6.2 切换到 `continuous_bidirectional`（往返扫描）

1. 修改 `MainFrameControl.py`：`strategy_name="continuous_bidirectional"`
2. 配置 `ReadDataConfig.toml`：
   - 设置 `max_swing_pulse`（往返扫描脉冲范围）
   - 设置 `swing_threshold`（端点检测阈值）
   - 设置 `max_scan_length`（最大单次扫描长度）

### 6.3 切换到 `complete_workpiece`（完整工件）

1. 修改 `MainFrameControl.py`：`strategy_name="complete_workpiece"`
2. 系统自动启动 `DataProcessingProcess`（无需手动配置）
3. 配置 `ReadDataConfig.toml`：
   - 设置 `start_skip_mm`（启动跳过距离）
   - 设置 `linked_stop_distance_mm`（多方向联动停止距离）
4. 配置 `ProcessConfig.toml`：
   - 设置 `find_boundary_type`（分区算法类型）
   - 设置 `send_data_type`（发送数据格式）

---

## 7. 进程架构说明

### 进程通信队列

| 队列名               | 生产者                  | 消费者                  | 数据格式                                      |
|----------------------|-------------------------|-------------------------|-----------------------------------------------|
| `pulse_queue`        | `PlcCommunicationProcess`| `LidarAcquisitionProcess`| `{"pulse": int, "fifo": int, "status": str}` |
| `raw_data_queue`     | `LidarAcquisitionProcess`| `PlcCommunicationProcess`或下游| 依策略不同（见下文）                    |
| `viz_queue`          | `LidarAcquisitionProcess`| 主进程可视化             | `{"points": ndarray, "boxes": ...}`           |
| `machine_data_queue` | `DataProcessingProcess`  | `PlcCommunicationProcess`| 处理后的机器人运动数据                        |

### `raw_data_queue` 数据格式（按策略）

**`frame_by_frame`**：

```python
{
        "fifo":         int,           # FIFO 编号
        "repeat_count": int,           # FIFO 跳变时的补发次数，V2.0 新增
        "lidar_status": int,           # 激光状态码，V2.0 新增
        "left":         AxisFrameData, # 左侧点云帧数据
        "right":        AxisFrameData, # 右侧点云帧数据
        # 可按配置扩展：
        # "left_upper":  AxisFrameData,
        # "right_upper": AxisFrameData,
}
```

其中 `AxisFrameData.FrameData` 为 `List[AxisData]`，每个 `AxisData`：
- 对 `left` / `right`：
    - `H_Axis`：Y 轴坐标层（mm）
    - `V_Axis_Max`：该 Y 层内 X 最大值（mm）
    - `V_Axis_Min`：该 Y 层内 X 最小值（mm）
- 对 `left_upper` / `right_upper`：
    - `H_Axis`：X 轴坐标层（mm）
    - `V_Axis_Max`：该 X 层内 Y 最大值（mm）
    - `V_Axis_Min`：该 X 层内 Y 最小值（mm）

**`continuous_bidirectional`**：

```python
{
    "left":  {"data": ndarray},   # 左侧完整点云 (N×3)
    "right": {"data": ndarray},   # 右侧完整点云 (N×3)
}
```

**`complete_workpiece`**：

```python
{
    "lidar_status": int,  # 激光状态码
    # 以下字段按当前启用方向动态生成：
    "left_stop_pulse":  int,
    "left_data":        ndarray,
    "right_stop_pulse": int,
    "right_data":       ndarray,
}
```

> **V2.0 补充**：当仅用于状态联锁时，`raw_data_queue` 还可能收到只包含 `lidar_status` 与 `reset_queue` 的状态消息。

### PLC 通信进程（工厂方法）

```python
# 自动根据策略选择正确的队列
plc_handler = PlcCommunicationProcess.create(
    strategy_name="frame_by_frame",  # 或 "continuous_bidirectional" / "complete_workpiece"
    raw_data_queue=raw_data_queue,
    pulse_queue=pulse_queue,
    machine_data_queue=machine_data_queue,  # 仅 complete_workpiece 需要
)
```

---

## 8. 常见问题

### Q1: 如何调整点云有效范围？

修改 `ReadDataConfig.toml` 中的滤波参数：

```toml
# 统一原点模式（translate_data_origin = 1）
combined_x_min = 1150   # 根据实际安装位置调整
combined_x_max = 2500
combined_y_min = 1100
combined_y_max = 3800
```

### Q2: 链条脉冲系数如何确定？

`pulse_to_mm` 是脉冲数到毫米的换算系数。通过以下方式标定：
1. 让链条移动已知距离 D（mm）
2. 记录脉冲变化量 P
3. `pulse_to_mm = P / D`（单位：pulse/mm）

> 示例：六楼实验室链条 `pulse_to_mm = 12.4`（即每移动 1mm，脉冲增加 12.4）

### Q3: FIFO 回绕处理

系统自动处理 FIFO 从 `max_fifo`（默认 29999）回绕到 0 的情况。确保 `max_fifo` 与 PLC 端配置一致。

### Q4: 激光雷达连接失败如何处理？

系统内置自动重连机制，重连次数由 `lidar_connect_max_retries` 控制（默认 10000 次）。
每次重连间隔 10 秒。如果需要快速失败，可将该值设为较小数（如 3）。

### Q5: 如何修改 PLC 通信数据格式？

1. 编辑 `model/plc/MovingFrameData.py`
2. 新增/删除/修改字段（使用 `Annotated[type, WireType()]` 格式）
3. **同步检查 `FRAME_SIZE` 常量**（固定帧协议下需与 PLC 端一致）
4. 确认 PLC 端字段顺序与本文件一致
5. 检查字节序设置（`BYTE_ORDER = '>'` 大端 / `'<'` 小端）

### Q6: 模拟模式如何开启/关闭？

在 `control/PlcCommunicationProcess.py` 中设置：

```python
self.simulation_mode = True   # True=模拟模式（不连接PLC），False=实际连接PLC
```
