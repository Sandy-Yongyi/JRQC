# JRQC Frame Topology Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans
> to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 JRQC 按帧模式改为四设备、14 发送轴/16 接收轴拓扑，并接入四按钮、设备位和 X 轴清理模式。

**Architecture:** `MachineConfig1.toml` 是唯一的新运行拓扑配置；`MachineAxisMap` 负责 SN0/SN1 的普通轴位置和 Y/X 限位索引，二维设备通过两个独立 PLC 字段发送。按帧规划按设备类型分流，未定义的顶底自动算法采用保持当前位置的安全阶段行为。

**Tech Stack:** Python dataclass PLC frames、TOML、wxPython。

**Spec:** `docs/superpowers/specs/2026-08-19-jrqc-frame-topology.md`

## Global Constraints

- 保护当前未提交的 `MotionFrameByFramePlanning.py`、`MovingFrameData.py` 和 `MotionOutLiftPlanning.py` 修改。
- 不修改 `MachineConfig2.toml`、`MachineConfig3.toml` 和 `LidarConfig.toml`。
- 不猜测未提供的轴参数、运动参数和安全参数。
- 不实现本阶段延期的顶底或二维运动算法。
- 非按帧策略必须在连接 JRQC PLC 前失败，不能发送截断的旧 48 轴指令。
- 按项目适配 skill 的规则，不新增或运行单元测试；使用静态结构验证。

---

### Task 1: 按帧配置与模式

**Files:**
- Modify: `model/tomls/MachineConfig1.toml`
- Modify: `model/tomls/ModeConfig.toml`
- Modify: `model/utils/MachineConfigUtil.py`

**Interfaces:**
- Produces: SN0-SN3 的配置字典和 `strategy_name=1`。

- [x] 用四设备已确认配置完全替换 `MachineConfig1.toml`。
- [x] 将 `ModeConfig.toml` 的策略代码设为 1。
- [x] 禁止配置加载和 UI 保存过程给缺失偏移自动补 100。
- [x] 用 `tomllib` 解析两个 TOML，并断言 SN 集合为 `{0,1,2,3}`。

### Task 2: PLC 轴数量与映射

**Files:**
- Modify: `model/plc/MovingFrameData.py`
- Modify: `model/motionplan/MachineAxisMap.py`

**Interfaces:**
- Produces: `create_axis_list()` 返回 14 个发送轴；接收帧解码 16 个反馈轴；`get_axis_map("xn_updown", direction)` 返回固定索引。

- [x] 分离发送轴数量 14 与接收轴数量 16，更新协议注释。
- [x] 增加 `xn_updown` 的 Y/X 限位索引和左右轴映射。
- [x] 序列化默认发送帧并断言载荷长度为 110。
- [x] 断言左右轴映射分别覆盖 `0..6` 和 `7..13`。

### Task 3: 四设备参数按钮

**Files:**
- Modify: `view/MainFrame.py`
- Modify: `control/MainFrameControl.py`
- Modify: `view/MachineConfigFrame.py`

**Interfaces:**
- Produces: 四个按钮分别打开 SN0-SN3；`xn_updown` 只显示用户指定的参数键。

- [x] 将参数按钮布局改成左右两列、顶底和二维两行。
- [x] 更新按钮启停与控制器事件绑定。
- [x] 为 `xn_updown` 使用精确的 11 个界面参数键；二维暂保留现有按帧参数集合。
- [x] 静态搜索确认旧三个按钮变量不再被引用。

### Task 4: 按帧使能与清理分流

**Files:**
- Modify: `model/motionplan/MotionCleaningPlanning.py`
- Modify: `model/motionplan/MotionFrameByFramePlanning.py`

**Interfaces:**
- Consumes: SN0/SN1 普通轴映射；SN2/SN3 独立二维字段。
- Produces: bit1/bit2 只控制 SN0/SN1；bit15 清理时全部 X 目标为 X 最大限位；二维每周期独立处理。

- [x] 将清理位从 bit6 改为 bit15；X 轴去最大限位，其他轴去 `safe_pos`。
- [x] 按帧主循环跳过 `out_lift`，避免给 SN2/SN3 分配不存在的 bit3/bit4。
- [x] 清理模式优先于 SN0/SN1 自动规划，并在发送 Enable 中保留 bit15。
- [x] `xn_updown` 自动算法未提供时发送保持当前位置指令，不调用旧 `out_fx` 规划器。
- [x] 保留左右二维独立字段分流和每周期调用。

### Task 5: 静态验收

**Files:**
- Verify: all modified Python and TOML files

**Interfaces:**
- Produces: 可复核的语法、配置、协议和残留搜索结果。

- [x] 对全部修改的 Python 文件执行 `ast.parse`。
- [x] 解析所有 TOML 文件。
- [x] 验证发送帧 110 字节、接收 110 字节、发送轴 14、接收轴 16。
- [x] 搜索按帧路径中的 48 轴、旧 SN/按钮、bit6 清理残留。
- [x] 运行 `git diff --check` 并人工核对未修改 MachineConfig2/3 与 LidarConfig。
