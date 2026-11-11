# JRQC 按帧设备拓扑阶段规格

## 本阶段目标

将 `MachineConfig1.toml` 对应的按帧运行模式切换到 JRQC 的四设备、16 轴 PLC 拓扑，同时保留 `MachineConfig2.toml`、`MachineConfig3.toml` 和旧设备规划代码。

## 设备矩阵

| SN | 类型 | 方向 | 发送映射 | 接收映射 |
|---:|---|---|---|---|
| 0 | `xn_updown` | `left` | `AxisList[0:7]`，轴序 `x1,y1,x2,y2,x3,y3,x4` | 同索引 |
| 1 | `xn_updown` | `right` | `AxisList[7:14]`，轴序 `x1,y1,x2,y2,x3,y3,x4` | 同索引 |
| 2 | `out_lift` | `left` | `Left2DLiftData` | `AxisList[14]` |
| 3 | `out_lift` | `right` | `Right2DLiftData` | `AxisList[15]` |

发送普通轴数为 14，接收总轴数为 16，发送和接收帧长度均为 110 字节。二维字段每周期发送，不受 SN0/SN1 设备运动使能影响。

## 已确认配置

- SN0/SN1：`origin_pos=[1020,1320,1620,3000]`；最后一项是固定喷枪4的 Y 位置，不存在 Y4 物理轴。
- SN0/SN1：限位数组顺序为 Y、X；`min_limit_pos=[0,0]`、`max_limit_pos=[1000,900]`、`max_limit_speed=[500,500]`、`safe_pos=[0,0]`。
- SN2/SN3：使用用户给出的 `out_lift` 配置；未使用的 `min_limit_pos`、`max_limit_pos`、`origin_pos`、`safe_pos` 均为 `[0]`。
- 目标策略是 `frame_by_frame`，即 `ModeConfig.strategy_name=1`。
- 沿用当前 `LidarConfig.toml`，本阶段不修改雷达配置。
- `MachineConfig2.toml`、`MachineConfig3.toml` 和旧规划代码只作能力保留；它们的 48 轴协议与 JRQC 110 字节报文不兼容，启动时必须在 PLC 连接前拒绝运行。

## 控制位

- bit0 (`0x0001`)：总使能。
- bit1 (`0x0002`)：SN0 左侧顶底设备。
- bit2 (`0x0004`)：SN1 右侧顶底设备。
- bit15 (`0x8000`)：清理模式。扫描全部按帧方向队列；任一 H/V 坐标非零时阻止清理并让设备前往 `safe_pos`，全部为零时 SN0/SN1 的 X 轴前往最大限位、其他轴前往 `safe_pos`。清理位关闭沿触发设备回安全位。
- SN2/SN3 不使用独立运动使能位。

## 界面

参数区为两行两列：第一列是左侧顶底设备和左侧二维设备，第二列是右侧顶底设备和右侧二维设备，按钮分别打开 SN0、SN2、SN1、SN3 的配置。

## 明确延期

- `xn_updown` 的点云搜索、定位、往复和完成状态算法等待后续精确规则；本阶段启用时保持当前轴位置。
- `auto_out_lift_machine_move` 的具体运动逻辑等待后续修改。
- 未提供数值的 SN0/SN1 界面运行参数不写入猜测值。
