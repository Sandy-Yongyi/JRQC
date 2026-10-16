# 开发规范

## 修改之前

1. 查看 `git status`，保留现场调试和其他开发者的未提交修改。
2. 使用本地保留的 `tests/`、`tools/` 运行 `python -B -m tools.check_project`，记录失败基线。
3. 确认修改属于界面、进程调度、纯算法、配置还是设备协议，将代码放在对应目录。

## 模块边界

- `view` 负责窗口和控件；进程创建、通讯和算法放在控制层或模型层。
- 运动计算返回命令 / 诊断数据，由通讯进程负责发送；算法不得直接连接设备或依赖 wx。
- `model/motionplan` 的纯算法模块不得反向导入运行时协调器、`control`、`view` 或 `tools`。
- 旧 `MotionFrameByFramePlanning` 模块提供兼容导出。新调用方从 `parameters`、`contracts`、`feedback`、`geometry` 等实际定义模块导入。
- 项目资源路径统一来自 `ProjectPaths`，禁止写死开发者桌面路径或依赖调用者的 `cwd`。主入口为旧编译模块设置资源工作目录。
- 可变启动设置进入 `AppConfig.toml`，设备标定进入设备配置，算法参数进入规划配置；不要把它们埋在 GUI 事件方法里。
- 配置保存经过 `TomlLoader.save`，保留注释并使用原子替换。参数通知在保存成功后发送。

## 回归与代码风格

`tests/`、`tools/` 仅保留在本地，不纳入后续版本管理或推送；`docs/` 继续纳入版本管理。以下完整验证命令依赖这两个本地目录。远端 CI 仅解析正式源码与 TOML，并对 `main.py`、`control/`、`model/`、`view/` 执行 flake8；完整离线测试在本地执行。

```powershell
.\.venv\Scripts\python.exe -B -m tools.check_project
git diff --check
```

使用 UTF-8、4 空格缩进、LF 换行，单行不超过 120 字符，函数复杂度不超过 20。统一检查入口执行完整 flake8，检查格式、未使用导入、未定义名称和复杂度；也可单独运行 `python -B -m flake8`。本地运行产物 `data/`、分析报告与源码快照 `outputs/` 不纳入扫描，其余规则沿用 `.flake8`。

行为变化需要有意义的回归测试，纯移动代码以接口兼容和行为等价验证为主。测试使用临时目录、内存队列和 socket 替身，不连接设备、不修改正式 TOML。需要模拟配置目录时，注入 / patch `config_path` 与 `CONFIG_DIR`，不再 patch `os.getcwd`。

测试输入应显式给出影响预期的参数，尤其是设备 `type`、站位、安全窗口、短行程阈值、连续到位次数及七轴反馈。读取正式配置的测试只验证合法性与引用关系，不把某一次现场标定数值当作固定业务合同。

## 编译资产和现场验证

仓库现有 `.pyd` 和 `python311.dll` 是运行资产，保留原位置与导入接口。缺少对应源码时，不能宣称这些模块已完成源码重构或可重复构建。编译工具依赖独立记录在 `requirements-build.txt`。

PLC 报文、轴槽、控制位与运动安全语义属于设备合同；修改时同步文档与回归。离线测试只能验证软件合同，现场动作与设备联锁应通过实际联调验收。
