"""Windows 桌面程序入口；导入本模块不会初始化 GUI 或连接设备。"""

import ctypes
import multiprocessing
import os

from model.utils.ProjectPaths import PROJECT_ROOT
from model.utils.WindowsRuntime import (
    disable_console_interaction,
    is_already_running,
    release_instance_lock,
)


def main() -> int:
    # 既有 .pyd 内仍可能使用相对路径，启动时为其固定资源根目录。
    os.chdir(PROJECT_ROOT)
    from model.utils.LoggerUtil import logger

    try:
        if is_already_running():
            ctypes.windll.user32.MessageBoxW(
                0, "程序已在运行中，请不要启动多个实例", "错误提示", 0x10
            )
            logger.error("检测到已有实例运行，退出程序")
            return 1

        import wx
        from control.MainFrameControl import MainApp

        logger.info("main process ID: %s", os.getpid())
        app = MainApp()
        app.TopWindow.SetSize(wx.Display().GetGeometry().GetSize())
        app.MainLoop()
        return 0
    finally:
        release_instance_lock()


if __name__ == "__main__":
    disable_console_interaction()
    multiprocessing.freeze_support()
    multiprocessing.set_start_method("spawn")
    raise SystemExit(main())
