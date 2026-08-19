import os
import wx
import sys
import ctypes
import multiprocessing
from model.utils.LoggerUtil import logger
from control.MainFrameControl import MainApp


def is_already_running():
    """检查是否已有程序实例在运行"""
    mutex_name = "Global\\MainExeMutex"

    try:
        # 尝试创建互斥体
        ctypes.windll.kernel32.CreateMutexW(None, False, mutex_name)
        last_error = ctypes.windll.kernel32.GetLastError()

        # 如果错误码是183（已存在），则返回True
        return last_error == 183  # ERROR_ALREADY_EXISTS
    except Exception as e:
        logger.error(f"检查单实例出错: {str(e)}")
        return True


def disable_console_interaction():
    """安全禁用控制台交互"""
    if sys.platform == 'win32':
        kernel32 = ctypes.WinDLL('kernel32')
        STD_INPUT_HANDLE = -10

        hStdin = kernel32.GetStdHandle(STD_INPUT_HANDLE)
        if hStdin is None or hStdin == ctypes.c_void_p(-1).value:
            return

        mode = ctypes.c_ulong()
        if kernel32.GetConsoleMode(hStdin, ctypes.byref(mode)):
            ENABLE_QUICK_EDIT_MODE = 0x0040
            new_mode = mode.value & ~ENABLE_QUICK_EDIT_MODE
            kernel32.SetConsoleMode(hStdin, new_mode)


def main():
    logger.info(f"main process ID: {os.getpid()}")

    if is_already_running():
        ctypes.windll.user32.MessageBoxW(
            0,
            "程序已在运行中，请不要启动多个实例",
            "错误提示",
            0x10  # MB_ICONERROR
        )
        logger.error("检测到已有实例运行，退出程序")
        sys.exit(1)  # 非零退出码表示异常退出

    # 创建GUI应用
    app = MainApp()
    # 获取屏幕尺寸并设置窗口大小为屏幕尺寸
    screen = wx.Display().GetGeometry()
    app.TopWindow.SetSize(screen.GetSize())
    app.MainLoop()

    # 调试激光---------------------------------------------------------------------
    # lidar_processor = LidarControl(
    #     config_path=os.path.join(os.getcwd() + "\\model\\tomls\\ReadDataConfig.toml"),
    #     lidar_config_path=os.path.join(os.getcwd() + "\\model\\tomls\\LidarConfig.toml")
    # )
    # # lidar_processor.process_all_lidar_data()
    # lidar_processor.process_one_lidar_data("4")

    # 调试PLC----------------------------------------------------------------------
    # plc_processor = PlcControl()


if __name__ == "__main__":
    disable_console_interaction()
    multiprocessing.freeze_support()
    multiprocessing.set_start_method('spawn')
    main()
