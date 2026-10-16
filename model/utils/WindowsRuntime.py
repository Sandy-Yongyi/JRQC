"""Windows 启动支持，不依赖 GUI 或设备驱动。"""

import ctypes
from ctypes import wintypes
import sys


_instance_handle = None
MUTEX_NAME = "Global\\MainExeMutex"
ERROR_ALREADY_EXISTS = 183


def is_already_running() -> bool:
    """创建并持有单实例锁；64 位 Windows 上使用正确的 HANDLE 类型。"""
    global _instance_handle
    if sys.platform != "win32" or _instance_handle is not None:
        return False
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateMutexW.argtypes = (ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR)
    kernel32.CreateMutexW.restype = wintypes.HANDLE
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    kernel32.CloseHandle.restype = wintypes.BOOL
    handle = kernel32.CreateMutexW(None, False, MUTEX_NAME)
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    if ctypes.get_last_error() == ERROR_ALREADY_EXISTS:
        kernel32.CloseHandle(handle)
        return True
    _instance_handle = handle
    return False


def release_instance_lock() -> None:
    global _instance_handle
    if _instance_handle is not None:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
        kernel32.CloseHandle.restype = wintypes.BOOL
        kernel32.CloseHandle(_instance_handle)
        _instance_handle = None


def disable_console_interaction() -> None:
    """禁用 Quick Edit，避免选中控制台文字时暂停控制进程。"""
    if sys.platform != "win32":
        return
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetStdHandle.argtypes = (wintypes.DWORD,)
    kernel32.GetStdHandle.restype = wintypes.HANDLE
    kernel32.GetConsoleMode.argtypes = (wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD))
    kernel32.SetConsoleMode.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    handle = kernel32.GetStdHandle(-10)
    if handle is None or handle == ctypes.c_void_p(-1).value:
        return
    mode = wintypes.DWORD()
    if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
        kernel32.SetConsoleMode(handle, (mode.value | 0x0080) & ~0x0040)
