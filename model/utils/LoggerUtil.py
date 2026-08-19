import logging
import os
import glob
from datetime import datetime

# 设置日志文件保存的目录
log_directory = os.getcwd() + "\\data\\logs"
if not os.path.exists(log_directory):
    os.makedirs(log_directory)

# 创建日志记录器
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# 最大日志文件大小 (300MB)
MAX_LOG_SIZE = 300 * 1024 * 1024  # 300MB

# 当前日志文件路径
current_log_file = None


def setup_logger():
    global current_log_file
    # 生成带时间戳的日志文件名
    log_filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".log"
    log_filepath = os.path.join(log_directory, log_filename)

    # 创建文件处理器
    # Windows 下显式使用 UTF-8 with BOM，避免中文日志在记事本/部分查看器中出现乱码
    file_handler = logging.FileHandler(log_filepath, encoding="utf-8-sig")
    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(filename)s - %(lineno)d - %(message)s")
    file_handler.setFormatter(formatter)

    # 注释掉控制台输出处理器 - 这样logger就不会在控制台输出
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO)  # 终端只显示INFO及以上级别

    # 清除现有处理器
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # 添加新处理器 - 只添加文件处理器
    logger.addHandler(file_handler)
    # logger.addHandler(console_handler)  # 移除控制台输出

    current_log_file = log_filepath
    logger.info(f"Logging started in new file: {log_filepath}")
    return logger


# 初始化日志记录器
setup_logger()


def check_and_rotate_log():
    """检查日志文件大小，超过300MB时创建新日志文件"""
    global current_log_file
    if current_log_file and os.path.exists(current_log_file):
        file_size = os.path.getsize(current_log_file)
        if file_size > MAX_LOG_SIZE:
            logger.info(f"Log file size ({file_size} bytes) exceeds limit, rotating log...")
            setup_logger()


def manage_log_files(log_directory, max_files=100):
    """管理日志文件数量，删除旧日志"""
    log_files = glob.glob(os.path.join(log_directory, "*.log"))
    if len(log_files) > max_files:
        log_files.sort(key=os.path.getmtime)
        files_to_delete = log_files[: len(log_files) - max_files]
        for file_path in files_to_delete:
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
                    logger.info(f"Deleted oldest log file: {file_path}")
                else:
                    logger.warning(f"File not found: {file_path}")
            except Exception as e:
                logger.error(f"Delete failed: {file_path} - {str(e)}")
