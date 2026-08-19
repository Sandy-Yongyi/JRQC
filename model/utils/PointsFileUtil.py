import os
import glob
from model.utils.LoggerUtil import logger


def manage_points_files(max_files=1000):
    # 设置目标目录
    points_directory = os.path.join(os.getcwd() + '\\data\\points')

    # 创建目录（如果不存在）
    if not os.path.exists(points_directory):
        os.makedirs(points_directory)
        logger.info(f"Created directory: {points_directory}")
        return  # 新目录无需处理

    # 获取目录下所有文件（不包含子目录）
    all_files = glob.glob(os.path.join(points_directory, '*'))

    # 仅保留文件（排除目录）
    files = [f for f in all_files if os.path.isfile(f)]

    # 检查文件数量
    if len(files) <= max_files:
        logger.debug(f"Points directory within limit: {len(files)}/{max_files} files")
        return

    # 按修改时间排序（旧文件在前）
    files.sort(key=os.path.getmtime)

    # 计算需要删除的数量
    delete_count = len(files) - max_files
    files_to_delete = files[:delete_count]

    logger.warning(f"Deleting {delete_count} old files from points directory (limit={max_files})")

    # 删除旧文件
    for file_path in files_to_delete:
        try:
            os.remove(file_path)
            logger.info(f"Deleted old file: {os.path.basename(file_path)}")
        except Exception as e:
            logger.error(f"Delete failed: {file_path} - {str(e)}")
