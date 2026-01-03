# -*- coding:utf-8 -*- #

# Copyright 2025 Nept-Epslion
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import logging
from datetime import datetime


root_file_name = ""
log_file_name: str = str("")


class ColoredFormatter(logging.Formatter):
    """自定义彩色日志格式化器"""

    # 颜色代码
    COLORS = {
        "DEBUG": "\033[36m",  # 青色
        "INFO": "\033[32m",  # 绿色
        "WARNING": "\033[33m",  # 黄色
        "ERROR": "\033[31m",  # 红色
        "CRITICAL": "\033[41m",  # 红底白字
        "RESET": "\033[0m",  # 重置颜色
    }

    def format(self, record):
        # 获取原始格式化的日志
        message = super().format(record)

        # 为不同级别添加颜色
        if record.levelname in self.COLORS:
            color = self.COLORS[record.levelname]
            reset = self.COLORS["RESET"]
            message = f"{color}{message}{reset}"

        return message


def setup_logger(name=__file__):
    global log_file_name

    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)  # 设置最低级别

    if logger.handlers:
        return logger

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    console_formatter = ColoredFormatter("%(message)s")
    console_handler.setFormatter(console_formatter)

    os.makedirs("logs", exist_ok=True)

    # 按日期生成日志文件名
    log_filename = (
        f"logs/{root_file_name}-{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )

    if log_file_name == "":
        log_file_name = log_filename

    file_handler = logging.FileHandler(log_filename, encoding="utf-8")
    file_handler.setLevel(logging.INFO)

    file_formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler.setFormatter(file_formatter)

    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    return logger
