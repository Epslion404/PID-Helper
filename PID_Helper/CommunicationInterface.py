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

import serial
from typing import Optional
from PID_Helper.Log import setup_logger

logger = setup_logger(__file__)


class CommunicationInterface:
    def __init__(self, port: str = "", baud_rate: int = -1) -> None:
        self._port = port
        self._baud_rate = baud_rate
        self.serial: serial.Serial = serial.Serial()

        if port != "" and baud_rate > 0:
            try:
                self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
                if self.serial.is_open:
                    logger.info(f"串口打开成功: {port}, 波特率: {baud_rate}")
                else:
                    logger.critical("串口打开失败")
            except Exception as e:
                logger.critical(f"串口初始化失败: {e}")
                self.serial: serial.Serial = serial.Serial()
        else:
            logger.warning("无串口配置数据")

    def open(self, port: str, baud_rate: int) -> bool:
        try:
            if self.serial.is_open:
                self.serial.close()

            self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=1)
            if self.serial.is_open:
                logger.info(f"串口打开成功: {port}, 波特率: {baud_rate}")
                return True
            else:
                logger.critical("串口打开失败")
                return False
        except Exception as e:
            logger.critical(f"串口打开失败: {e}")
            return False

    def close(self) -> bool:
        if not self.serial.is_open:
            logger.warning("串口关闭失败，未打开串口")
            return False
        else:
            self.serial.close()
            if self.serial.is_open:
                logger.error("串口关闭失败")
                return False
            else:
                logger.info("串口关闭成功")
                return True

    def is_open(self) -> bool:
        return self.serial.is_open

    def is_close(self) -> bool:
        return not self.serial.is_open

    def write(self, data: str, encode: str = "ascii") -> bool:
        if not self.serial.is_open:
            logger.error("串口未打开")
            return False
        elif not self.serial.writable():
            logger.error("串口不可写")
            return False
        else:
            try:
                self.serial.write(data.encode(encode))
                return True
            except Exception as e:
                logger.error(f"发送失败：{e}")
                return False

    def read(self, size: int = 1, decode: str = "ascii") -> Optional[str]:
        if not self.serial.is_open:
            logger.error("串口未打开")
            return None
        elif not self.serial.readable():
            logger.error("串口不可读")
            return None
        else:
            try:
                buf = self.serial.read(size).decode(decode, errors="ignore").strip()
                return buf if buf else None
            except Exception as e:
                logger.error(f"接收失败：{e}")
                return None

    def read_line(self, decode: str = "ascii") -> Optional[str]:
        if not self.serial.is_open:
            logger.error("串口未打开")
            return None
        elif not self.serial.readable():
            logger.error("串口不可读")
            return None
        else:
            try:
                line = self.serial.readline().decode(decode, errors="ignore").strip()
                return line if line else None
            except Exception as e:
                logger.error(f"接收失败：{e}")
                return None
