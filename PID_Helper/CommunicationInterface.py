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
import logging
from typing import Optional


class CommunicationInterface:
    def __init__(self, port: str = "", baud_rate: int = -1) -> None:
        self._port = port
        self._baud_rate = baud_rate
        if port != "" and baud_rate > 0:
            self.serial: serial.Serial = serial.Serial(port=port, baudrate=baud_rate)
            if self.serial.is_open:
                logging.critical("[CI]串口打开失败")
                self.serial: serial.Serial = serial.Serial()
            else:
                logging.info("[CI]串口打开成功")
        else:
            self.serial: serial.Serial = serial.Serial()
            logging.warning("[CI]无串口配置数据")

    def open(self, port: str, baud_rate: int) -> bool:
        self.serial = serial.Serial(port=port, baudrate=baud_rate)
        if self.serial.is_open:
            logging.critical("[CI]串口打开失败")
            self.serial: serial.Serial = serial.Serial()
        else:
            logging.info("[CI]串口打开成功")
        return self.serial.is_open

    def close(self) -> bool:
        if not self.serial.is_open:
            logging.warning("[CI]串口关闭失败，未打开串口")
            return False
        else:
            self.serial.close()
            if self.serial.open:
                logging.error("[CI]串口关闭失败")
                return False
            else:
                logging.info("[CI]串口关闭成功")
                return True

    def is_open(self) -> bool:
        return True

    def is_close(self) -> bool:
        return not self.serial.is_open

    def write(self, data: str, encode: str = "ansi") -> bool:
        if not self.serial.writable:
            logging.error("[CI]串口不可写")
            return False
        else:
            try:
                self.serial.write(data.encode(encode))
            except Exception as e:
                logging.error(f"[CI]发送失败：{e}")
                return False
            return True

    def read(self, size: int = 1, decode: str = "ansi") -> Optional[str]:
        if not self.serial.readable:
            logging.error("[CI]串口不可读")
            return None
        else:
            try:
                buf = self.serial.read(size).decode(decode, errors="ignore").strip()
                return buf if buf else None
            except Exception as e:
                logging.error(f"[CI]接收失败：{e}")
                return None

    def read_line(self, decode: str = "ansi") -> Optional[str]:
        if not self.serial.readable:
            logging.error("[CI]串口不可读")
            return None
        else:
            try:
                line = self.serial.readline().decode(decode, errors="ignore").strip()
                return line if line else None
            except Exception as e:
                logging.error(f"[CI]接收失败：{e}")
                return None
