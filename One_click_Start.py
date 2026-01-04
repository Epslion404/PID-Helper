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

import PID_Helper.Log as Log

Log.root_file_name = "One_click_Start"
import PID_Helper.PID_Helper as PH
import PID_Helper.CommunicationInterface as CI
from PID_Helper.Log import setup_logger
import serial.tools.list_ports as lp


class CI_Console(CI.CommunicationInterface):
    def __init__(self, port: str = "", baud_rate: int = -1) -> None:
        # super().__init__(port, baud_rate)
        return None

    def open(self, port: str, baud_rate: int) -> bool:
        return True

    def is_open(self) -> bool:
        return True

    def is_close(self) -> bool:
        return True

    def close(self) -> bool:
        return True

    def write(self, data: str, encode: str = "ascii") -> bool:
        logger.info(data)
        return True

    def read(self, size: int = 1, decode: str = "ascii") -> str | None:
        return input("请在这里输入(注意不要忘了在整数末尾加上.0)：")

    def read_line(self, decode: str = "ascii") -> str | None:
        return input("请在这里输入(注意不要忘了在整数末尾加上.0)：")


def main() -> None:

    logger.info("运行模式：")
    logger.info("1. 手动输入模式（下位机数据分析）：")
    logger.info("2. 全自动模式（下位机数据分析）：")
    logger.info("3. 全自动模式（上位机数据分析）：")
    logger.info("请输入运行模式：")
    mode: str = input()

    if (not (mode in [str(i) for i in range(1, 4)])) or mode == "":
        logger.critical("无效选项，请重新输入")
        return None

    comm_interface = None
    caculate_from_mcu = False
    manual_input_data = False
    delta_t = 0.0

    if mode == "1":
        logger.info("手动输入模式，使用控制台通信")
        comm_interface = CI_Console("", -1)
        caculate_from_mcu = False
        manual_input_data = True

    else:
        logger.info("全自动模式，使用串口通信")

        # 获取可用串口列表
        ports = list(lp.comports())
        if ports:
            logger.info("可用的串口列表：")
            for i in ports:
                logger.info(i.description)
        else:
            logger.critical("未找到可用的串口。")
            return None

        port = input("请输入串口号: ").strip()
        baud_rate = int(input("请输入波特率: ").strip())

        comm_interface = CI.CommunicationInterface(port, baud_rate)
        if not comm_interface.is_open():
            logger.critical("无法打开串口，退出……")
            return None

        if mode == "2":
            caculate_from_mcu = False
        elif mode == "3":
            caculate_from_mcu = True
            delta_t = float(input("请输入PID调控周期(秒): ").strip())

    # 配置PSO参数
    logger.info("\n\n配置PSO参数:")
    if caculate_from_mcu:
        f = input(
            r"输入数据解析格式（需要TARGET和OUTPUT，例如：TARGET:{target:f},OUTPUT:{output:f}）："
        )
        if f == "":
            f = "TARGET:{target:f},OUTPUT:{output:f}"
    else:
        f = input(
            r"输入数据解析格式（默认：ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{setting:f},SSE:{sse:f}）："
        )
        if f == "":
            f = "ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{setting:f},SSE:{sse:f}"

    start_cmd = input("输入启动PID指令: ")
    stop_cmd = input("输入停止PID指令: ")
    use_default = input("使用默认PSO配置? (y/n): ").strip().lower()

    if use_default == "y":
        config = PH.FastPSO_PID_Conf(
            kp_min=0.1,
            kp_max=10.0,
            ki_min=0.0,
            ki_max=5.0,
            kd_min=0.0,
            kd_max=5.0,
            pop_size=10,
            max_iter=20,
        )
    else:
        # 自定义配置
        logger.info("\n输入参数范围:")
        kp_min = float(input("Kp最小值: "))
        kp_max = float(input("Kp最大值: "))
        ki_min = float(input("Ki最小值: "))
        ki_max = float(input("Ki最大值: "))
        kd_min = float(input("Kd最小值: "))
        kd_max = float(input("Kd最大值: "))

        logger.info("\n输入算法参数:")
        pop_size = int(input("种群大小: "))
        max_iter = int(input("最大迭代次数: "))
        eva_delay = float(input("每次评估后的延迟: "))
        max_eva_delay = float(input("最大评估延迟: "))

        config = PH.FastPSO_PID_Conf(
            kp_min=kp_min,
            kp_max=kp_max,
            ki_min=ki_min,
            ki_max=ki_max,
            kd_min=kd_min,
            kd_max=kd_max,
            pop_size=pop_size,
            max_iter=max_iter,
            evaluation_delay=eva_delay,
            max_evaluation_time=max_eva_delay,
        )

    # 创建优化器
    optimizer = PH.FastPSO_PID_Optimizer(
        config,
        comm_interface,
        caculate_from_mcu,
        f,
        start_cmd,
        stop_cmd,
        delta_t,
        manual_input_data,
    )

    # 运行优化
    try:
        best_params, best_fitness = optimizer.optimize(verbose=True)

        logger.info("\n优化完成!")
        logger.info(
            f"最优参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}"
        )
        logger.info(f"最优适应度: {best_fitness:.6f}")

        # 保存结果
        save = input("\n\n是否保存优化结果? (y/n): ").strip().lower()
        if save == "y":
            filename = input("请输入文件名 (默认: pso_pid_results.json): ").strip()
            if not filename:
                filename = "pso_pid_results.json"
            optimizer.save_results(filename)

    except KeyboardInterrupt:
        logger.info("\n用户终止优化过程")
    except Exception as e:
        logger.critical(f"\n优化过程中发生错误: {e}")
    finally:
        # 清理
        if comm_interface and comm_interface.is_open():
            comm_interface.close()

    return None


if __name__ == "__main__":
    logger = setup_logger(__file__)

    main()
