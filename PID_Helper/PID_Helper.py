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

# 基于PSO算法的PID迭代算法

import serial.tools.list_ports as lp
import time
import numpy as np
import json
import logging
from parse import parse
from PID_Helper.CommunicationInterface import CommunicationInterface
from PID_Helper.Log import setup_logger

logger = setup_logger(__file__)


# PSO粒子
class Particle:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        best_fitness: float = 1e9,
        current_fitness: float = 1e9,
    ) -> None:
        self.position = np.array([kp, ki, kd], dtype=np.float64)  # 当前位置
        self.velocity = np.zeros(3, dtype=np.float64)  # 当前速度，形状固定为(3,)
        self.best_position = np.array(
            [kp, ki, kd], dtype=np.float64
        )  # 个体历史最优位置
        self.best_fitness: float = best_fitness  # 个体历史最优适应度
        self.current_fitness: float = current_fitness  # 当前适应度

        # 性能指标
        self.itae: float = 0.0
        self.overshoot: float = 0.0
        self.settling_time: float = 0.0
        self.steady_state_error: float = 0.0


# PSO配置类
class FastPSO_PID_Conf:
    def __init__(
        self,
        kp_min: float,
        ki_min: float,
        kd_min: float,
        kp_max: float,
        ki_max: float,
        kd_max: float,
        pop_size: int = 15,
        max_iter: int = 30,
        inertia_weight: float = 0.7298,
        c1: float = 1.49618,
        c2: float = 1.49618,
        constriction_factor: float = 0.0,
        evaluation_delay: float = 0.5,
        max_evaluation_time: float = 5.0,
    ) -> None:
        self.swarm_size: int = pop_size
        self.max_iterations: int = max_iter
        self.inertia_weight: float = inertia_weight
        self.c1: float = c1  # 加速系数
        self.c2: float = c2  # 加速系数

        # 收缩系数（Clerc and Kennedy, 2002） - 保证收敛
        self.constriction_factor: float = constriction_factor

        # 参数范围
        self.max_param = np.array([kp_max, ki_max, kd_max], dtype=np.float64)
        self.min_param = np.array([kp_min, ki_min, kd_min], dtype=np.float64)

        # 惯性权重衰减
        self.inertia_max: float = 0.9
        self.inertia_min: float = 0.4

        # 适应度权重
        self.weights = np.array(
            [1.0, 0.5, 0.3, 0.2], dtype=np.float64
        )  # itae, overshoot, settling_time, sse

        # 评估参数
        self.evaluation_delay: float = evaluation_delay  # 每次评估后的延迟
        self.max_evaluation_time: float = max_evaluation_time  # 最大评估时间


# 快速收敛的PSO优化器
class FastPSO_PID_Optimizer:
    def __init__(
        self,
        config: FastPSO_PID_Conf,
        comm_interface: CommunicationInterface,
        caculate_from_mcu: bool = False,
        f: str = "ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{setting:f},SSE:{sse:f}",
        start: str = "start",
        stop: str = "stop",
        delta_t: float = 0.0,
        manual_input_data: bool = False,
    ) -> None:
        """
        FastPSO_PID_Optimizer初始化

        :param comm_interface: FastPSO_PID_Optimizer配置类
        :type config: FastPSO_PID_Conf
        :param comm_interface: 通信类
        :type comm_interface: CommunicationInterface
        :param caculate_from_mcu: 是否从单片机计算性能数据，若为否，则需要从下位机接收PID运行过程参数
        :type caculate_from_mcu: bool
        :param f: 单片机数据输出格式，取决于 `caculate_from_mcu` 的值，需要ITAE、OverShoot、Setting、SSE或Target、Output
        :type f: str
        :param start: 启动pid命令
        :type start: str
        :param stop: 停止pid控制命令
        :type stop: str
        :param delta_t: pid调控周期（秒），仅在caculate_from_mcu为否时起作用
        :type delta_t: float
        :param manual_input_data: 手动输入数据
        :type manual_input_data: bool
        """
        self.config = config
        self.comm_interface = comm_interface
        self.format: str = f
        self.caculate_from_mcu: bool = caculate_from_mcu
        self.start_cmd: str = start
        self.stop_cmd: str = stop

        # 粒子群
        self.swarm: list[Particle] = []
        self.global_best_position: np.ndarray = np.zeros(3, dtype=np.float64)
        self.global_best_fitness: float = 1e9

        # 历史记录
        self.best_fitness_history: list[float] = []
        self.avg_fitness_history: list[float] = []

        # 统计
        self.evaluation_count: int = 0

        # 自适应参数
        self.success_count: float = 0.0
        self.total_updates: float = 0.0

        # 随机种子
        np.random.seed(int(time.time()))
        logger.debug(f"FastPSO_PID_Optimizer 初始化完成，随机种子: {int(time.time())}")

        # PID运行过程
        self.target_list: list = []
        self.output_list: list = []
        self.delta_t: float = delta_t
        self.manual_input_data: bool = manual_input_data

    def initialize_swarm(self) -> None:
        """初始化粒子群"""
        self.swarm = []

        for i in range(self.config.swarm_size):
            # 在参数范围内随机初始化位置
            kp = np.random.uniform(self.config.min_param[0], self.config.max_param[0])
            ki = np.random.uniform(self.config.min_param[1], self.config.max_param[1])
            kd = np.random.uniform(self.config.min_param[2], self.config.max_param[2])

            # 初始化速度（在参数范围的10%内）
            velocity_range = (self.config.max_param - self.config.min_param) * 0.1
            velocity = np.random.uniform(-velocity_range, velocity_range, size=3)

            particle = Particle(kp, ki, kd)
            particle.velocity = velocity.reshape(3)
            self.swarm.append(particle)

        logger.info(f"初始化完成，共{len(self.swarm)}个粒子")

    def send_pid_to_device(self, kp: float, ki: float, kd: float) -> bool:
        """发送PID参数到设备
        设备协议：发送"KP:value,KI:value,KD:value\n"
        """
        command = f"KP:{kp:.4f},KI:{ki:.4f},KD:{kd:.4f}\n"
        logger.debug(f"发送PID参数到设备: {command.strip()}")
        return self.comm_interface.write(command)

    def receive_pid_runtime_data(self) -> None:
        """从设备接收PID运行过程数据（target和output）"""
        if not self.comm_interface.is_open():
            logger.error("通信接口未打开")
            return

        # 清空之前的数据
        self.target_list.clear()
        self.output_list.clear()

        # 持续接收数据直到评估时间结束
        start_time = time.time()
        while time.time() - start_time < self.config.evaluation_delay:
            line = self.comm_interface.read_line()
            if line:
                try:
                    # 解析目标值和输出值
                    result = parse(self.format, line)
                    if result:
                        target = result.named.get("TARGET")
                        output = result.named.get("OUTPUT")
                        if target is not None and output is not None:
                            self.target_list.append(float(target))
                            self.output_list.append(float(output))
                            logger.debug(
                                f"接收到数据: Target={target}, Output={output}"
                            )
                except Exception as e:
                    logger.warning(f"解析运行数据失败: {e}, 原始数据: {line}")

            # 小延迟避免CPU占用过高
            time.sleep(0.001)

        logger.info(f"接收到 {len(self.target_list)} 个数据点")

    def receive_performance_from_device(self) -> tuple[float, float, float, float]:
        """从设备接收反馈参数
        返回格式：f + "\n"
        """
        start_time = time.time()

        if self.manual_input_data:
            while True:
                line = self.comm_interface.read_line()
                if line:
                    try:
                        result = parse(self.format, line)
                        logger.debug(f"从设备接收到的原始数据: {line.strip()}")

                        # 提取需要的指标
                        itae = result.named["itae"]
                        overshoot = result.named["overshoot"]
                        settling = result.named["setting"]
                        sse = result.named["sse"]

                        logger.info(
                            f"性能指标 - ITAE: {itae:.4f}, 超调: {overshoot:.2f}%, 稳定时间: {settling:.3f}s, 稳态误差: {sse:.4f}"
                        )
                        return itae, overshoot, settling, sse
                    except Exception as e:
                        logger.warning(f"解析设备反馈数据失败: {e}, 原始数据: {line}")
                        continue
        else:
            while time.time() - start_time < self.config.max_evaluation_time:
                line = self.comm_interface.read_line()
                if line:
                    try:
                        result = parse(self.format, line)
                        logger.debug(f"从设备接收到的原始数据: {line.strip()}")

                        # 提取需要的指标
                        itae = result.named["itae"]
                        overshoot = result.named["overshoot"]
                        settling = result.named["setting"]
                        sse = result.named["sse"]

                        logger.info(
                            f"性能指标 - ITAE: {itae:.4f}, 超调: {overshoot:.2f}%, 稳定时间: {settling:.3f}s, 稳态误差: {sse:.4f}"
                        )
                        return itae, overshoot, settling, sse
                    except Exception as e:
                        logger.warning(f"解析设备反馈数据失败: {e}, 原始数据: {line}")
                        continue

                time.sleep(0.01)

            # 超时返回默认值
            logger.error("等待反馈参数超时")
            raise RuntimeError("等待反馈参数超时")

    def caculate_mcu_data(
        self, evaluation_time: float
    ) -> tuple[float, float, float, float]:
        """根据MCU数据计算性能指标"""
        if len(self.target_list) < 2 or len(self.output_list) < 2:
            logger.critical("数据点不足，无法计算性能指标")
            raise RuntimeError("数据点不足，无法计算性能指标")

        # 确保目标值和输出值长度一致
        min_len = min(len(self.target_list), len(self.output_list))
        targets = np.array(self.target_list[:min_len])
        outputs = np.array(self.output_list[:min_len])

        # 稳态值（最后10%数据的平均值）
        steady_idx = int(min_len * 0.9)
        steady_target = np.mean(targets[steady_idx:])
        steady_output = np.mean(outputs[steady_idx:])

        # ITAE
        time_points = np.arange(min_len) * self.delta_t
        errors = targets - outputs
        itae: float = float(np.sum(np.abs(errors) * time_points))

        # 超调量
        if steady_target != 0:
            overshoot_percent = (
                (np.max(outputs) - steady_target) / np.abs(steady_target)
            ) * 100
        else:
            overshoot_percent = 0.0

        # 稳定时间
        error_band = 0.02 * np.abs(steady_target)
        settled = np.abs(outputs - steady_target) <= error_band

        # 第一次进入误差带后保持的时间
        settling_time = evaluation_time  # 默认值为评估时间
        for i in range(min_len - 1, 0, -1):
            if not settled[i]:
                settling_time = time_points[i]
                break

        # 稳态误差
        sse = np.abs(steady_target - steady_output)

        logger.info(
            f"计算性能指标 - ITAE: {itae:.4f}, 超调: {overshoot_percent:.2f}%, "
            f"稳定时间: {settling_time:.3f}s, 稳态误差: {sse:.4f}"
        )

        return itae, overshoot_percent, settling_time, sse

    def evaluate_particle(self, particle: Particle) -> bool:
        """评估单个粒子"""
        kp, ki, kd = particle.position

        logger.info(f"评估: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")

        # 发送参数到设备
        if not self.send_pid_to_device(kp, ki, kd):
            logger.error("发送参数失败")
            return False

        if not self.comm_interface.write(self.start_cmd):
            logger.error("发送启动指令失败")
            return False

        # 等待设备响应
        if not self.caculate_from_mcu:
            time.sleep(self.config.evaluation_delay)
        else:
            start_time = time.time()
            self.receive_pid_runtime_data()
            stop_time = time.time()
            evaluation_time = stop_time - start_time

        if not self.comm_interface.write(self.stop_cmd):
            logger.error("发送停止指令失败")
            return False

        # 接收性能指标
        if not self.caculate_from_mcu:
            itae, overshoot, settling_time, sse = self.receive_performance_from_device()
        else:
            itae, overshoot, settling_time, sse = self.caculate_mcu_data(
                evaluation_time
            )

        # 计算适应度（加权和，越小越好）
        fitness = (
            self.config.weights[0] * itae
            + self.config.weights[1] * overshoot
            + self.config.weights[2] * settling_time
            + self.config.weights[3] * sse
        )

        # 更新粒子信息
        particle.current_fitness = fitness
        particle.itae = itae
        particle.overshoot = overshoot
        particle.settling_time = settling_time
        particle.steady_state_error = sse

        # 更新个体最优
        if fitness < particle.best_fitness:
            particle.best_fitness = fitness
            particle.best_position = particle.position.copy()
            self.success_count += 1
            logger.debug(f"粒子个体最优更新: 适应度 {fitness:.4f}")

        self.evaluation_count += 1
        self.total_updates += 1

        logger.debug(
            f"评估完成，适应度: {fitness:.4f}, 总评估次数: {self.evaluation_count}"
        )
        return True

    def update_inertia_weight(self, iteration: int) -> None:
        """更新惯性权重"""
        # 线性衰减策略
        progress = iteration / self.config.max_iterations
        self.config.inertia_weight = (
            self.config.inertia_max
            - (self.config.inertia_max - self.config.inertia_min) * progress
        )
        logger.debug(
            f"迭代 {iteration}: 惯性权重更新为 {self.config.inertia_weight:.4f}"
        )

    def update_particle(self, particle: Particle, iteration: int) -> None:
        """更新粒子位置和速度"""
        r1 = np.random.random(3)
        r2 = np.random.random(3)

        # 更新速度（使用收缩系数）
        if self.config.constriction_factor > 0:
            cognitive = (
                self.config.c1 * r1 * (particle.best_position - particle.position)
            )
            social = (
                self.config.c2 * r2 * (self.global_best_position - particle.position)
            )
            new_velocity = self.config.constriction_factor * (
                self.config.inertia_weight * particle.velocity + cognitive + social
            )
            particle.velocity = np.array(new_velocity, dtype=np.float64).reshape(3)
        else:
            # 标准PSO更新
            new_velocity = (
                self.config.inertia_weight * particle.velocity
                + self.config.c1 * r1 * (particle.best_position - particle.position)
                + self.config.c2 * r2 * (self.global_best_position - particle.position)
            )
            particle.velocity = np.array(new_velocity, dtype=np.float64).reshape(3)

        # 更新位置
        particle.position += particle.velocity

        # 边界处理（反弹边界）
        for i in range(3):
            if particle.position[i] < self.config.min_param[i]:
                particle.position[i] = self.config.min_param[i]
                particle.velocity[i] = -particle.velocity[i] * 0.5
            elif particle.position[i] > self.config.max_param[i]:
                particle.position[i] = self.config.max_param[i]
                particle.velocity[i] = -particle.velocity[i] * 0.5

        logger.debug(f"粒子位置更新: {particle.position}, 速度: {particle.velocity}")

    def update_global_best(self) -> None:
        """更新全局最优解"""
        for particle in self.swarm:
            if particle.best_fitness < self.global_best_fitness:
                self.global_best_fitness = particle.best_fitness
                self.global_best_position = particle.best_position.copy()
                logger.info(
                    f"全局最优更新: 适应度 {self.global_best_fitness:.4f}, 位置: {self.global_best_position}"
                )

    def check_convergence(self, tolerance: float = 1e-4) -> bool:
        """检查是否收敛"""
        if len(self.best_fitness_history) < 5:
            return False

        # 检查最近5次迭代的改进
        recent_history = self.best_fitness_history[-5:]
        improvement = abs(recent_history[0] - recent_history[-1])

        logger.debug(f"收敛检查: 最近5次改进 {improvement:.6f}, 容忍度 {tolerance}")
        return improvement < tolerance

    def optimize(self, verbose: bool = True) -> tuple[np.ndarray, float]:
        """运行优化过程"""
        if not self.comm_interface.is_open():
            logger.error("错误: 通信接口未打开")
            return self.global_best_position, self.global_best_fitness

        logger.info("开始PSO优化过程")
        logger.info(
            f"种群大小: {self.config.swarm_size}, 最大迭代: {self.config.max_iterations}"
        )
        logger.info(
            f"参数范围: Kp[{self.config.min_param[0]}, {self.config.max_param[0]}], "
            f"Ki[{self.config.min_param[1]}, {self.config.max_param[1]}], "
            f"Kd[{self.config.min_param[2]}, {self.config.max_param[2]}]"
        )

        # 初始化粒子群
        self.initialize_swarm()

        # 初始评估
        logger.info("\n初始评估...")
        for particle in self.swarm:
            while not self.evaluate_particle(particle):
                logger.error("评估失败")
                retry = input("重试？(y/n)")
                retry = retry.lower()
                if retry == "y":
                    continue
                else:
                    break

        self.update_global_best()
        self.best_fitness_history.append(self.global_best_fitness)

        # 迭代优化
        for iteration in range(1, self.config.max_iterations + 1):
            if verbose:
                logger.info(f"\n迭代 {iteration}/{self.config.max_iterations}")

            # 更新惯性权重
            self.update_inertia_weight(iteration)

            # 更新所有粒子
            for particle in self.swarm:
                self.update_particle(particle, iteration)

            # 评估更新后的粒子
            logger.info(f"评估第{iteration}代...")
            for particle in self.swarm:
                self.evaluate_particle(particle)

            # 更新全局最优
            self.update_global_best()
            self.best_fitness_history.append(self.global_best_fitness)

            # 显示进度
            if verbose and (
                iteration % 5 == 0 or iteration == self.config.max_iterations
            ):
                logger.info(
                    f"进度: 迭代{iteration}, 最优适应度={self.global_best_fitness:.6f}"
                )
                logger.info(
                    f"最优参数: Kp={self.global_best_position[0]:.4f}, "
                    f"Ki={self.global_best_position[1]:.4f}, "
                    f"Kd={self.global_best_position[2]:.4f}"
                )

            # 检查收敛
            if self.check_convergence():
                logger.info(f"算法在第{iteration}代收敛")
                break

        # 输出结果
        self.print_results()

        return self.global_best_position, self.global_best_fitness

    def print_results(self) -> None:
        """打印优化结果"""
        logger.info("\n" + "=" * 50)
        logger.info("PSO优化完成!")
        logger.info("=" * 50)
        logger.info(f"总评估次数: {self.evaluation_count}")
        logger.info(f"最优参数:")
        logger.info(f"  Kp = {self.global_best_position[0]:.6f}")
        logger.info(f"  Ki = {self.global_best_position[1]:.6f}")
        logger.info(f"  Kd = {self.global_best_position[2]:.6f}")
        logger.info(f"最优适应度: {self.global_best_fitness:.6f}")

    def save_results(self, filename: str = "pso_pid_results.json") -> None:
        """保存优化结果到文件"""
        results = {
            "best_parameters": {
                "Kp": float(self.global_best_position[0]),
                "Ki": float(self.global_best_position[1]),
                "Kd": float(self.global_best_position[2]),
            },
            "best_fitness": float(self.global_best_fitness),
            "fitness_history": [float(f) for f in self.best_fitness_history],
            "config": {
                "kp_range": [
                    float(self.config.min_param[0]),
                    float(self.config.max_param[0]),
                ],
                "ki_range": [
                    float(self.config.min_param[1]),
                    float(self.config.max_param[1]),
                ],
                "kd_range": [
                    float(self.config.min_param[2]),
                    float(self.config.max_param[2]),
                ],
                "pop_size": self.config.swarm_size,
                "max_iterations": self.config.max_iterations,
            },
        }

        try:
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
            logger.info(f"结果已保存到: {filename}")
        except Exception as e:
            logger.error(f"保存结果失败: {e}")


def main() -> None:
    # 获取可用串口列表
    ports = list(lp.comports())
    if ports:
        logging.info("可用的串口列表：")
        for i in ports:
            logging.info(i.device)
    else:
        logging.error("未找到可用的串口。")

    port = input("请输入串口号").strip()
    baud_rate = int(input("请输入波特率").strip())

    comm_interface = CommunicationInterface(port, baud_rate)
    if not comm_interface.is_open():
        logging.critical("无法打开串口，退出……")
        return None

    # 配置PSO参数
    logging.info("\n\n配置PSO参数:")
    f = input(
        r"输入数据解析格式（默认：ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{setting:f},SSE:{sse:f}）："
    )
    if f == "":
        f = "ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{setting:f},SSE:{sse:f}"
    start_cmd = input("输入启动PID指令")
    stop_cmd = input("输入停止PID指令")
    use_default = input("使用默认配置? (y/n): ").strip().lower()

    if use_default == "y":
        config = FastPSO_PID_Conf(
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
        logging.info("\n输入参数范围:")
        kp_min = float(input("Kp最小值: "))
        kp_max = float(input("Kp最大值: "))
        ki_min = float(input("Ki最小值: "))
        ki_max = float(input("Ki最大值: "))
        kd_min = float(input("Kd最小值: "))
        kd_max = float(input("Kd最大值: "))

        logging.info("\n输入算法参数:")
        pop_size = int(input("种群大小: "))
        max_iter = int(input("最大迭代次数: "))
        eva_delay = float(input("每次评估后的延迟"))
        max_eva_delay = float(input("最大评估延迟"))

        config = FastPSO_PID_Conf(
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
    optimizer = FastPSO_PID_Optimizer(
        config, comm_interface, True, f, start_cmd, stop_cmd
    )

    # 运行优化
    try:
        best_params, best_fitness = optimizer.optimize(verbose=True)

        logging.info("\n优化完成!")
        logging.info(
            f"最优参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}"
        )
        logging.info(f"最优适应度: {best_fitness:.6f}")

        # 保存结果
        save = input("\n\n是否保存优化结果? (y/n): ").strip().lower()
        if save == "y":
            filename = input("请输入文件名 (默认: pso_pid_results.json): ").strip()
            if not filename:
                filename = "pso_pid_results.json"
            optimizer.save_results(filename)

    except KeyboardInterrupt:
        logging.info("\n用户终止优化过程")
    except Exception as e:
        logging.critical(f"\n优化过程中发生错误: {e}")
    finally:
        # 清理
        if comm_interface.is_open():
            comm_interface.close()

    return None


if __name__ == "__main__":
    main()
