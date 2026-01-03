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

import numpy as np
from typing import List, Tuple, Optional
from PID_Helper.Log import setup_logger
import PID_Helper.Log as Log

Log.root_file_name = "demo"
import PID_Helper.CommunicationInterface as CI
import PID_Helper.PID_Helper as PH
import matplotlib
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
matplotlib.rcParams["axes.unicode_minus"] = False  # 正常显示负号

import time
import warnings

warnings.filterwarnings("ignore", category=UserWarning)


class PID:
    """PID"""

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        out_max: float = 999.0,
        out_min: float = -999.0,
        i_max: float = 999.0,
        time_step: float = 0.01,
    ) -> None:
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        self.exp: float = 0.0  # 期望
        self.mea: float = 0.0  # 测量
        self.mea1: float = 0.0  # 上次测量值
        self.err: float = 0.0  # 当前误差
        self.err1: float = 0.0  # 上次误差

        self.i: float = 0.0  # 积分
        self.i_max: float = i_max  # 积分限幅

        self.out: float = 0.0
        self.out_max: float = out_max
        self.out_min: float = out_min

        self.time_step: float = time_step

    def update(self) -> float:
        self.err = self.exp - self.mea

        # 积分项
        self.i += self.ki * self.err * self.time_step
        if self.i > self.i_max:
            self.i = self.i_max
        elif self.i < -self.i_max:
            self.i = -self.i_max

        # 微分项（使用误差微分）
        d: float = 0
        if self.time_step > 0:
            d = self.kd * (self.err - self.err1) / self.time_step

        self.err1 = self.err
        self.mea1 = self.mea

        self.out = self.kp * self.err + self.i + d

        if self.out > self.out_max:
            self.out = self.out_max
        elif self.out < self.out_min:
            self.out = self.out_min

        return self.out

    def reset(self) -> None:
        self.i = 0.0
        self.err1 = 0.0
        self.mea = 0.0
        self.mea1 = 0.0
        self.err = 0.0
        self.out = 0.0


class DemoSystem:
    """模拟被控系统（一阶惯性系统，AI写的）"""

    def __init__(self, time_step: float = 0.01):
        self.time_step = time_step
        self.position = 0.0
        self.velocity = 0.0

        # 系统参数
        self.time_constant = 0.5  # 时间常数（越小响应越快）
        self.noise_level = 0.001  # 测量噪声

    def update(self, control_signal: float) -> float:
        """更新系统状态，返回当前位置"""
        # 纯一阶惯性系统：T * dy/dt + y = K * u
        # 离散化：y[k] = (K*u*Δt + T*y[k-1]) / (T + Δt)

        # 更新位置
        self.position = (
            control_signal * self.time_step + self.time_constant * self.position
        ) / (self.time_constant + self.time_step)

        # 添加测量噪声
        noisy_position = self.position + np.random.normal(0, self.noise_level)

        return float(noisy_position)

    def reset(self):
        self.position = 0.0
        self.velocity = 0.0


class Demo_CI(CI.CommunicationInterface):
    def __init__(self, port: str = "", baud_rate: int = -1) -> None:
        # 不调用父类__init__，因为不需要真实串口
        self.w_data: str = ""
        self.r_data: Optional[str] = None
        self.current_pid: Optional[Tuple[float, float, float]] = None
        self.system = DemoSystem(time_step=0.01)
        self.pid_controller = PID(time_step=0.01)
        self.performance_history: List[dict] = []
        self._is_open = True  # 模拟串口打开状态

        # 性能指标统计
        self.evaluation_count = 0

    def open(self, port: str, baud_rate: int) -> bool:
        self._is_open = True
        return True

    def close(self) -> bool:
        self._is_open = False
        return True

    def is_open(self) -> bool:
        return self._is_open

    def is_close(self) -> bool:
        return not self._is_open

    def write(self, data: str, encode: str = "ansi") -> bool:
        self.w_data = data
        try:
            if data.startswith("KP:"):
                parts = data.strip().split(",")
                kp = float(parts[0].split(":")[1])
                ki = float(parts[1].split(":")[1])
                kd = float(parts[2].split(":")[1])

                # 设置PID参数
                self.pid_controller.kp = kp
                self.pid_controller.ki = ki
                self.pid_controller.kd = kd
                self.current_pid = (kp, ki, kd)

                # 运行模拟测试
                self._run_simulation(kp, ki, kd)

        except Exception as e:
            logger.error(f"解析PID参数失败: {e}")
            return False

        return True

    def _run_simulation(self, kp: float, ki: float, kd: float) -> None:
        """运行阶跃响应模拟测试"""
        # 重置系统
        self.system.reset()
        self.pid_controller.reset()

        # 设置参数
        self.pid_controller.kp = kp
        self.pid_controller.ki = ki
        self.pid_controller.kd = kd

        # 模拟参数
        sim_time = 10.0  # 模拟时间
        time_step = 0.01  # 时间步长
        steps = int(sim_time / time_step)
        setpoint = 1.0  # 阶跃信号

        # 响应数据
        time_points: List[float] = []
        positions: List[float] = []
        errors: List[float] = []
        outputs: List[float] = []

        # 运行
        for step in range(steps):
            t = step * time_step

            # 设置期望值（t=1秒时加入阶跃）
            if t < 1.0:
                self.pid_controller.exp = 0.0
            else:
                self.pid_controller.exp = setpoint

            # 获取测量值
            self.pid_controller.mea = self.system.position

            # 计算PID
            pid_output = self.pid_controller.update()

            # 更新系统状态
            current_position = self.system.update(pid_output)

            # 记录数据
            time_points.append(t)
            positions.append(current_position)
            errors.append(self.pid_controller.err)
            outputs.append(pid_output)

        # 计算性能指标
        itae, overshoot, settling_time, sse = self._calculate_performance(
            time_points, positions, errors, setpoint
        )

        # 存储性能指标
        self.performance_history.append(
            {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "itae": itae,
                "overshoot": overshoot,
                "settling_time": settling_time,
                "sse": sse,
                "time_points": time_points,
                "positions": positions,
                "errors": errors,
                "outputs": outputs,
            }
        )

        # 生成返回数据
        self.r_data = f"ITAE:{itae:.4f},OVERSHOOT:{overshoot:.4f},SETTLING:{settling_time:.4f},SSE:{sse:.4f}\n"

        self.evaluation_count += 1

        # 记录日志
        logger.info(
            f"评估 #{self.evaluation_count}: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}"
        )
        logger.info(
            f"  性能: ITAE={itae:.2f}, 超调={overshoot:.1f}%, 调节时间={settling_time:.2f}s, 稳态误差={sse:.4f}"
        )

    def _calculate_performance(
        self,
        time_points: List[float],
        positions: List[float],
        errors: List[float],
        setpoint: float,
    ) -> Tuple[float, float, float, float]:
        """计算性能指标"""
        if not positions:
            return 1000.0, 100.0, 10.0, 1.0

        # 转换为numpy
        time_array = np.array(time_points)
        position_array = np.array(positions)
        error_array = np.array(errors)

        # 找出阶跃开始时间
        step_start_idx = np.argmax(time_array >= 1.0)

        if step_start_idx >= len(time_array) or step_start_idx == 0:
            return 1000.0, 100.0, 10.0, 1.0

        # 阶跃后数据
        post_step_time = time_array[step_start_idx:] - 1.0
        post_step_position = position_array[step_start_idx:]
        post_step_error = error_array[step_start_idx:]

        if len(post_step_time) == 0:
            return 1000.0, 100.0, 10.0, 1.0

        # ITAE (时间乘以绝对误差积分)
        try:
            itae = float(
                np.trapezoid(post_step_time * np.abs(post_step_error), post_step_time)
            )
        except:
            itae = 1000.0

        # 超调量 (%)
        overshoot: float = 0.0
        if setpoint > 0 and len(post_step_position) > 0:
            max_value = float(np.max(post_step_position))
            if max_value > setpoint:
                overshoot = ((max_value - setpoint) / setpoint) * 100.0

        # 调节时间 (以进入±2%误差带的时间为调节时间为准)
        settling_time: float = 0.0
        if setpoint > 0 and len(post_step_position) > 0:
            settling_threshold = 0.02 * setpoint
            # 找到进入误差带的时间
            for i, pos in enumerate(post_step_position):
                if abs(pos - setpoint) <= settling_threshold:
                    # 检查后续是否保持在误差带内
                    settled = True
                    for j in range(i, min(i + 100, len(post_step_position))):
                        if abs(post_step_position[j] - setpoint) > settling_threshold:
                            settled = False
                            break
                    if settled:
                        settling_time = float(post_step_time[i])
                        break

            if settling_time == 0:
                settling_time = float(post_step_time[-1])

        # 稳态误差 (最后10%时间的平均误差)
        sse: float = 0.0
        if len(post_step_error) > 10:
            steady_start_idx = int(len(post_step_error) * 0.9)  # 最后10%
            sse = float(np.mean(np.abs(post_step_error[steady_start_idx:])))
        elif len(post_step_error) > 0:
            sse = float(np.abs(post_step_error[-1]))

        return itae, overshoot, settling_time, sse

    def read(self, size: int = 1, decode: str = "ansi") -> Optional[str]:
        result = self.r_data
        self.r_data = None
        return result

    def read_line(self, decode: str = "ansi") -> Optional[str]:
        result = self.r_data
        self.r_data = None
        return result

    def get_performance_history(self) -> List[dict]:
        return self.performance_history


class Demo_FastPSO_PID_Optimizer(PH.FastPSO_PID_Optimizer):
    def __init__(self, config: PH.FastPSO_PID_Conf, comm_interface: Demo_CI) -> None:
        super().__init__(config, comm_interface)
        self.convergence_history: List[dict] = []

    def evaluate_particle(self, particle: PH.Particle) -> bool:
        """重写评估方法，使用模拟通信"""
        kp, ki, kd = particle.position

        logger.info(f"评估: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")

        # 发送到设备
        if not self.comm_interface.write(f"KP:{kp:.4f},KI:{ki:.4f},KD:{kd:.4f}\n"):
            logger.error("发送参数失败")
            return False

        # 等待响应
        time.sleep(0.05)

        # 接收性能指标
        line = self.comm_interface.read_line()
        if line:
            try:
                # 解析数据
                parts = line.split(",")
                metrics = {}
                for part in parts:
                    if ":" in part:
                        key, value = part.split(":")
                        metrics[key.strip()] = float(value.strip())

                # 提取指标
                itae = metrics.get("ITAE", 0.0)
                overshoot = metrics.get("OVERSHOOT", 0.0)
                settling = metrics.get("SETTLING", 0.0)
                sse = metrics.get("SSE", 0.0)

                # 计算适应度（加权和，越小越好）
                fitness = (
                    self.config.weights[0] * itae
                    + self.config.weights[1] * overshoot
                    + self.config.weights[2] * settling
                    + self.config.weights[3] * sse
                )

                # 更新粒子信息
                particle.current_fitness = fitness
                particle.itae = itae
                particle.overshoot = overshoot
                particle.settling_time = settling
                particle.steady_state_error = sse

                # 更新个体最优
                if fitness < particle.best_fitness:
                    particle.best_fitness = fitness
                    particle.best_position = particle.position.copy()

                # 更新评估计数器
                self.evaluation_count += 1

                return True
            except Exception as e:
                logger.error(f"解析性能数据失败: {e}")
                return False
        else:
            logger.error("未接收到性能数据")
            return False

    def optimize(self, verbose: bool = True) -> tuple[np.ndarray, float]:
        # 重置历史记录
        self.convergence_history = []

        # 调用父类方法
        result = super().optimize(verbose)

        # 记录收敛历史
        if hasattr(self.comm_interface, "get_performance_history"):
            perf_history = self.comm_interface.get_performance_history()
            self.convergence_history = perf_history

            # 绘制收敛过程
            if len(self.convergence_history) > 5:
                self._plot_convergence()

        return result

    def _plot_convergence(self):
        """绘制收敛过程"""
        if not self.convergence_history:
            return

        plt.figure(figsize=(14, 10))

        # 适应度收敛曲线
        plt.subplot(2, 2, 1)
        fitness_values = [perf["itae"] for perf in self.convergence_history]
        plt.plot(
            range(len(fitness_values)), fitness_values, "b-o", linewidth=2, markersize=4
        )
        plt.xlabel("迭代代数", fontsize=12)
        plt.ylabel("ITAE", fontsize=12)
        plt.title("适应度收敛曲线", fontsize=14)
        plt.grid(True, alpha=0.3)

        # 参数进化过程
        plt.subplot(2, 2, 2)
        kp_values = [perf["kp"] for perf in self.convergence_history]
        ki_values = [perf["ki"] for perf in self.convergence_history]
        kd_values = [perf["kd"] for perf in self.convergence_history]

        plt.plot(
            range(len(kp_values)),
            kp_values,
            "r-o",
            label="Kp",
            linewidth=2,
            markersize=4,
        )
        plt.plot(
            range(len(ki_values)),
            ki_values,
            "g-o",
            label="Ki",
            linewidth=2,
            markersize=4,
        )
        plt.plot(
            range(len(kd_values)),
            kd_values,
            "b-o",
            label="Kd",
            linewidth=2,
            markersize=4,
        )

        plt.xlabel("迭代代数", fontsize=12)
        plt.ylabel("参数值", fontsize=12)
        plt.title("参数进化过程", fontsize=14)
        plt.legend()
        plt.grid(True, alpha=0.3)

        # 超调量变化
        plt.subplot(2, 2, 3)
        overshoot_values = [perf["overshoot"] for perf in self.convergence_history]
        plt.plot(
            range(len(overshoot_values)),
            overshoot_values,
            "m-o",
            linewidth=2,
            markersize=4,
        )
        plt.xlabel("迭代代数", fontsize=12)
        plt.ylabel("超调量 (%)", fontsize=12)
        plt.title("超调量变化", fontsize=14)
        plt.grid(True, alpha=0.3)

        # 稳态误差变化
        plt.subplot(2, 2, 4)
        sse_values = [perf["sse"] for perf in self.convergence_history]
        plt.plot(range(len(sse_values)), sse_values, "c-o", linewidth=2, markersize=4)
        plt.xlabel("迭代代数", fontsize=12)
        plt.ylabel("稳态误差", fontsize=12)
        plt.title("稳态误差变化", fontsize=14)
        plt.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


def manual_pid_test():
    """手动测试PID参数"""
    print("\n" + "=" * 50)
    print("PID参数手动测试")
    print("=" * 50)

    # 创建通信接口
    comm = Demo_CI()

    while True:
        print("\n请输入PID参数 (输入q退出):")

        try:
            kp_str = input("Kp (比例系数): ").strip()
            if kp_str.lower() == "q":
                break
            kp = float(kp_str)

            ki_str = input("Ki (积分系数): ").strip()
            if ki_str.lower() == "q":
                break
            ki = float(ki_str)

            kd_str = input("Kd (微分系数): ").strip()
            if kd_str.lower() == "q":
                break
            kd = float(kd_str)

        except ValueError:
            print("请输入有效的数字!")
            continue

        # 运行模拟
        print(f"\n测试参数: Kp={kp}, Ki={ki}, Kd={kd}")
        comm._run_simulation(kp, ki, kd)

        # 获取结果
        if comm.performance_history:
            perf = comm.performance_history[-1]

            print(f"性能指标:")
            print(f"  ITAE = {perf['itae']:.2f}")
            print(f"  超调量 = {perf['overshoot']:.1f}%")
            print(f"  调节时间 = {perf['settling_time']:.2f}s")
            print(f"  稳态误差 = {perf['sse']:.4f}")

            # 绘制响应曲线
            _plot_response(perf)

            # 计算适应度（使用PSO的权重）
            weights = np.array([1.0, 0.5, 0.3, 0.2])
            fitness = (
                weights[0] * perf["itae"]
                + weights[1] * perf["overshoot"]
                + weights[2] * perf["settling_time"]
                + weights[3] * perf["sse"]
            )
            print(f"  综合适应度 = {fitness:.4f} (越小越好)")

            # 询问是否继续
            cont = input("\n是否继续测试? (y/n): ").strip().lower()
            if cont != "y":
                break

    print("手动测试结束。")
    return comm.performance_history


def _plot_response(perf: dict):
    """绘制单个测试的响应曲线"""
    if "time_points" not in perf or "positions" not in perf:
        return

    plt.figure(figsize=(12, 8))

    # 系统响应
    plt.subplot(2, 2, 1)
    time_points = perf["time_points"]
    positions = perf["positions"]
    setpoint = 1.0

    plt.plot(time_points, positions, "b-", linewidth=2, label="PID响应")
    plt.axhline(y=setpoint, color="r", linestyle="--", linewidth=1.5, label="目标值")
    plt.axvline(x=1.0, color="g", linestyle=":", linewidth=1.5, label="起始时间")

    plt.xlabel("时间 (s)", fontsize=12)
    plt.ylabel("位置", fontsize=12)
    plt.title(
        f'系统响应 - Kp={perf["kp"]:.3f}, Ki={perf["ki"]:.3f}, Kd={perf["kd"]:.3f}',
        fontsize=14,
    )
    plt.grid(True, alpha=0.3)
    plt.legend()

    # 误差曲线
    plt.subplot(2, 2, 2)
    errors = perf["errors"]
    plt.plot(time_points, errors, "r-", linewidth=2)
    plt.xlabel("时间 (s)", fontsize=12)
    plt.ylabel("误差", fontsize=12)
    plt.title("误差曲线", fontsize=14)
    plt.grid(True, alpha=0.3)

    # PID输出
    plt.subplot(2, 2, 3)
    if "outputs" in perf:
        outputs = perf["outputs"]
        plt.plot(time_points, outputs, "g-", linewidth=2)
        plt.xlabel("时间 (s)", fontsize=12)
        plt.ylabel("PID输出", fontsize=12)
        plt.title("PID输出", fontsize=14)
        plt.grid(True, alpha=0.3)

    # 性能指标
    plt.subplot(2, 2, 4)
    metrics = ["ITAE", "过冲", "调节时间", "稳态误差"]
    values = [perf["itae"], perf["overshoot"], perf["settling_time"], perf["sse"]]
    colors = ["blue", "orange", "green", "red"]

    bars = plt.bar(metrics, values, color=colors)
    plt.ylabel("值", fontsize=12)
    plt.title("性能指标", fontsize=14)
    plt.grid(True, alpha=0.3, axis="y")

    # 在柱状图上显示数值
    for bar, value in zip(bars, values):
        height = bar.get_height()
        plt.text(
            bar.get_x() + bar.get_width() / 2.0,
            height + max(values) * 0.02,
            f"{value:.2f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    plt.tight_layout()
    plt.show()


def quick_test():
    """快速测试几组典型参数"""
    print("\n" + "=" * 50)
    print("快速测试 - 典型PID参数")
    print("=" * 50)

    # 创建通信接口
    comm = Demo_CI()

    # 测试几组典型参数
    test_params = [
        (0.5, 0.0, 0.0, "仅P控制"),
        (1.0, 0.2, 0.1, "PI控制"),
        (1.5, 0.3, 0.5, "PID控制"),
        (2.0, 0.5, 1.0, "强PID控制"),
        (0.8, 0.1, 0.3, "中等PID"),
    ]

    results = []
    for kp, ki, kd, desc in test_params:
        print(f"\n{desc}: Kp={kp}, Ki={ki}, Kd={kd}")
        comm._run_simulation(kp, ki, kd)

        if comm.performance_history:
            perf = comm.performance_history[-1]
            results.append((desc, perf))

            print(
                f"  性能: ITAE={perf['itae']:.2f}, 超调={perf['overshoot']:.1f}%, "
                f"调节时间={perf['settling_time']:.2f}s, 稳态误差={perf['sse']:.4f}"
            )

    # 找到最佳参数（ITAE最小）
    if results:
        best_desc, best_perf = min(results, key=lambda x: x[1]["itae"])
        print(f"\n最佳参数: {best_desc}")
        print(
            f"  Kp={best_perf['kp']:.2f}, Ki={best_perf['ki']:.2f}, Kd={best_perf['kd']:.2f}"
        )

        # 绘制最佳响应
        _plot_response(best_perf)

    return results


def main() -> None:
    print("=" * 50)
    print("PID参数优化演示系统(模拟阶跃响应)")
    print("=" * 50)

    while True:
        print("\n模式:")
        print("1. 快速测试（预定义参数）")
        print("2. 手动输入PID参数测试")
        print("3. 运行PSO优化")
        print("4. 退出")

        choice = input("请输入选项 (1-4): ").strip()

        if choice == "1":
            quick_test()

        elif choice == "2":
            manual_pid_test()

        elif choice == "3":
            print("\n配置PSO优化参数:")

            # 使用默认配置或自定义
            use_default = input("使用默认配置? (y/n): ").strip().lower()

            if use_default == "y":
                config = PH.FastPSO_PID_Conf(
                    kp_min=10.0,
                    kp_max=30.0,
                    ki_min=0.0,
                    ki_max=30.0,
                    kd_min=-10.0,
                    kd_max=10.0,
                    pop_size=6,
                    max_iter=8,
                )
            else:
                print("\n请输入参数范围:")
                kp_min = float(input("Kp最小值: "))
                kp_max = float(input("Kp最大值: "))
                ki_min = float(input("Ki最小值: "))
                ki_max = float(input("Ki最大值: "))
                kd_min = float(input("Kd最小值: "))
                kd_max = float(input("Kd最大值: "))

                print("\n请输入算法参数:")
                pop_size = int(input("种群大小 (建议6-10): "))
                max_iter = int(input("最大迭代次数 (建议8-15): "))

                config = PH.FastPSO_PID_Conf(
                    kp_min=kp_min,
                    kp_max=kp_max,
                    ki_min=ki_min,
                    ki_max=ki_max,
                    kd_min=kd_min,
                    kd_max=kd_max,
                    pop_size=pop_size,
                    max_iter=max_iter,
                )

            # 创建通信接口和优化器
            com = Demo_CI()
            optimizer = Demo_FastPSO_PID_Optimizer(config, com)

            # 运行优化
            try:
                print("\n开始优化...")
                best_params, best_fitness = optimizer.optimize(verbose=True)

                print("\n" + "=" * 50)
                print("优化完成!")
                print("=" * 50)
                print(
                    f"最优参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}"
                )
                print(f"最优适应度: {best_fitness:.6f}")
                print(f"总评估次数: {optimizer.evaluation_count}")

                # 显示最终响应
                print("\n绘制最终优化结果的阶跃响应...")
                com.system.reset()
                com.pid_controller.reset()
                com.pid_controller.kp = float(best_params[0])
                com.pid_controller.ki = float(best_params[1])
                com.pid_controller.kd = float(best_params[2])
                com._run_simulation(
                    float(best_params[0]), float(best_params[1]), float(best_params[2])
                )

                # 绘制最终响应
                if com.performance_history:
                    _plot_response(com.performance_history[-1])

                # 保存结果
                save = input("\n是否保存优化结果? (y/n): ").strip().lower()
                if save == "y":
                    filename = input("请输入文件名 (默认: pso_results.json): ").strip()
                    if not filename:
                        filename = "pso_results.json"
                    optimizer.save_results(filename)
                    print(f"结果已保存到 {filename}")

            except KeyboardInterrupt:
                print("\n用户终止优化过程")
            except Exception as e:
                print(f"\n优化过程中发生错误: {e}")

        elif choice == "4":
            print("退出程序")
            break

        else:
            print("无效选项，请重新输入")


if __name__ == "__main__":
    logger = setup_logger(__file__)
    main()
