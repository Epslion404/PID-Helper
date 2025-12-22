# -*- coding:utf-8 -*- #

# 基于PSO算法的PID迭代算法

import serial
import serial.tools.list_ports
import random
import time
import numpy as np
import json
from typing import Optional


class CommunicationInterface:
    def __init__(self, port: str = "", baud_rate: int = -1) -> None:
        self._port = port
        self._baud_rate = baud_rate
        if port is not "" and baud_rate > 0:
            self.serial: serial.Serial = serial.Serial(port=port, baudrate=baud_rate)
            if self.serial.is_open:
                print("[CI]串口打开失败")
                self.serial: serial.Serial = serial.Serial()
            else:
                print("[CI]串口打开成功")
        else:
            self.serial: serial.Serial = serial.Serial()
            print("[CI]无串口配置数据")

    def open(self, port: str, baud_rate: int) -> bool:
        self.serial = serial.Serial(port=port, baudrate=baud_rate)
        if self.serial.is_open:
            print("[CI]串口打开失败")
            self.serial: serial.Serial = serial.Serial()
        else:
            print("[CI]串口打开成功")
        return self.serial.is_open

    def close(self) -> bool:
        if not self.serial.is_open:
            print("[CI]串口关闭失败，未打开串口")
            return False
        else:
            self.serial.close()
            if self.serial.open:
                print("[CI]串口关闭失败")
                return False
            else:
                print("[CI]串口关闭成功")
                return True

    def is_open(self) -> bool:
        return self.serial.is_open

    def is_close(self) -> bool:
        return not self.serial.is_open

    def write(self, data: str, encode: str = "ansi") -> bool:
        if not self.serial.writable:
            print("[CI]串口不可写")
            return False
        else:
            try:
                self.serial.write(data.encode(encode))
            except Exception as e:
                print(f"[CI]发送失败：{e}")
                return False
            return True

    def read(self, size: int = 1, decode: str = "ansi") -> Optional[str]:
        if not self.serial.readable:
            print("[CI]串口不可读")
            return None
        else:
            try:
                buf = self.serial.read(size).decode(decode, errors="ignore").strip()
                return buf if buf else None
            except Exception as e:
                print(f"[CI]接收失败：{e}")
                return None

    def read_line(self, decode: str = "ansi") -> Optional[str]:
        if not self.serial.readable:
            print("[CI]串口不可读")
            return None
        else:
            try:
                line = self.serial.readline().decode(decode, errors="ignore").strip()
                return line if line else None
            except Exception as e:
                print(f"[CI]接收失败：{e}")
                return None


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
        self.evaluation_delay: float = 0.5  # 每次评估后的延迟
        self.max_evaluation_time: float = 5.0  # 最大评估时间


# 快速收敛的PSO优化器
class FastPSO_PID_Optimizer:
    def __init__(
        self, config: FastPSO_PID_Conf, comm_interface: CommunicationInterface
    ) -> None:
        self.config = config
        self.comm_interface = comm_interface

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

        print(f"初始化完成，共{len(self.swarm)}个粒子")

    def send_pid_to_device(self, kp: float, ki: float, kd: float) -> bool:
        """发送PID参数到设备"""
        # 设备协议：发送"KP:value,KI:value,KD:value\n"
        command = f"KP:{kp:.4f},KI:{ki:.4f},KD:{kd:.4f}\n"
        return self.comm_interface.write(command)

    def receive_performance_from_device(self) -> tuple[float, float, float, float]:
        """从设备接收反馈参数"""
        # 返回格式："ITAE:value,OVERSHOOT:value,SETTLING:value,SSE:value\n"
        start_time = time.time()

        while time.time() - start_time < self.config.max_evaluation_time:
            line = self.comm_interface.read_line()
            if line:
                try:
                    # 解析性能数据
                    parts = line.split(",")
                    metrics = {}
                    for part in parts:
                        if ":" in part:
                            key, value = part.split(":")
                            metrics[key.strip()] = float(value.strip())

                    # 提取需要的指标
                    itae = metrics.get("ITAE", 0.0)
                    overshoot = metrics.get("OVERSHOOT", 0.0)
                    settling = metrics.get("SETTLING", 0.0)
                    sse = metrics.get("SSE", 0.0)

                    return itae, overshoot, settling, sse
                except:
                    continue

            time.sleep(0.01)

        # 超时返回默认值
        return 1000.0, 100.0, 10.0, 1.0

    def evaluate_particle(self, particle: Particle) -> bool:
        """评估单个粒子"""
        kp, ki, kd = particle.position

        print(f"评估: Kp={kp:.4f}, Ki={ki:.4f}, Kd={kd:.4f}")

        # 发送参数到设备
        if not self.send_pid_to_device(kp, ki, kd):
            print("发送参数失败")
            return False

        # 等待设备响应
        time.sleep(self.config.evaluation_delay)

        # 接收性能指标
        itae, overshoot, settling_time, sse = self.receive_performance_from_device()

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

        self.evaluation_count += 1
        self.total_updates += 1

        return True

    def update_inertia_weight(self, iteration: int) -> None:
        """更新惯性权重"""
        # 线性衰减策略
        progress = iteration / self.config.max_iterations
        self.config.inertia_weight = (
            self.config.inertia_max
            - (self.config.inertia_max - self.config.inertia_min) * progress
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

    def update_global_best(self) -> None:
        """更新全局最优解"""
        for particle in self.swarm:
            if particle.best_fitness < self.global_best_fitness:
                self.global_best_fitness = particle.best_fitness
                self.global_best_position = particle.best_position.copy()

    def check_convergence(self, tolerance: float = 1e-4) -> bool:
        """检查是否收敛"""
        if len(self.best_fitness_history) < 5:
            return False

        # 检查最近5次迭代的改进
        recent_history = self.best_fitness_history[-5:]
        improvement = abs(recent_history[0] - recent_history[-1])

        return improvement < tolerance

    def optimize(self, verbose: bool = True) -> tuple[np.ndarray, float]:
        """运行优化过程"""
        if not self.comm_interface.is_open():
            print("错误: 通信接口未打开")
            return self.global_best_position, self.global_best_fitness

        print("开始PSO优化过程")
        print(
            f"种群大小: {self.config.swarm_size}, 最大迭代: {self.config.max_iterations}"
        )
        print(
            f"参数范围: Kp[{self.config.min_param[0]}, {self.config.max_param[0]}], "
            f"Ki[{self.config.min_param[1]}, {self.config.max_param[1]}], "
            f"Kd[{self.config.min_param[2]}, {self.config.max_param[2]}]"
        )

        # 初始化粒子群
        self.initialize_swarm()

        # 初始评估
        print("\n初始评估...")
        for particle in self.swarm:
            self.evaluate_particle(particle)

        self.update_global_best()
        self.best_fitness_history.append(self.global_best_fitness)

        # 迭代优化
        for iteration in range(1, self.config.max_iterations + 1):
            if verbose:
                print(f"\n迭代 {iteration}/{self.config.max_iterations}")

            # 更新惯性权重
            self.update_inertia_weight(iteration)

            # 更新所有粒子
            for particle in self.swarm:
                self.update_particle(particle, iteration)

            # 评估更新后的粒子
            print(f"评估第{iteration}代...")
            for particle in self.swarm:
                self.evaluate_particle(particle)

            # 更新全局最优
            self.update_global_best()
            self.best_fitness_history.append(self.global_best_fitness)

            # 显示进度
            if verbose and (
                iteration % 5 == 0 or iteration == self.config.max_iterations
            ):
                print(
                    f"进度: 迭代{iteration}, 最优适应度={self.global_best_fitness:.6f}"
                )
                print(
                    f"最优参数: Kp={self.global_best_position[0]:.4f}, "
                    f"Ki={self.global_best_position[1]:.4f}, "
                    f"Kd={self.global_best_position[2]:.4f}"
                )

            # 检查收敛
            if self.check_convergence():
                print(f"算法在第{iteration}代收敛")
                break

        # 输出结果
        self.print_results()

        return self.global_best_position, self.global_best_fitness

    def print_results(self) -> None:
        """打印优化结果"""
        print("\n" + "=" * 50)
        print("PSO优化完成!")
        print("=" * 50)
        print(f"总评估次数: {self.evaluation_count}")
        print(f"最优参数:")
        print(f"  Kp = {self.global_best_position[0]:.6f}")
        print(f"  Ki = {self.global_best_position[1]:.6f}")
        print(f"  Kd = {self.global_best_position[2]:.6f}")
        print(f"最优适应度: {self.global_best_fitness:.6f}")

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
            print(f"结果已保存到: {filename}")
        except Exception as e:
            print(f"保存结果失败: {e}")


def main() -> None:
    # 获取可用串口列表
    ports = list(serial.tools.list_ports.comports())
    if ports:
        print("可用的串口列表：")
        for i in ports:
            print(i.device)
    else:
        print("未找到可用的串口。")

    port = input("请输入串口号").strip()
    baud_rate = int(input("请输入波特率").strip())

    comm_interface = CommunicationInterface(port, baud_rate)
    if not comm_interface.is_open():
        print("无法打开串口，退出……")
        return None

    # 配置PSO参数
    print("\n\n配置PSO参数:")
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
        print("\n输入参数范围:")
        kp_min = float(input("Kp最小值: "))
        kp_max = float(input("Kp最大值: "))
        ki_min = float(input("Ki最小值: "))
        ki_max = float(input("Ki最大值: "))
        kd_min = float(input("Kd最小值: "))
        kd_max = float(input("Kd最大值: "))

        print("\n输入算法参数:")
        pop_size = int(input("种群大小: "))
        max_iter = int(input("最大迭代次数: "))

        config = FastPSO_PID_Conf(
            kp_min=kp_min,
            kp_max=kp_max,
            ki_min=ki_min,
            ki_max=ki_max,
            kd_min=kd_min,
            kd_max=kd_max,
            pop_size=pop_size,
            max_iter=max_iter,
        )

    # 创建优化器
    optimizer = FastPSO_PID_Optimizer(config, comm_interface)

    # 运行优化
    try:
        best_params, best_fitness = optimizer.optimize(verbose=True)

        print("\n优化完成!")
        print(
            f"最优参数: Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}"
        )
        print(f"最优适应度: {best_fitness:.6f}")

        # 保存结果
        save = input("\n\n是否保存优化结果? (y/n): ").strip().lower()
        if save == "y":
            filename = input("请输入文件名 (默认: pso_pid_results.json): ").strip()
            if not filename:
                filename = "pso_pid_results.json"
            optimizer.save_results(filename)

    except KeyboardInterrupt:
        print("\n用户终止优化过程")
    except Exception as e:
        print(f"\n优化过程中发生错误: {e}")
    finally:
        # 清理
        if comm_interface.is_open():
            comm_interface.close()

    return None


if __name__ == "__main__":
    main()
