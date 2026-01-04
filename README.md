# PIDHelper

## 项目简介

一个基于粒子群优化算法（PSO）的上位机工具，用于自动化优化下位机的PID参数。  
通过迭代评估系统性能指标，自动寻找最优PID参数组合，提高调参效率。  
运行 `demo.py` 可查看仿真示例。

## 环境要求

- Python 3.10.x
- 依赖库：
  ```bash
  pip install pyserial numpy parse matplotlib scipy
  ```
- 或者直接使用项目自带的venv虚拟环境

## 快速开始

1. **安装环境**  
   安装Python 3.10及上述依赖库。

2. **配置通信接口**  
   默认使用串口通信。若需其他通信方式（如TCP/IP），请继承 `CommunicationInterface` 类实现自定义接口。

3. **运行优化程序**  
   ```bash
   python One_click_Start.py
   ```
   程序启动后，根据提示选择运行模式，确保通信正常，程序将自动迭代直至完成。

   > 没有安装Python请在release页面下载可执行文件直接运行

## 运行模式说明

程序支持以下三种运行模式：

### 模式1：手动输入模式（调试模式）

- **使用场景**：适合开发调试阶段，无需硬件设备
- **工作流程**：
  - 每次迭代时由用户手动在控制台输入性能指标（ITAE、超调量、调节时间、稳态误差）
  - 上位机基于输入数据通过PSO算法计算优化方向
  - 适合算法验证和小规模参数优化测试

### 模式2：全自动模式（下位机计算性能指标）

- **使用场景**：实际硬件自动化优化，下位机具备性能指标计算能力
- **工作流程**：
  1. 上位机通过串口发送PID参数至下位机
  2. 下位机执行PID控制，实时计算ITAE、超调量、调节时间、稳态误差等指标
  3. 下位机返回计算结果至上位机
  4. 上位机通过PSO算法迭代优化参数

### 模式3：全自动模式（上位机计算性能指标）

- **使用场景**：实际硬件自动化优化，上位机分析原始数据
- **工作流程**：
  1. 上位机通过串口发送PID参数至下位机
  2. 下位机执行PID控制，返回目标值（TARGET）和实际输出值（OUTPUT）等原始数据
  3. 上位机接收数据并根据配置的控制周期 $\Delta t$ 计算性能指标
  4. 上位机通过PSO算法迭代优化参数

## 通信配置

### 1. 数据格式配置

程序支持自定义数据解析格式，使用Parse库语法：

**模式2数据格式示例**：

```
下位机返回: ITAE:0.1234,OVERSHOOT:2.34,SETTLING:1.0,SSE:0.0456

解析格式: "ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{settling:f},SSE:{sse:f}"
```

> 解析格式中的 `itae` 、`overshoot` 、 `setting` 、 `sse` （小写部分）不可修改

**模式3数据格式示例**：

```
下位机返回: TARGET:100.0,OUTPUT:98.5

解析格式: "TARGET:{target:f},OUTPUT:{output:f}"
```

> 解析格式中的 `target` 、`output` （小写部分）不可修改

### 2. 下位机需实现的功能

**模式2（下位机计算指标）**：  
下位机需计算以下性能指标并返回：
- **ITAE**（时间乘以绝对误差积分）
- **超调量（%）**
- **调节时间**
- **稳态误差**

**模式3（上位机计算指标）**：  
下位机需返回：
- **TARGET**（目标值）
- **OUTPUT**（PID输出值）

### 3. 通信协议

- 上位机发送PID参数至下位机：`KP:value,KI:value,KD:value\n`
- 下位机执行控制过程，计算性能指标或返回原始数据
- 通信格式需与上位机解析器匹配

> 具体过程参考 `PID_Helper.py` → `FastPSO_PID_Optimizer` → `optimize()` 和 `evaluate_particle()`

### 4. 自定义通信接口

如需非串口通信（如TCP/IP、UDP等），请按以下示例实现：

```python
from PID_Helper.CommunicationInterface import CommunicationInterface

class CustomCommunication(CommunicationInterface):
    def open(self, port: str, baud_rate: int) -> bool:
        # 实现自定义打开逻辑
        pass
    
    def read_line(self, decode: str = "ascii") -> str | None:
        # 实现自定义读取逻辑
        pass
    
    def write(self, data: str, encode: str = "ascii") -> bool:
        # 实现自定义写入逻辑
        pass
    
    def close(self) -> bool:
        # 实现自定义关闭逻辑
        pass
```

## PID性能指标算法

### 1. ITAE（时间乘以绝对误差积分）

连续形式：
$$
\text{ITAE} = \int_{t_{start}}^{t_{stop}} t \cdot |e(t)| \, dt
$$

离散形式：
$$
\text{ITAE} = \sum_{i=0}^{n} |e_i| \cdot t_i \cdot \Delta t
$$

**C语言实现示例**：

```c
// 全局变量
float itae = 0;
float pid_time = 0;

// PID运行代码
void pid_run(float dt) {
    // ... PID计算 ...
    
    pid_time += dt;
    itae += fabsf(pid.error) * pid_time * dt;
}

// PID停止代码
void pid_stop() {
    // 串口输出ITAE
    printf("ITAE:%.4f\n", itae);
    
    // 重置变量
    itae = 0;
    pid_time = 0;
}
```

### 2. 超调量（%）

$$
\text{超调量} = 

\begin{cases} 

\frac{\max(\text{Measure}) - \text{Target}}{\text{Target}} \times 100\%, & \text{if } \max(\text{Measure}) > \text{Target} \\
0\%, & \text{otherwise}

\end{cases}
$$

**C语言实现示例**：
```c
// 全局变量
float overshoot = 0;
float max_measure = 0;

// PID运行代码
void pid_run(float dt) {
    // ... PID计算 ...
    
    // 更新最大测量值
    if (pid.measure > max_measure) {
        max_measure = pid.measure;
    }
}

// PID停止代码
void pid_stop() {
    // 计算超调量（百分比）
    if (max_measure > pid.target) {
        overshoot = (max_measure - pid.target) / pid.target * 100.0f;
    } else {
        overshoot = 0;
    }
    
    // 串口输出超调量
    printf("OVERSHOOT:%.4f\n", overshoot);
    
    // 重置变量
    overshoot = 0;
    max_measure = 0;
}
```

### 3. 调节时间

系统响应进入并保持在目标值±2%误差带内所需的时间。  

**C语言实现示例**：
```c
// 全局变量
float settling_time = 0;
float pid_time = 0;

// PID运行代码
void pid_run(float dt) {
    // ... PID计算 ...
    
    pid_time += dt;
    
    // 如果超出误差带，更新调节时间
    if (!(pid.measure > pid.target * 0.98f && pid.measure < pid.target * 1.02f)) {
        settling_time = pid_time;
    }
}

// PID停止代码
void pid_stop() {
    // 串口输出调节时间
    printf("SETTLING:%.4f\n", settling_time);
    
    // 重置变量
    settling_time = 0;
    pid_time = 0;
}
```

> 可根据实际需求调整误差带范围（默认2%，可在代码中修改）

### 4. 稳态误差

取响应过程最后10%时间段内的平均误差。  

**C语言实现示例**：
```c
// 全局变量
float sse = 0;
float evaluate_time = 5.0f;              // 评估时间（秒）
float start_cal_sse_time = evaluate_time * 0.9f;  // 开始计算SSE的时间
float sse_timer = 0;
int16_t cal_sse_tick = 0;                // 计算SSE期间经历PID计算的次数

// PID运行代码
void pid_run(float dt) {
    // ... PID计算 ...
    
    sse_timer += dt;
    
    if (sse_timer > start_cal_sse_time) {
        sse += fabsf(pid.error);
        cal_sse_tick++;
    }
}

// PID停止代码
void pid_stop() {
    // 计算平均稳态误差
    if (cal_sse_tick > 0) {
        sse = sse / cal_sse_tick;
    }
    
    // 串口输出稳态误差
    printf("SSE:%.4f\n", sse);
    
    // 重置变量
    sse = 0;
    sse_timer = 0;
    cal_sse_tick = 0;
}
```

> 也可采用其他稳态误差计算方法

## PSO算法参数说明

| 参数 | 说明 | 默认值 | 建议范围 |
|------|------|--------|----------|
| `pop_size` | 粒子群大小 | 10 | 10-15 |
| `max_iter` | 最大迭代次数 | 20 | 20-30 |
| `inertia_weight` | 惯性权重 | 0.7298 | 0.4-0.9 |
| `c1` | 个体学习因子 | 1.49618 | 1.0-2.0 |
| `c2` | 社会学习因子 | 1.49618 | 1.0-2.0 |
| `evaluation_delay` | 评估延迟（秒） | 0.5 | 0.1-1.0 |
| `max_evaluation_time` | 最大评估时间（秒） | 5.0 | 3.0-10.0 |

## 日志和结果保存

- **日志文件**：自动保存至 `logs/` 目录，格式为 `{module_name}-{timestamp}.log`
- **优化结果**：支持保存为JSON格式，包含最优参数、适应度历史和配置信息

**结果文件示例**：
```json
{
  "best_parameters": {
    "Kp": 1.2345,
    "Ki": 0.5678,
    "Kd": 0.0123
  },
  "best_fitness": 0.123456,
  "fitness_history": [1.23, 0.98, 0.76, ...],
  "config": { ... }
}
```

## 注意事项

- 确保通信稳定，避免数据丢失或格式错误
- 下位机计算性能指标时需与上位机定义的时间基准一致（特别是模式3）
- 可调整PSO算法参数以平衡收敛速度与精度
- 第一次运行建议使用模式1验证数据格式和算法流程
- 手动输入模式下，输入浮点数时需要加上 `.0` 后缀（如 `1.0` 而不是 `1`）
- 若串口无法打开，请检查：
  - 设备是否正确连接
  - 串口号是否正确（Windows: COM1, Linux: /dev/ttyUSB0）
  - 波特率是否匹配
  - 是否安装了必要的驱动程序
  - 是否有其他程序占用该串口

## 项目结构

```
PIDHelper/
├── One_click_Start.py          # 主启动程序
├── demo.py                     # 仿真示例
├── PID_Helper/
│   ├── __init__.py
│   ├── PID_Helper.py          # PSO优化器核心
│   ├── CommunicationInterface.py  # 通信接口
│   └── Log.py                 # 日志模块
├── logs/                      # 日志文件目录
└── README.md                  # 本文档
```

## 许可证

开源项目，遵循 [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) 许可证。
