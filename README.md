# PIDHelper

## 项目简介

一个基于粒子群优化算法（PSO）的上位机工具，用于自动化优化下位机的PID参数。  
通过迭代评估系统性能指标，自动寻找最优PID参数组合，提高调参效率。

运行 `demo.py` 可查看仿真示例。

## 环境要求

- Python 3.10.x
- 依赖库：
  ```bash
  pip install pyserial numpy parse
  ```

## 快速开始

1. **安装环境**  
   安装Python 3.10及上述依赖库。

2. **配置通信接口**  
   默认使用串口通信。若需其他通信方式（如TCP/IP），请继承 `CommunicationInterface` 类实现自定义接口。

3. **运行优化程序**  
   ```bash
   python PID_Helper.py
   ```
   确保通信正常，程序将自动迭代直至完成。

## 通信配置

### 1. 下位机需实现的功能
下位机需计算以下性能指标并向上位机返回：
- **ITAE**（时间乘以绝对误差积分）
- **超调量（%）**
- **调节时间**
- **稳态误差**

### 2. 通信协议
- 上位机发送PID参数至下位机
- 下位机执行控制过程，计算性能指标并返回结果
- 通信格式需与上位机解析器匹配

### 3. 自定义通信接口
如需非串口通信，请按以下示例实现：
```python
from communication_interface import CommunicationInterface

class CustomCommunication(CommunicationInterface):
    def send(self, data):
        # 实现发送逻辑
        pass
    def receive(self):
        # 实现接收逻辑
        pass
```

## PID性能指标算法

### 1. ITAE（时间乘以绝对误差积分）
$$
ITAE = \int_{t_{start}}^{t_{stop}} t \cdot |e(t)| \, dt
$$

### 2. 超调量（%）
$$
\text{超调量} = 
\begin{cases} 
\frac{\max(\text{OUT}_{\text{PID}}) - \text{Target}}{\text{Target}} \times 100\%, & \text{if } \max(\text{OUT}_{\text{PID}}) > \text{Target} \\
0\%, & \text{otherwise}
\end{cases}
$$

### 3. 调节时间
系统响应进入并保持在目标值±2%误差带内所需的时间。  
*注：可根据实际需求调整误差带范围。*

### 4. 稳态误差
取响应过程最后10%时间段内误差的平均值。  
*注：也可采用其他稳态误差计算方法。*

## 项目结构
```
PIDHelper/
├── PID_Helper.py      # 主程序
├── communication_interface.py  # 通信接口基类
├── demo.py            # 示例程序
└── README.md
```

## 注意事项
- 确保通信稳定，避免数据丢失
- 下位机计算性能指标时需与上位机定义的时间基准一致
- 可调整PSO算法参数以平衡收敛速度与精度

## 许可证
开源项目，遵循MIT许可证。