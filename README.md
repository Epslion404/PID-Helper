# PIDHelper

## 介绍
在上位机使用PSO算法帮助下位机迭代调PID参数  
运行 `demo.py` 可以查看模拟情况

## 安装教程

1.  安装python 3.10.x
2.  安装库 `pyserial` , `numpy` , `parse`

## 使用说明

1.  安装python与必要库
2.  配置通信（见下文）
3.  运行 `PID_Helper.py` , 保持良好通信, 等待迭代完成

## 配置通信

> 默认使用串口与下位机进行通信，如果使用其他方式通信需要继承 `CommunicationInterface` 类  

1. 在下位机添加 `时间乘以绝对误差积分(ITAE)` 、 `过冲/超调程度(%)` 、 `调节时间` 、 `稳态误差` 的算法（见下文）
2. 在下位机构建上位机命令解析器与消息格式化函数
3. 运行 `PID_Helper.py` 配置参数

## PID评估参数算法

### ITAE

时间乘以绝对误差的积分

$$
ITAE = \int_{t_{start}}^{t_{stop}} t|e(t)| dt
$$

### 过冲/超调程度(%)

$$
OVERSHOOT =

\begin{cases}

\frac{max(OUT_{PID}) - Target}{Target}, & \text{if } max(OUT_{PID}) - Target > 0 \\
0, & \text{if } max(OUT_{PID}) - Target \leq 0

\end{cases}
$$

### 调节时间

进入±2%误差带的时间，也可以使用其它算法

### 稳态误差

最后10%时间的平均误差，也可以使用其它算法