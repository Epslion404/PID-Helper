# PIDHelper

## Project Introduction

A PC-based tool using Particle Swarm Optimization (PSO) algorithm to automatically optimize PID parameters for embedded systems.  
By iteratively evaluating system performance metrics, it automatically finds the optimal PID parameter combination, improving tuning efficiency.  
Run `demo.py` to view simulation examples.

## Environment Requirements

- Python 3.10.x
- Dependencies:
  ```bash
  pip install pyserial numpy parse matplotlib scipy
  ```
- Alternatively, use the built-in venv virtual environment

## Quick Start

1. **Install Environment**  
   Install Python 3.10 and the required dependencies.

2. **Configure Communication Interface**  
   Default uses serial communication. For other communication methods (e.g., TCP/IP), inherit the `CommunicationInterface` class to implement custom interfaces.

3. **Run Optimization Program**  
   ```bash
   python One_click_Start.py
   ```
   After startup, follow prompts to select operation mode. Ensure communication is normal, and the program will automatically iterate until completion.

   > If Python is not installed, download the executable from the release page and run directly.

## Operation Mode Description

The program supports three operation modes:

### Mode 1: Manual Input Mode (Debug Mode)

- **Use Case**: Suitable for development and debugging, no hardware required
- **Workflow**:
  - User manually inputs performance metrics (ITAE, Overshoot, Settling Time, Steady-State Error) in console for each iteration
  - PC calculates optimization direction using PSO algorithm based on input data
  - Suitable for algorithm verification and small-scale parameter optimization testing

### Mode 2: Fully Automatic Mode (Embedded System Calculates Performance Metrics)

- **Use Case**: Actual hardware automated optimization, embedded system has performance metric calculation capability
- **Workflow**:
  1. PC sends PID parameters to embedded system via serial port
  2. Embedded system executes PID control, calculates ITAE, Overshoot, Settling Time, Steady-State Error metrics in real-time
  3. Embedded system returns calculation results to PC
  4. PC iteratively optimizes parameters using PSO algorithm

### Mode 3: Fully Automatic Mode (PC Calculates Performance Metrics)

- **Use Case**: Actual hardware automated optimization, PC analyzes raw data
- **Workflow**:
  1. PC sends PID parameters to embedded system via serial port
  2. Embedded system executes PID control, returns raw data like Target Value (TARGET) and Actual Output Value (OUTPUT)
  3. PC receives data and calculates performance metrics based on configured control period $\Delta t$
  4. PC iteratively optimizes parameters using PSO algorithm

## Communication Configuration

### 1. Data Format Configuration

Program supports custom data parsing formats using Parse library syntax:

**Mode 2 Data Format Example**:

```
Embedded system returns: ITAE:0.1234,OVERSHOOT:2.34,SETTLING:1.0,SSE:0.0456

Parsing format: "ITAE:{itae:f},OVERSHOOT:{overshoot:f},SETTLING:{settling:f},SSE:{sse:f}"
```

> The lowercase parts in parsing format (`itae`, `overshoot`, `setting`, `sse`) cannot be modified.

**Mode 3 Data Format Example**:

```
Embedded system returns: TARGET:100.0,OUTPUT:98.5

Parsing format: "TARGET:{target:f},OUTPUT:{output:f}"
```

> The lowercase parts in parsing format (`target`, `output`) cannot be modified.

### 2. Embedded System Required Functions

**Mode 2 (Embedded System Calculates Metrics)**:  
Embedded system must calculate and return:
- **ITAE** (Integral of Time multiplied by Absolute Error)
- **Overshoot (%)**
- **Settling Time**
- **Steady-State Error**

**Mode 3 (PC Calculates Metrics)**:  
Embedded system must return:
- **TARGET** (Target value)
- **OUTPUT** (PID output value)

### 3. Communication Protocol

- PC sends PID parameters to embedded system: `KP:value,KI:value,KD:value\n`
- Embedded system executes control process, calculates performance metrics or returns raw data
- Communication format must match PC parser

> For detailed process, refer to `PID_Helper.py` → `FastPSO_PID_Optimizer` → `optimize()` and `evaluate_particle()`

### 4. Custom Communication Interface

For non-serial communication (e.g., TCP/IP, UDP), implement as follows:

```python
from PID_Helper.CommunicationInterface import CommunicationInterface

class CustomCommunication(CommunicationInterface):
    def open(self, port: str, baud_rate: int) -> bool:
        # Implement custom open logic
        pass
    
    def read_line(self, decode: str = "ascii") -> str | None:
        # Implement custom read logic
        pass
    
    def write(self, data: str, encode: str = "ascii") -> bool:
        # Implement custom write logic
        pass
    
    def close(self) -> bool:
        # Implement custom close logic
        pass
```

## PID Performance Metrics Algorithm

### 1. ITAE (Integral of Time multiplied by Absolute Error)

Continuous form:
$$
\text{ITAE} = \int_{t_{start}}^{t_{stop}} t \cdot |e(t)| \, dt
$$

Discrete form:
$$
\text{ITAE} = \sum_{i=0}^{n} |e_i| \cdot t_i \cdot \Delta t
$$

**C Language Example**:

```c
// Global variables
float itae = 0;
float pid_time = 0;

// PID execution code
void pid_run(float dt) {
    // ... PID calculation ...
    
    pid_time += dt;
    itae += fabsf(pid.error) * pid_time * dt;
}

// PID stop code
void pid_stop() {
    // Serial output ITAE
    printf("ITAE:%.4f\n", itae);
    
    // Reset variables
    itae = 0;
    pid_time = 0;
}
```

### 2. Overshoot (%)

$$
\text{Overshoot} = 

\begin{cases} 

\frac{\max(\text{Measure}) - \text{Target}}{\text{Target}} \times 100\%, & \text{if } \max(\text{Measure}) > \text{Target} \\
0\%, & \text{otherwise}

\end{cases}
$$

**C Language Example**:
```c
// Global variables
float overshoot = 0;
float max_measure = 0;

// PID execution code
void pid_run(float dt) {
    // ... PID calculation ...
    
    // Update maximum measurement
    if (pid.measure > max_measure) {
        max_measure = pid.measure;
    }
}

// PID stop code
void pid_stop() {
    // Calculate overshoot (percentage)
    if (max_measure > pid.target) {
        overshoot = (max_measure - pid.target) / pid.target * 100.0f;
    } else {
        overshoot = 0;
    }
    
    // Serial output overshoot
    printf("OVERSHOOT:%.4f\n", overshoot);
    
    // Reset variables
    overshoot = 0;
    max_measure = 0;
}
```

### 3. Settling Time

Time required for system response to enter and remain within ±2% error band of target value.

**C Language Example**:
```c
// Global variables
float settling_time = 0;
float pid_time = 0;

// PID execution code
void pid_run(float dt) {
    // ... PID calculation ...
    
    pid_time += dt;
    
    // If outside error band, update settling time
    if (!(pid.measure > pid.target * 0.98f && pid.measure < pid.target * 1.02f)) {
        settling_time = pid_time;
    }
}

// PID stop code
void pid_stop() {
    // Serial output settling time
    printf("SETTLING:%.4f\n", settling_time);
    
    // Reset variables
    settling_time = 0;
    pid_time = 0;
}
```

> Error band range can be adjusted as needed (default 2%, can be modified in code)

### 4. Steady-State Error

Average error during last 10% of response time.

**C Language Example**:
```c
// Global variables
float sse = 0;
float evaluate_time = 5.0f;              // Evaluation time (seconds)
float start_cal_sse_time = evaluate_time * 0.9f;  // Start time for SSE calculation
float sse_timer = 0;
int16_t cal_sse_tick = 0;                // Number of PID calculations during SSE period

// PID execution code
void pid_run(float dt) {
    // ... PID calculation ...
    
    sse_timer += dt;
    
    if (sse_timer > start_cal_sse_time) {
        sse += fabsf(pid.error);
        cal_sse_tick++;
    }
}

// PID stop code
void pid_stop() {
    // Calculate average steady-state error
    if (cal_sse_tick > 0) {
        sse = sse / cal_sse_tick;
    }
    
    // Serial output steady-state error
    printf("SSE:%.4f\n", sse);
    
    // Reset variables
    sse = 0;
    sse_timer = 0;
    cal_sse_tick = 0;
}
```

> Other steady-state error calculation methods can also be used

## PSO Algorithm Parameter Description

| Parameter | Description | Default | Recommended Range |
|-----------|-------------|---------|-------------------|
| `pop_size` | Particle swarm size | 10 | 10-15 |
| `max_iter` | Maximum iterations | 20 | 20-30 |
| `inertia_weight` | Inertia weight | 0.7298 | 0.4-0.9 |
| `c1` | Individual learning factor | 1.49618 | 1.0-2.0 |
| `c2` | Social learning factor | 1.49618 | 1.0-2.0 |
| `evaluation_delay` | Evaluation delay (seconds) | 0.5 | 0.1-1.0 |
| `max_evaluation_time` | Maximum evaluation time (seconds) | 5.0 | 3.0-10.0 |

## Logging and Result Saving

- **Log files**: Automatically saved to `logs/` directory, format: `{module_name}-{timestamp}.log`
- **Optimization results**: Can be saved as JSON format, including optimal parameters, fitness history, and configuration

**Result file example**:
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

## Important Notes

- Ensure stable communication to avoid data loss or format errors
- When embedded system calculates performance metrics, time reference must match PC definition (especially Mode 3)
- Adjust PSO algorithm parameters to balance convergence speed and accuracy
- For first run, use Mode 1 to verify data format and algorithm flow
- In manual input mode, add `.0` suffix when entering floating-point numbers (e.g., `1.0` not `1`)
- If serial port cannot open, check:
  - Device connection
  - Correct port name (Windows: COM1, Linux: /dev/ttyUSB0)
  - Baud rate matching
  - Required drivers installed
  - Other programs occupying the port

## Project Structure

```
PIDHelper/
├── One_click_Start.py          # Main startup program
├── demo.py                     # Simulation examples
├── PID_Helper/
│   ├── __init__.py
│   ├── PID_Helper.py          # PSO optimizer core
│   ├── CommunicationInterface.py  # Communication interface
│   └── Log.py                 # Logging module
├── logs/                      # Log file directory
└── README.md                  # This document
```

## License

Open-source project, licensed under [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0).