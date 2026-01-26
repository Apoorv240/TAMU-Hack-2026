# TAMU Robotic Arm

A comprehensive multi-platform robotic arm control system integrating embedded firmware, real-time kinematics computation, and a web-based interface for intuitive operation and visualization.

## Project Overview

This project implements a complete robotic arm system with distributed processing across multiple microcontrollers and a central PC control station. The architecture enables precise motor control, forward/inverse kinematics calculations, real-time communication, and advanced vision-based hand stability analytics. A key focus is the collection and analysis of operator hand dynamics, including jerk tracking and load stability assessment, to optimize control safety and precision.

## System Architecture

The project is organized into three primary subsystems:

### Microcontroller Firmware

**ESP32 Components**
- `jointedarm`: Main articulated arm firmware
- `server`: Network communication server for remote commands
- UART testing utilities for component validation

### PC Control Station

A Python-based control platform providing:
- **Kinematics Module** (`arm/kinematics.py`): Forward and inverse kinematics calculations for arm trajectory planning
- **Vision System** (`vision/`): Multi-camera integration with advanced analytics
  - Real-time hand and load tracking across multiple camera feeds
  - Jerk calculation and operator tremor analysis for motion smoothness assessment
  - Stability metrics collection for load safety evaluation
  - Data aggregation pipeline for analytics and performance trending
- **Serial Communication** (`util/serial_monitor.py`): Non-blocking UART communication thread for microcontroller interaction
- **Web Interface** (`site/`): HTML-based control dashboard with real-time stability metrics visualization
- **Main Controller** (`main.py`): Multi-threaded central orchestrator managing concurrent vision processing, kinematics calculations, serial I/O, and analytics pipelines

## Technology Stack

- **Firmware**: C++ (ESP32)
- **Control Software**: Python 3
- **Kinematics**: Custom computational geometry algorithms
- **Communication**: UART serial protocols
- **Web Interface**: HTML, CSS, JavaScript
- **Vision Processing**: OpenCV-compatible camera libraries

## Key Features

- Real-time inverse kinematics for multi-joint trajectory planning
- Multi-threaded PC architecture supporting concurrent vision processing, kinematics computation, and serial communication
- Advanced hand stability tracking with jerk analysis and operator dynamics assessment
- Multi-camera vision system with real-time data collection and analytics pipeline
- Quantitative stability metrics for load bearing and operator control evaluation
- Networked command interface via ESP32 server
- Web-based control dashboard with real-time analytics visualization
- UART-based hardware communication layer
- Modular architecture enabling independent component testing

## Project Structure

```
TAMU-Hack-2026/
├── esp32/                 # ESP32 firmware components
│   ├── jointedarm/        # Main arm controller
│   ├── server/            # Network communication server
│   └── uart_test/         # UART validation utilities
├── pc/                    # Python control station
│   ├── arm/               # Kinematics computation
│   ├── vision/            # Camera and stability analysis
│   ├── site/              # Web interface
│   ├── util/              # Serial communication utilities
│   └── main.py            # Central controller
└── stm32/                 # Legacy STM32 motor driver (deprecated)
```

## Dependencies

- Python 3.8+
- GCC for embedded compilation
- Standard C++ libraries with C++17 support
- OpenCV (for vision processing)
- Flask or equivalent web framework (for site module)

## Usage

The system operates through the central PC control station (`pc/main.py`), which employs a multi-threaded architecture to manage concurrent operations:
1. **Serial Communication Thread**: Non-blocking UART communication with microcontrollers for motor commands and telemetry
2. **Vision Processing Thread**: Parallel camera feed acquisition and analysis from multiple sensors
3. **Analytics Thread**: Real-time jerk calculation, stability metric computation, and data collection for hand tracking
4. **Kinematics Thread**: Parallel inverse kinematics calculations for arm positioning
5. **Web Service Thread**: Asynchronous handling of dashboard requests and metrics updates

This multi-threaded design ensures that vision analytics and stability tracking operate independently of motor control latency, enabling responsive hand stability monitoring. Remote operation is available through the ESP32 server component, enabling networked command submission to the robotic arm.

## Communication Protocols

- **UART**: Primary communication channel between PC and ESP32 devices
- **Network**: ESP32 server enables remote command interface
- **Web**: HTML interface communicates with PC control station for visualization and command submission

## Data Processing and Algorithms

The hand stability tracking system employs multiple signal processing techniques to ensure reliable motion metrics despite noisy camera input:

### Exponential Smoothing

Camera-tracked hand positions are preprocessed using exponential smoothing with a configurable smoothing factor (alpha = 0.6) to attenuate pixel-level jitter and sensor noise while preserving meaningful motion features. This approach is applied independently to x and y coordinates before derivative computation.

### Derivative Step Optimization

To reduce noise amplification in velocity, acceleration, and jerk calculations, derivatives are computed using a step size greater than unity (typically step = 2 or 3 frames) rather than frame-by-frame differences. This increases the effective time interval for finite-difference approximation, reducing sensitivity to high-frequency noise while maintaining responsiveness to true operator motion.

### Jerk and Stability Metrics

From smoothed position data, the system computes:
- **Velocity**: Magnitude of position change per unit time, computed across step-sized intervals
- **Acceleration**: Rate of change of velocity, derived from smoothed velocity sequences
- **Jerk (RMS)**: Root-mean-square of acceleration derivatives, quantifying motion smoothness; lower jerk indicates more controlled, deliberate movement
- **Path Length**: Cumulative Euclidean distance traveled
- **Speed Statistics**: Mean velocity and standard deviation across the tracking window

These metrics are computed over a sliding 5-second window, enabling real-time assessment of operator control quality and load stability.

## Kinematics: Mathematical Derivation

The robotic arm employs a two-link planar kinematic chain with link lengths $L_1$ and $L_2$, controlled by shoulder and elbow joint angles $\theta_1$ and $\theta_2$.

### Inverse Kinematics

Given a desired end-effector position $(x, y)$ in Cartesian space, we solve for joint angles using the following derivation:

**Step 1: Elbow Angle Calculation**

The distance from the base to the target is:
$$r = \sqrt{x^2 + y^2}$$

Using the law of cosines on the triangle formed by $L_1$, $L_2$, and $r$:
$$r^2 = L_1^2 + L_2^2 + 2L_1L_2\cos(\theta_2)$$

Solving for $\cos(\theta_2)$:
$$\cos(\theta_2) = \frac{r^2 - L_1^2 - L_2^2}{2L_1L_2}$$

We constrain this value to $[-1, 1]$ to ensure physical validity. The elbow angle is then:
$$\theta_2 = \text{atan2}(-\sqrt{1 - \cos^2(\theta_2)}, \cos(\theta_2))$$

The negative square root selects the elbow-down configuration (one of two possible solutions).

**Step 2: Shoulder Angle Calculation**

The angle to the target from the base is:
$$\phi = \text{atan2}(y, x)$$

The combined link vector in the end-effector frame is:
$$k_1 = L_1 + L_2\cos(\theta_2), \quad k_2 = L_2\sin(\theta_2)$$

The shoulder angle is then:
$$\theta_1 = \phi - \text{atan2}(k_2, k_1)$$

**Workspace Validation**

The inverse kinematics solution exists only when:
$$(L_1 - L_2)^2 \leq r^2 \leq (L_1 + L_2)^2$$

Points outside this range are unreachable and return zero angles.

## Coordinate Transformations

The system performs multiple coordinate transformations to map hand positions from camera pixel coordinates to arm control space.

### Camera to Normalized Arm Coordinates

Camera frames capture hand position in pixel coordinates with origin at the top-left corner. The transformation to normalized arm coordinates accounts for:

1. **Vertical Flip and Centering**: Convert image coordinates to a centered coordinate system

```
y' = -(y - SCREEN_HEIGHT/2) + SCREEN_HEIGHT/2
```

where `SCREEN_HEIGHT = 500` pixels.

2. **Scaling and Offset**: Convert from pixel space to metric arm space using calibrated conversion factors

```
x_arm = x / WIDTH_CONV - x_offset
y_arm = y' / HEIGHT_CONV
```

where `WIDTH_CONV = 23` pixels/unit, `HEIGHT_CONV = 25` pixels/unit, and `x_offset = 15` units.

Combined transformation:

```
x_arm = x / 23 - 15
y_arm = (-(y - 250) + 250) / 25
```

### Arm Joint Angles to Pixel Coordinates

The inverse transformation renders the arm on the display by converting joint angles back to pixel coordinates for visualization.

**Forward Kinematics**: From joint angles, compute end-effector position

```
x_ee = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
y_ee = L1 * sin(θ1) + L2 * sin(θ1 + θ2)
```

**Pixel Mapping**: With base position at `(x_b, y_b) = (SCREEN_WIDTH/2, SCREEN_HEIGHT)` and accounting for the inverted y-axis in image coordinates:

Shoulder joint position:

```
x1 = x_b + L1_px * cos(θ1)
y1 = y_b - L1_px * sin(θ1)
```

End-effector position:

```
θ12 = θ1 + θ2
x2 = x1 + L2_px * cos(θ12)
y2 = y1 - L2_px * sin(θ12)
```

where `L1_px = L1 * 24` and `L2_px = L2 * 24` (pixels per unit link length).

## Challenges

During development, an STM32F446-based stepper motor driver was initially evaluated for direct motor control. However, the interrupt-based UART protocol implementation proved unreliable in practice, with packet loss occurring during high-frequency communication exchanges. This necessitated a pivot to the ESP32 as the primary microcontroller, which provided more robust handling of concurrent UART operations and allowed for more stable real-time control of the arm actuators.

## Development Notes

Each subsystem can be tested independently:
- Kinematics calculations can be verified separately from hardware
- Vision system and stability analytics operate independently for jerk tracking and hand dynamics analysis
- UART test utilities validate serial communication without full arm integration
- Web interface can be developed and tested without full arm integration
- Multi-threaded components use thread-safe queues for inter-process communication to ensure data consistency during concurrent operations


