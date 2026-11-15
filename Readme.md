# Self-Balancing Robot: Dynamic Modeling and PID Control

## Overview

This project focuses on the mathematical modeling, dynamic analysis, simulation, and practical implementation of a two-wheeled self-balancing robot.

The main objective is to develop a mathematical representation of the robot's dynamics, investigate its stability, and implement a PID-based feedback controller to maintain the robot in an upright position.

The project combines theoretical control systems concepts with practical hardware implementation using an Arduino-based control system.

## Project Objectives

- Develop a mathematical model of a two-wheeled self-balancing robot.
- Derive the equations of motion based on mechanical principles.
- Obtain a state-space representation of the system.
- Analyze the open-loop stability of the robot.
- Design and tune a PID controller for balancing control.
- Implement the control algorithm using an Arduino microcontroller.
- Measure the robot's inclination using an MPU6050 IMU sensor.
- Drive the DC motors using a TB6612FNG motor driver.
- Evaluate the system's balancing performance.

## System Modeling

A two-wheeled self-balancing robot is an inherently unstable system that requires continuous feedback control to maintain its upright equilibrium position.

Its dynamic behavior depends on several physical parameters:

- Robot body mass
- Wheel mass
- Distance between the wheel axis and center of mass
- Moment of inertia
- Gravitational acceleration
- Motor characteristics

The mathematical model is developed using the mechanical dynamics of the robot.

### State-Space Representation

The system can be represented in state-space form as:

$$
\dot{x}(t)=Ax(t)+Bu(t)
$$

$$
y(t)=Cx(t)+Du(t)
$$

Where:

- $x(t)$ represents the state vector.
- $u(t)$ represents the control input.
- $y(t)$ represents the system output.
- $A$, $B$, $C$, and $D$ are the system matrices.

The state variables describe the robot's position, velocity, body inclination angle, and angular velocity.

## PID Controller Design

A PID controller is used to regulate the robot's body inclination and maintain its upright equilibrium position.

The control error is defined as:

$$
e(t)=\theta_{ref}(t)-\theta(t)
$$

The PID control law is:

$$
u(t)=K_p e(t)+K_i\int_0^t e(\tau)d\tau+K_d\frac{de(t)}{dt}
$$

Where:

- $K_p$ is the proportional gain.
- $K_i$ is the integral gain.
- $K_d$ is the derivative gain.
- $e(t)$ is the control error.
- $u(t)$ is the controller output.

The controller parameters are tuned to achieve stable balancing behavior and improve the transient response.

## Hardware Implementation

The physical prototype is implemented using the following components:

| Component | Description |
|---|---|
| Arduino Nano | Main microcontroller |
| MPU6050 (GY-521) | Inertial measurement sensor |
| TB6612FNG | Dual DC motor driver |
| DC Motors | Robot actuation |
| Wheels and Chassis | Mechanical structure |
| Battery Pack | Power supply |

The MPU6050 provides acceleration and angular velocity measurements used to estimate the robot's inclination.

The Arduino Nano processes the sensor measurements and executes the PID control algorithm.

The TB6612FNG motor driver receives the control signals and drives the DC motors to correct the robot's inclination.

## Technical Documentation

Detailed technical documentation is provided in the `Documentation` directory.

| Document | Description |
|---|---|
| [Mathematical Model](Documentation/Mathematical_Model.pdf) | Mathematical representation and dynamic modeling of the robot. |
| [Derivation](Documentation/Derivation.pdf) | Derivation of the governing equations and mathematical relationships. |
| [Controller Analysis](Documentation/Controller_Analysis.pdf) | Analysis of the control system and controller performance. |
| [Technical Report](Documentation/Technical_Report.pdf) | Comprehensive technical report covering the project. |

## Control System Performance

The performance of the balancing controller can be evaluated using the following indicators:

- Rise Time
- Settling Time
- Maximum Overshoot
- Steady-State Error
- Stability
- Disturbance Rejection
- Balancing Performance

## Tools and Technologies

- MATLAB
- Arduino IDE
- C/C++
- Classical Control Theory
- Dynamic System Modeling
- State-Space Representation
- PID Control
- Feedback Control Systems
- MPU6050 Sensor Integration
- Embedded Systems

## Future Improvements

- Implementing advanced control strategies such as LQR.
- Exploring state estimation using Kalman filtering.
- Investigating nonlinear control methods.
- Improving sensor fusion and angle estimation.
- Studying the effects of parameter uncertainty.
- Implementing more advanced motor control techniques.

## Author

**Amirali Mehrzad**

B.Sc. Electrical Engineering Student  
Iran University of Science and Technology (IUST)

## License

This project is licensed under the MIT License.