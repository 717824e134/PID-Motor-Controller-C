# Smart Closed-Loop Motor Controller (Pre-Silicon Simulation)

## Overview
This project implements a discrete-time Proportional-Integral-Derivative (PID) controller written entirely in standard C. To validate the control logic without physical hardware, the system includes a "Digital Twin" mathematical model of a DC motor, allowing for real-time Hardware-in-the-Loop (HIL) style simulation directly in the browser.

The firmware is designed to track dynamic speed setpoints and features aggressive disturbance rejection to maintain stability under simulated physical loads.

## Live Simulation
**[Run the Simulation in Wokwi] (https://wokwi.com/projects/472516766638424065)**
*(Click the link above and hit the green "Play" button to view the serial monitor output in real-time).*

## System Architecture
The system is cleanly separated into hardware simulation and control logic, following standard embedded architecture boundaries:

*   **External Entities:** The primary entity is the simulated DC Motor, which acts as both the actuator (receiving power) and the sensor (reporting RPM). A secondary entity is the simulated "Load Disturbance," which injects physical friction into the system.
*   **Processes:** The core process is the discrete `compute_pid()` algorithm. It calculates the error between the target and actual speeds, applies the proportional, integral, and derivative gains, and computes the exact corrective output required.
*   **Data Flows:** Sensor data (Actual RPM) flows from the plant model into the PID process. Corrective control signals (PWM Output) flow back out to the motor model. 
*   **Data Stores:** The `PID_Controller` struct acts as the primary data store, retaining the tuning constants ($K_p$, $K_i$, $K_d$) in memory, alongside the running integral sum and previous error states required for time-step calculus.

## Key Technical Features
*   **Bare-Metal C:** No external control libraries used; the algorithm is written from scratch.
*   **Digital Twin Modeling:** The `simulate_motor()` function uses basic momentum and friction equations to mimic real silicon behavior.
*   **Disturbance Rejection:** The main loop injects a massive sudden load drop at the 5-second mark to test and validate the PID controller's recovery response.
*   **Output Clamping:** The algorithm includes hardware-safe limits to prevent impossible PWM power values.

## File Breakdown
*   `main.c`: Contains the Digital Twin physics model, the PID algorithm struct/functions, and the execution loop.
