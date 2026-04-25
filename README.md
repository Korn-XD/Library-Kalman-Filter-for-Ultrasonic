# HC-SR04 and Kalman Filter (1D/2D) Library for STM32

This repository contains the custom C libraries developed for **FRA233 Lab 3: Estimation**. It implements the reading of the HC-SR04 Ultrasonic sensor using Hardware Timers (Input Capture) and two variants of Discrete Kalman Filters (1-State Static and 2-State Kinematic) utilizing the CMSIS-DSP matrix library.

---

## 📋 Criteria Verification Checklist

This library is designed specifically to meet the evaluation criteria specified in the laboratory guidelines:

- ✅ **Criteria P2-B (HC-SR04 Library):** 
  - File separated cleanly (`hcsr04.h`, `hcsr04.c`).
  - Employs Timer Input Capture instead of simple delay polling.
  - Implements standard physics equation $s = T_{echo} \times \frac{v_{sound}}{2}$.
- ✅ **Criteria P2-C (Discrete KF 1-State):**
  - Custom 1D Kalman implementation.
  - Evaluates static state ($x = \text{position}$) considering process ($Q$) and measurement ($R$) noise.
- ✅ **Criteria P3 (Discrete KF 2-State):**
  - Extends implementation to 2 states ($x_1 = \text{position}, x_2 = \text{velocity}$).
  - Fully utilizes Matrix Operations via `arm_math.h` (CMSIS-DSP).
  - Handles matrix multiplication, transposition, and inversion for predictions and updates.

---

## 🛠️ 1. HC-SR04 Sensor Library

### Overview
This library reads distance using the HC-SR04 ultrasonic sensor. It uses a **Timer Input Capture** (TIM2 CH1) interrupt, avoiding blocking `HAL_Delay()` functions, making it perfect for real-time systems running RTOS or high-frequency loops.

### How to Use
1. Configure `CubeMX`:
   - Set **TIM2 Channel 1** to **Input Capture direct mode**.
   - Enable TIM2 global interrupt in NVIC.
   - Set a GPIO Output pin for the Trigger (e.g., PA9).
2. In your `main.c`:
   ```c
   #include "hcsr04.h"

   // Start the Input Capture in interrupt mode
   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

   while(1) {
       HCSR04_Trigger(); // Send 10us trigger pulse
       HAL_Delay(50);    // Wait for echo (20Hz Update Rate)
       
       // The variable `raw_distance_m` is automatically updated via interrupts.
       printf("Raw Distance: %.4f m\r\n", raw_distance_m);
   }
   ```

### Mathematical Model
The distance is calculated directly inside the `HAL_TIM_IC_CaptureCallback` using:
$$ s = \frac{T_{echo} \times v_{sound}}{2} $$
Where $v_{sound}$ is approx $343.2 \text{ m/s}$ ($0.0003432 \text{ m/us}$).

---

## 🧮 2. Kalman Filter 1-State (Static Model)

### Overview
Used to filter the noisy raw data assuming the object is mostly stationary or moving very slowly.

### Mathematical Model
- **State ($x$):** Position
- **State Transition ($F$):** $1$ (Position stays the same)
- **Observation ($H$):** $1$ (We directly measure position)
- **Equations:**
  - Predict: $P = P + Q$
  - Update: $K = \frac{P}{P + R}$
  - Estimate: $x = x + K(z - x)$
  - Covariance Update: $P = (1 - K)P$

### How to Use
```c
#include "kalman.h"

Kalman1D_t kf1d;
float filtered_distance;

// Init: Q=1e-5, R=0.00292, Initial P=1.0, Initial Value=0.0
Kalman1D_Init(&kf1d, 0.00001f, 0.00292f, 1.0f, 0.0f);

while(1) {
    // ... get new raw measurement ...
    filtered_distance = Kalman1D_Update(&kf1d, raw_distance_m);
}
```

---

## 🚀 3. Kalman Filter 2-State (Kinematic Model)

### Overview
Used for dynamic objects. By estimating both Position and Velocity, the filter can anticipate where the object will be, heavily reducing the "lag" typically seen in heavy 1D filtering.

### Mathematical Model

**State Vector ($x$):**
$$
x = \begin{bmatrix} p \\ v \end{bmatrix}
$$

**State Transition Matrix ($F$):**
$$
F = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}
$$

**Observation Matrix ($H$):**
$$
H = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

**Process Noise Covariance ($Q$):**
$$
Q = \begin{bmatrix} q_{pos} & 0 \\ 0 & q_{vel} \end{bmatrix}
$$

**Measurement Noise Covariance ($R$):**
$$
R = \begin{bmatrix} r_{pos} \end{bmatrix}
$$

This utilizes the full matrix equations provided by CMSIS-DSP:

**Predict:**
$$
\hat{x}_{k|k-1} = F \hat{x}_{k-1|k-1}
$$
$$
P_{k|k-1} = F P_{k-1|k-1} F^T + Q
$$

**Update:**
$$
K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})
$$
$$
P_{k|k} = (I - K_k H) P_{k|k-1}
$$

### How to Use
```c
#include "kalman.h"

Kalman2D_t kf2d;
float pos, vel;

// Init: dt=0.05s (20Hz), q_pos=0.0005, q_vel=0.01, r_pos=0.005
Kalman2D_Init_Kinematic(&kf2d, 0.05f, 0.0005f, 0.01f, 0.005f);

while(1) {
    // ... get new raw measurement ...
    Kalman2D_Update(&kf2d, raw_distance_m);
    
    pos = kf2d.x_data[0]; // Estimated Position
    vel = kf2d.x_data[1]; // Estimated Velocity
}
```

---
*Generated according to FRA233 Lab 3 specifications.*