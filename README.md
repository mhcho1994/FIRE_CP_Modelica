# Model Notes & Parameter Update Guide
## Low-Fidelity Rover Model (RoverLowFidelity)

### 1. Overview

**Goal:**  
Fast **low-fidelity kinematic model** of a rover for early-stage algorithm development and vulnerability analysis.

**Key features:**
- Models planar kinematics only — **no ground friction or actuator dynamics**.  
- Uses a simplified **bicycle-like steering geometry**.  
- Includes **rollover detection (kinematic-based)** and basic **sensor module (sampling)**.  
- Designed for **real-time or large-scale fuzzing simulations**.
- Includes **ultrasonic injection attack model**.

**Assumptions:**
- Planar motion (`z=0` fixed, `der(z)=0`).  
- Direct throttle input: forward velocity commanded by `D`.  
- No wheel slip or suspension.  
- Static rollover condition (no transient load transfer).

---

### 2. Model Inputs, States, and Frames

#### Inputs
| Variable | Type | Description |
|-----------|------|-------------|
| `D` | discrete | Throttle command (normalized 0–1) |
| `delta_cmd` | discrete | Steering angle command [rad] |

---

#### States
| Category | Variables | Description |
|-----------|------------|-------------|
| **Position** | `x`, `y`, `z` | Position in inertial coordinates (m) |
| **Velocity** | `vx`, `vy`, `vz` | Body-frame linear velocity (m/s) |
| **Acceleration** | `ax`, `ay`, `az` | Body-frame acceleration (m/s²) |
| **Attitude** | `phi`, `theta`, `psi` | Roll, pitch, yaw (rad) |
| **Angular rates** | `p`, `q`, `r` | Body-frame angular velocity (rad/s) |
| **IMU/Mag** | `specific_g[3]`, `mx`, `my`, `mz` | Gravity projection & magnetometer readings |
| **Auxiliary** | `turn_radius`, `rollover_detected` | Turning curvature and rollover flag |

---

#### Frames

- **Inertial:** flat ENU plane with origin $(0,0,0)$, where the x-axis points East and y-axis points North.  
- **Body:** attached to the rover’s center of gravity, with x-axis forward and y-axis lateral, and heading `psi` measured counterclockwise from the inertial x-axis (East) to the body x-axis.  
- **Transform:** coordinate transformation from inertial to body using Euler angles $(\phi, \theta, \psi)$. For planar motion, roll and pitch are constrained $(\phi=\theta=0)$; the rotation reduces to a yaw rotation about the vertical axis.

---

### 3. Core Kinematics

The commands inputs are converted into velocity and steering angle as follows:
``` math
v = v_{\max} \cdot D,\quad \delta = \delta_{cmd}.
```

Then, the planar kinematics of rover chassis is given by

``` math
\begin{aligned}
\dot{x} &= v\cos\psi, \\
\dot{y} &= v\sin\psi, \\
\dot{\psi} &= \frac{v}{l_{\text{total}}}\tan\delta
\end{aligned}
```

with lateral acceleration

``` math
a_y = \frac{v^2}{l_{\text{total}}}\tan(-\delta).
```

Note that a positive steering angle $(\delta > 0)$ corresponds to a left turn in the rover forward direction, resulting in a positive yaw rate $(\dot{\psi} > 0)$. 

---

### 4. Geometry and Derived Quantities

| Symbol | Description |
|---------|--------------|
| `track_width` | Distance between left and right wheels (m) |
| `l_total` | Distance between rear and front axles (m) |
| `cg_height` | Height of center of gravity (m) |
| `phi_f` | Incline angle from tire contact to CG |
| `length_to_tire` | Distance CG → tire contact point |

---

### 5. Rollover Detection Logic

**Condition:**
Assuming flat terrain, the rover rolls over when the centrifugal force acting on CG exceeds the horizontal component of gravity in the body frame:
``` math
g \cos(\phi_f) < \frac{v^2}{R} \sin(\phi_f)
```

where the turning radius is given by
``` math
R = \frac{l_{\text{total}}}{\tan\vert\delta\vert}.
```

If inequality is satisfied, rollover flag is triggered (`rollover_detected = 1` in the Modelica implementation).

This is a **quasi-static** check for rollover risk — useful for identifying unsafe maneuvers or speed thresholds at low computational cost.

---

### 6. Sensor Modeling

- **Accelerometer:**  
  Projects specific forces into body frame and store in  `specific_g`
``` math
  \mathbf{f}_b = C_{n}^{b} \begin{bmatrix}0\\0\\g\end{bmatrix}.
``` 

- **Magnetometer:**  
  Computes the local Earth magnetic field using the reference latitude `lat0` and transforms it into the body (sensor) frame using the vehicle attitude $(\phi, \theta, \psi)$
  ``` math
    \mathbf{m}_b = C_{n}^{b} \begin{bmatrix}m_{e,x}\\m_{e,y}\\m_{e,z}\end{bmatrix}
  ```  
  where $C_{n}^{b}$ is the direction cosine matrix (DCM) from inertial to body frame.

- **Gyroscope:** 
  The nominal gyroscope measures the true angular velocity of the rover body expressed in the body frame:
  ``` math
    \mathbf{\omega}_b =\begin{bmatrix}p\\q\\r\end{bmatrix}.
  ``` 
  The acoustic injection attack introduces a periodic disturbance that couples into the MEMS gyro through the mechanical resonance of its proof mass. Because the vehicle’s heave motion is neglected, the acoustic excitation primarily affects the yaw-rate channel. 
  The corrupted measurement is modeled as
  ``` math
    \tilde{\mathbf{\omega}}_b = \mathbf{\omega}_b+(A_{atk}cos(f_{atk}t+\phi_{atk}) \\ +B_{atk}cos(\psi_{atk}))\mathbf{u}_{atk}.
  ``` 
  where 
  - $A_{atk}$, $B_{atk}$: attack magnitudes derived from the acoustic coupling model [rad/s],
  - $f_{atk}$ acoustic excitation frequency determined by the transducer frequency and the natural (driving) frequency of the MEMS proof mass [Hz],
  - $\phi_{atk}$, $\psi_{atk}$: phase differences determined by the attack model [rad],
  - $\mathbf{u}_{atk}$: unit vector defining the sensitive axis (coupling direction).
  
  This signal emulates the resonance–induced bias oscillation observed in MEMS gyroscopes when exposed to high-SPL narrow-band sound (e.g., 20–26 kHz). The model captures conditions that can cause attitude drift or false angular-rate feedback in control loop.

---

### 7. Updating Parameters for a New Rover

#### A. Core Geometry (Required)
These geometric parameters can be updated by measuring the physical properties of the new rover.

| Parameter | Meaning | How to Measure/Estimate |
|------------|----------|-------------------------|
| `l_total` | Wheelbase (m) | Distance between axle centers |
| `track_width` | Track width (m) | L/R wheel spacing (use smaller if front/rear differ) |
| `cg_height` | CG height (m) | From spec or tilt test |
| `phi_t` | Terrain slope (rad) | 0 for flat ground (use if needed, default 0) |
| `v_max` | Max forward speed (m/s) | From test logs or spec |

---

#### B. Sign Convention & Frame Consistency

Check that **positive `delta` corresponds to positive yaw rate** (`ψ̇ > 0`).  
If not, flip the sign of `delta` globally.
- Positive steering angle ($\delta > 0$) produces a positive yaw rate ($\dot{\psi} > 0$), corresponding to a left turn in the rover’s forward direction.
- The rover operates on a flat plane under an ENU-like convention:
  $x$ (East), $y$ (North), $psi = 0$  &rarr; facing East
- The IMU measures specific force, i.e., non-gravitational acceleration. The accelerometer senses an upward acceleration equal in magnitude to gravity, $\mathbf{f}_b = \begin{bmatrix}0, 0, g \end{bmatrix}^{T}$ when stationary on flat ground. When the rover is in free fall, there is no contact reaction, and thus the accelerometer measures zero: $\mathbf{f}_b = \mathbf{0}$.
---

#### C. Units & Frame Consistency

- Length [$m$] 
- Velocity [$m/s$]
- Acceleration [$m/s^2$]
- Angle [$rad$]
- Angular rate [$rad/s$]

---

#### D. Quick Validation Checklist

| Test | Expectation |
|------|--------------|
| Straight line <br> ($\delta_{cmd}=0$) | $\dot{\psi}=0$, $a_{y}=0$, $x=\int{v\,dt}$ |
| Constant turn  ($D \neq 0$, $\delta_{cmd}=c$) | $R = {l_{\text{total}}}/{\tan\vert\delta\vert}$, $\dot{\psi}=v/R$, <br> $a_{y}=v^{2}/R$ |
| Rollover threshold | `rollover_detected` toggles near  $ v_{crit} ≈ \sqrt{g R tan(φ_f)} $ |

---

<br/><br/><br/>

## High-Fidelity Rover Model (RoverHighFidelity)

### 1. Overview

**Goal:**  
High-fidelity rover **dynamics** model for precise controller evaluation, vulnerability analysis.

**Key features:**
- **Rigid-body chassis** (yaw & roll DOFs in addition to the Lo-Fi model; planar heave locked).
- **Four wheels** with **Dugoff tire** forces and **relaxation lengths** (longitudinal & lateral).
- **Motor & drivetrain**: BLDC `dq` (lumped) with back-EMF, torque constant, resistance, inductance, rotor inertia, viscous loss, **gearbox** to wheels.
- **Steering servo** first-order dynamics.
- **Load transfer** and **roll stiffness/damping** (normal forces per wheel).
- **Aerodynamics** (placeholder coefficient).
- **Magnetometer + EMI**: field composition from attitude + wire-induced perturbation.

**Assumptions:**
- Pitch and heave dynamics are suppressed (`thetȧ=0`, `ż=0`).   
- No wheel slip or suspension.  
- Static rollover condition (no transient load transfer).

### 2. Model Inputs, States, and Frames

#### Inputs
| Variable | Type | Description |
|-----------|------|-------------|
| `D` | discrete | Throttle command (normalized 0–1) |
| `delta_cmd` | discrete | Steering angle command [rad] |

---

#### States
| Category | Variables | Description |
|-----------|------------|-------------|
| **Position** | `x`, `y`, `z` | Position in inertial coordinates (m) |
| **Velocity** | `vx`, `vy`, `vz` | Body-frame linear velocity (m/s) |
| **Acceleration** | `ax`, `ay`, `az` | Body-frame acceleration (m/s²) |
| **Attitude** | `phi`, `theta`, `psi` | Roll, pitch, yaw (rad) |
| **Angular rates** | `p`, `q`, `r` | Body-frame angular velocity (rad/s) |
| **IMU/Mag** | `specific_g[3]`, `mx`, `my`, `mz` | Gravity projection & magnetometer readings |
| **Auxiliary** | `turn_radius`, `rollover_detected` | Turning curvature and rollover flag |

---

#### Frames

- **Inertial:** flat ENU plane with origin $(0,0,0)$, where the x-axis points East and y-axis points North.  
- **Body:** attached to the rover’s center of gravity, with x-axis forward and y-axis lateral, and heading `psi` measured counterclockwise from the inertial x-axis (East) to the body x-axis.  
- **Transform:** coordinate transformation from inertial to body using Euler angles $(\phi, \theta, \psi)$. For planar motion, roll and pitch are constrained $(\phi=\theta=0)$; the rotation reduces to a yaw rotation about the vertical axis.