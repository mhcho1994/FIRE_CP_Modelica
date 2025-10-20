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
R = \frac{l_{\text{total}}}{\tan|\delta|}.
```

If inequality is satisfied, rollover flag is triggered (`rollover_detected = 1` in the Modelica implementation).

This is a **quasi-static** check for rollover risk — useful for identifying unsafe maneuvers or speed thresholds at low computational cost.

---

### 6. Sensor Modeling

- **Accelerometer:**  
  Projects gravity into body frame and store in  `specific_g`
 ``` math
  \mathbf{g}_b = C_{n}^{b} \begin{bmatrix}0\\0\\g\end{bmatrix}.
 ``` 

- **Magnetometer:**  
  Uses current latitude attitude (`φ, θ, ψ`) to compute Earth field vector.  
 ``` math
  \mathbf{g}_b = C_{n}^{b} \begin{bmatrix}0\\0\\g\end{bmatrix}.
 ``` 

---

### 7. Updating Parameters for a New Rover

#### A. Core Geometry (Required)
These geometric parameters can be updated by measuring the physical properties of the new rover.

| Parameter | Meaning | How to Measure/Estimate |
|------------|----------|-------------------------|
| `l_total` | Wheelbase (m) | Distance between axle centers |
| `track_width` | Track width (m) | L/R wheel spacing (use smaller if front/rear differ) |
| `cg_height` | CG height (m) | From spec or tilt test |
| `phi_t` | Terrain slope (rad) | 0 for flat ground (use if needed) |
| `v_max` | Max forward speed (m/s) | From test logs or spec |

**Tilt test (for CG height):**
 ``` math
\tan\alpha^* \approx \frac{0.5 \; \text{track\_width}}{\text{cg\_height}} \Rightarrow \text{cg\_height} \approx \frac{0.5\,\text{track\_width}}{\tan\alpha^*}
```

---

#### B. Steering Sign Convention

Check that **positive `delta` corresponds to positive yaw rate** (`ψ̇ > 0`).  
If not, flip the sign of `delta` globally.

---

#### C. Units & Frame Consistency

- Length → meters  
- Angle → radians  
- Gravity → m/s²  
- Flat plane yaw convention (ENU-like)  
- IMU uses NED for gravity projection

---

#### D. Magnetometer Updates

If the IMU or location changes:
- Update **mounting rotation** (body ↔ sensor).  
- Adjust **hard-iron bias** and **soft-iron matrix**.  
- Set new **local declination/inclination** for magnetic field.

---

#### E. Quick Validation Checklist

| Test | Expectation |
|------|--------------|
| Straight line (`δ=0`) | `ψ̇≈0`, `ay≈0`, `x≈∫v dt` |
| Constant turn (`δ≠0`) | `R≈l_total/tan|δ|`, `ψ̇≈v/R`, `ay≈v²/R` |
| Rollover threshold | `rollover_detected` toggles near  \[ v_crit ≈ √(g R cos(φ_f+φ_t) / sin(φ_f)) \] |

---

<br/><br/><br/>

## High-Fidelity Rover Model (RoverHighFidelity)

### 1. Overview

**Goal:**  
Fast **low-fidelity kinematic model** of a rover for early-stage algorithm development and vulnerability analysis.