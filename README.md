# Model Notes & Parameter Update Guide
## RoverLowFidelity

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

- **Inertial:** flat plane with origin $(0,0,0)$, typically representing the Earth-fixed frame (NED or ENU convention depending on context).  
- **Body:** aligned with rover’s longitudinal axis that has heading `psi` with respect to intertial coordinates.  
- **Transform:**
  ```math
  C_{n2b} = \text{transpose}\big(eul2rot(\{\phi, \theta, \psi\_\text{mod}\})\big)
  ```
- Gravity projection: `specific_g[i] = C_n2b[i,3]*g`.  
- Planar translation uses ENU-like `ψ`, while IMU math uses NED convention — acceptable for Lo-Fi but should be documented.

---

### 3. Core Kinematics

Let  
\[
v = v_{\max} \cdot D,\quad \delta = \text{steer angle}
\]

Then:

\[
\begin{aligned}
\dot{x} &= v\cos\psi, \\
\dot{y} &= v\sin\psi, \\
\dot{\psi} &= \frac{v}{l_{\text{total}}}\tan\delta
\end{aligned}
\]

Lateral acceleration:

\[
a_y = \frac{v^2}{l_{\text{total}}}\tan(-\delta)
\]

---

## 4. Geometry and Derived Quantities

| Symbol | Description |
|---------|--------------|
| `track_width` | Distance between left and right wheels (m) |
| `cg_height` | Height of center of gravity (m) |
| `phi_f = atan2(cg_height, 0.5*track_width)` | Incline angle from tire contact to CG |
| `length_to_tire = sqrt(cg_height² + (0.5*track_width)²)` | Distance CG → tire contact point |

---

## 5. Rollover Detection Logic

**Condition:**
\[
g \cos(\phi_f + \phi_t) > \frac{v^2}{R} \sin(\phi_f)
\]

where  
\[
R = \frac{l_{\text{total}}}{\tan|\delta|}
\]

If inequality flips, rollover is detected (`rollover_detected = 1`).

This is a **quasi-static** check — useful for identifying unsafe maneuvers at low simulation cost.

---

## 6. IMU and Magnetometer Modeling

- **Accelerometer:**  
  Projects gravity into body frame:
  \[
  \mathbf{g}_b = C_{n2b} \begin{bmatrix}0\\0\\g\end{bmatrix}
  \]
  Stored in `specific_g`.

- **Magnetometer:**  
  Uses current attitude (`φ, θ, ψ`) to compute Earth field vector.  
  Output:
  ```modelica
  mx = mag.b_earth[1];
  my = mag.b_earth[2];
  mz = mag.b_earth[3];
  ```

---

## 8. Updating Parameters for a New Rover

### A. Core Geometry (Required)

| Parameter | Meaning | How to Measure/Estimate |
|------------|----------|-------------------------|
| `l_total` | Wheelbase (m) | Distance between axle centers |
| `track_width` | Track width (m) | L/R wheel spacing (use smaller if front/rear differ) |
| `cg_height` | CG height (m) | From spec or tilt test |
| `phi_t` | Terrain slope (rad) | 0 for flat ground |
| `v_max` | Max forward speed (m/s) | From test logs or spec |

**Tilt test (for CG height):**
\[
\tan\alpha^* \approx \frac{0.5 \; \text{track\_width}}{\text{cg\_height}} \Rightarrow \text{cg\_height} \approx \frac{0.5\,\text{track\_width}}{\tan\alpha^*}
\]

---

### B. Steering Sign Convention

Check that **positive `delta` corresponds to positive yaw rate** (`ψ̇ > 0`).  
If not, flip the sign of `delta` globally.

---

### C. Units & Frame Consistency

- Length → meters  
- Angle → radians  
- Gravity → m/s²  
- Flat plane yaw convention (ENU-like)  
- IMU uses NED for gravity projection

---

### D. Magnetometer Updates

If the IMU or location changes:
- Update **mounting rotation** (body ↔ sensor).  
- Adjust **hard-iron bias** and **soft-iron matrix**.  
- Set new **local declination/inclination** for magnetic field.

---

### E. Quick Validation Checklist

| Test | Expectation |
|------|--------------|
| Straight line (`δ=0`) | `ψ̇≈0`, `ay≈0`, `x≈∫v dt` |
| Constant turn (`δ≠0`) | `R≈l_total/tan|δ|`, `ψ̇≈v/R`, `ay≈v²/R` |
| Rollover threshold | `rollover_detected` toggles near  \[ v_crit ≈ √(g R cos(φ_f+φ_t) / sin(φ_f)) \] |

---

## 9. Implementation Options

### Option 1 — Inline Parameters
```modelica
parameter Real l_total     = 0.278;
parameter Real track_width = 0.234;
parameter Real cg_height   = 0.064;
parameter Real phi_t       = 0.0;
parameter Real v_max       = 15.0;
```

---

### Option 2 — Parameter Record (Recommended)
```modelica
package RoverParamsPkg
  record RoverParams
    parameter Real l_total;
    parameter Real track_width;
    parameter Real cg_height;
    parameter Real phi_t;
    parameter Real v_max;
  end RoverParams;
end RoverParamsPkg;

model RoverLowFidelity
  import RoverParamsPkg.*;
  parameter RoverParams P = RoverParams(
      l_total=0.278,
      track_width=0.234,
      cg_height=0.064,
      phi_t=0.0,
      v_max=15.0);

  Real phi_f = atan2(P.cg_height, 0.5*P.track_width);
equation
  der(psi) = (P.v_max*pre(D))/P.l_total * tan(delta);
end RoverLowFidelity;
```

Then define multiple configs:
```modelica
parameter RoverParamsPkg.RoverParams P_RoverA(
  l_total=0.300, track_width=0.260, cg_height=0.080, phi_t=0.0, v_max=12.0);

parameter RoverParamsPkg.RoverParams P_RoverB(
  l_total=0.275, track_width=0.240, cg_height=0.070, phi_t=0.05, v_max=14.0);
```

---

### Option 3 — Scripted Setup
Use `.mos` or FMU runtime scripts to set parameters externally so the model file remains immutable.

---

## 10. Common Pitfalls & Tips

- **Small `δ` values:**  
  Replace event-based radius update with smooth expression if needed:
  \[
  R = \frac{l_{\text{total}}}{\tan\delta + \epsilon\,\text{sign}(\delta)},\quad \epsilon \ll 1
  \]

- **Yaw wrap:**  
  `mod(ψ + π, 2π) − π` ensures continuous yaw wrapping.

- **Throttle realism:**  
  Add a first-order lag on `thr` for smooth transitions:
  \[
  \dot{v} = (v_{\text{cmd}} - v)/τ
  \]

- **Frame note:**  
  Always document that translation uses ENU plane while IMU uses NED rotation.

---

## 11. “New Rover” Quick Checklist

- [ ] Measure `l_total` (wheelbase, m)  
- [ ] Measure `track_width` (m)  
- [ ] Estimate `cg_height` (m)  
- [ ] Set `phi_t` (terrain slope, rad)  
- [ ] Set `v_max` (m/s)  
- [ ] Verify steering sign convention  
- [ ] Update magnetometer alignment (if needed)  
- [ ] Run validation tests (straight line, constant turn, rollover)

---

**End of Document**  
*(Prepared for CP-Glimpse Lo-Fi rover simulation framework)*
