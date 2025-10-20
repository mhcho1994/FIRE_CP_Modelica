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

**Accelerometer:**  
Projects specific forces into body frame and store in  `specific_g`
``` math
  \mathbf{f}_b = C_{n}^{b} \begin{bmatrix}0\\0\\g\end{bmatrix}.
``` 

**Magnetometer:**  
Computes the local Earth magnetic field using the reference latitude `lat0` and transforms it into the body (sensor) frame using the vehicle attitude $(\phi, \theta, \psi)$
``` math
  \mathbf{m}_b = C_{n}^{b} \begin{bmatrix}m_{e,x}\\m_{e,y}\\m_{e,z}\end{bmatrix}
```  
where $C_{n}^{b}$ is the direction cosine matrix (DCM) from inertial to body frame.

**Gyroscope:** 
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

## 2. Inputs, States, and Outputs

### Inputs (discrete, ZOH via `pre()`)

| Name | Units | Meaning |
|---|---|---|
| `D` | – | PWM duty (0–1) → motor voltage command (via dq transform & clip) |
| `delta_cmd` | rad | Steering command to servo |

`pre()` ensures event-safe, piecewise-constant inputs.

---

### States (selected)

**Chassis kinematics/dynamics**
- Position: `x, y, z` (m) with `der(z)=0`.
- Body-frame vels/accels: `vx, vy, vz`, `ax, ay, az`.
- Attitude & rates: `phi, theta, psi`, `p, q, r` with `der(theta)=0`, `der(psi)=r`.
- Roll DOF with stiffness/damping: `der(p) = (roll moments − k_rllsp*phi − c_rllsp*p)/I_xx`.

**Wheels**
- Angular speeds: `omega_fl, …, omega_rr` with `I_wheel`.
- Slip ratio `kappa_*` (longitudinal, with relaxation length `Lrelx`).
- Slip angle `alpha_*` (lateral, with relaxation length `Lrely`).
- Per-wheel velocities in tire coordinates `vx_*, vy_*`.

**Motor/drivetrain**
- dq voltage/current: `Vq, Iq`.
- Motor speed/angle: `omega, lambda`.
- Power: `Pmech = Kt_q*Iq*omega`, losses `Ploss = Iq^2 R_phi`.
- Gear constraint: `4*omega/gratio = sum(omega_wheels)`.

**Magnetometer/EMI**
- Earth field via attitude; EMI via motor current/angle (`emi.Iq`, `emi.lambda`).

**Events/flags**
- `turn_radius` update when `delta` changes.
- Rollover flag when any `Fz_*` ≤ threshold (`normal_lim` ~1.5 N in code).

---

## 3. Frames & Transforms

- Body ↔ inertial via Euler `{phi, theta, psi}`; yaw wraps as `mod(psi+π, 2π)−π`.
- Gravity projection to body:  
  \[
  \mathbf{g}_b = C_{n\to b}\,[0,0,g]^T,\quad C_{n\to b}=\text{transpose}(\text{eul2rot}(\phi,\theta,\psi_\text{mod}))
  \]
- Planar translation uses standard body–inertial projection (`ẋ = v_x\cos\psi - v_y\sin\psi`, etc.).

---

## 4. Subsystem Equations (bird’s-eye)

### 4.1 Motor & Drivetrain
- Voltage command (with safety clip):
  \[
  V_q = \sqrt{\tfrac{3}{2}}\,V_s\,\text{clip}\!\big(\text{pre}(D),\, V_{\min},\,1\big)
  \]
- Electrical dynamics:
  \[
  \dot I_q = \frac{V_q - R_\phi I_q - K_b \, \omega}{L_e}
  \]
- Rotor dynamics & kinematic constraint:
  \[
  \dot\omega = \frac{K_t I_q - b\,\omega - 4\,\text{thr}}{J},\quad 
  \frac{4}{\text{gratio}}\,\omega = \omega_{fl}+\omega_{fr}+\omega_{rl}+\omega_{rr}
  \]

> **Note:** `thr` is the torque sent to each wheel shaft (post-gear). In your code it’s solved implicitly by the constraint; keep this consistent when identifying `J, b, Kt_q, Kb_q, R_phi, Le, gratio`.

---

### 4.2 Steering Servo
First-order lag:
\[
\dot\delta = \tfrac{1}{\tau_\text{servo}}\big(\text{pre}(\delta_\text{cmd})-\delta\big)
\]

---

### 4.3 Tire Kinematics (per wheel)
Example for front-left (steered):
\[
\begin{aligned}
v_{x,fl} &= \cos\delta\,(v_x + \tfrac{t_w}{2}r) + \sin\delta\,(v_y + l_{front} r) \\
v_{y,fl} &= -\sin\delta\,(v_x + \tfrac{t_w}{2}r) + \cos\delta\,(v_y + l_{front} r)
\end{aligned}
\]

Rear wheels use fixed heading.

---

### 4.4 Slip Dynamics with Relaxation
Longitudinal (fl example):
\[
\dot\kappa_{fl} = -\frac{|v_x \pm \tfrac{t_w}{2}r|}{\text{clip}(\tanh|v_x|,\,0.001,1)\,L_{relx}}\kappa_{fl} + \frac{r_t \omega_{fl} - (v_x \pm \tfrac{t_w}{2}r)}{L_{relx}}
\]
Lateral (via `\alpha` with similar relaxation to `Lrely`).

Bounds:
\[
\kappa \in [\kappa_{min},\kappa_{max}],\quad \alpha \in [\alpha_{min},\alpha_{max}]
\]

---

### 4.5 Dugoff Combined Slip Forces
Define combined slip “speed”:
\[
v_s = v_x \sqrt{\kappa^2 + \tan^2\alpha}
\]
Friction scaling with speed:
\[
\mu = \text{clip}\big(\mu_0(1 - A_s v_s),\,0,\,\mu_0\big)
\]
Load transfer → normal forces `Fz_*` (includes roll stiffness/damping, longitudinal & lateral accelerations).

Dugoff factor:
\[
z = \text{clip}\!\left(\frac{\mu F_z (1-\kappa)}{2\sqrt{(c_\kappa \kappa)^2 + (c_\alpha \tan\alpha)^2 + \varepsilon}},\, z_{min}, z_{max}\right),\quad
f_z = \begin{cases}
z(2-z), & z<1\\
1, & z\ge 1
\end{cases}
\]
Forces:
\[
F_x = \frac{c_\kappa \kappa}{1-\kappa}\, f_z,\qquad
F_y = \frac{c_\alpha \tan\alpha}{1-\kappa}\, f_z
\]

---

### 4.6 Chassis Dynamics
Translational:
\[
\begin{aligned}
\dot x &= v_x\cos\psi - v_y\sin\psi,\quad \dot y = v_x\sin\psi + v_y\cos\psi \\
\dot v_x &= v_y r + a_x,\quad \dot v_y = -v_x r + a_y + \text{(yaw-inertia coupling)}
\end{aligned}
\]

Accelerations from wheel forces (steered vs fixed orientations accounted):
\[
a_x = \frac{1}{m}\sum_i (F_{x,i}\cos \gamma_i + F_{y,i}\sin \gamma_i),\quad
a_y = \frac{1}{m}\sum_i (-F_{x,i}\sin \gamma_i + F_{y,i}\cos \gamma_i)
\]
where \(\gamma_i\) is wheel steering angle (`δ` for front, `0` rear).

Yaw & roll:
\[
\begin{aligned}
\dot\psi &= r,\quad
\dot r = \frac{M_z(F_x,F_y)}{I_{zz}} \\
\dot\phi &= p,\quad
\dot p = \frac{-m_s a_y h_s \cos\phi + m_s g h_s \sin\phi - k_{rllsp}\phi - c_{rllsp} p}{I_{xx}}
\end{aligned}
\]

---

### 4.7 Rollover & Events
- **Turn radius (for logging):** \(R = l_{total}/\tan|\delta|\) (capped).
- **Rollover flag:** triggers when any `Fz_*` ≤ `normal_lim` (approximate wheel lift).

---

## 5. Parameters You Typically Identify (MATLAB SI)

Group parameters you’ll estimate from data (others from CAD/specs):

**Motor/Drivetrain**  
`R_phi, L_e, Kt_q, Kb_q, J, b, gratio`  
(If dq gain structure is fixed by geometry, you may identify an overall gain instead.)

**Tires**  
`c_kappa, c_alpha, Lrelx, Lrely, mu0, As, kappa_min/max, alpha_min/max`  
(You may keep bounds fixed; fit stiffnesses & relaxations.)

**Chassis/Load Transfer**  
`I_xx, I_zz, k_rllsp, c_rllsp, hs, huf, hur`  
(If inertias known from CAD, lock them and fit roll terms.)

**Aero (optional)**  
`c_aero` (often ≈ 0 at low speeds; fit only if long straights are available).

---

## 6. System Identification Workflow (MATLAB)

### 6.1 Data You Need
- **Inputs:** `D(t)`, `delta_cmd(t)` (and possibly logged `delta(t)` if measured).  
- **Observables:** `v_x, v_y, r, phi, p`, **wheel speeds** `omega_*` if available, **current** `Iq` or phase current proxy, and **position/yaw** from GNSS/INS for cross-checks.  
- **Sampling:** ≥ 50–100 Hz (faster helps tire relaxation fitting).  
- **Excitations:** 
  - Straight-line accel/decel sweeps (motor & longitudinal tire).
  - Constant-radius turns at multiple speeds (lateral tire & load transfer).
  - Gentle weave/sine sweep (roll, yaw, lateral tire, servo lag).

### 6.2 Cost Function (example)
Minimize weighted error between measured and simulated outputs:
\[
J(\theta) = \sum_t \|\ W_v \, [\hat v_x - v_x] \|^2 + \|\ W_r \, [\hat r - r ] \|^2 + \|\ W_\omega \, [\hat\omega_* - \omega_*]\|^2 + \cdots
\]
- Start with \(v_x, r, \omega_*\). Add \(p, \phi\) once force fit stabilizes.
- Use segment-wise normalization to balance maneuvers.

### 6.3 Constraints & Priors
- Box bounds (e.g., `R_phi∈[0.02,0.6]Ω`, `L_e∈[0.1,50] mH`, `J∈[1e-6, 5e-3] kg·m²`, `b≥0`).
- Tire stiffness `c_alpha` typically ≫ `c_kappa` for small robots as well; start with your current defaults and scale.
- Relaxation lengths `Lrelx,Lrely ∈ [0.03, 0.5] m` (ballpark for small tires).

### 6.4 Identification Loop (pseudocode)
```matlab
% 1) Load & preprocess
D = data.D; delta_cmd = data.delta_cmd; y_meas = data.y; t = data.t;

% 2) Parameter struct & bounds
theta0 = struct('R_phi',0.12,'Le',8e-3,'Kt_q',0.05,'Kb_q',0.05, ...
                'J',1e-4,'b',6e-4,'gratio',2.5, ...
                'c_kappa',4,'c_alpha',20,'Lrelx',0.18,'Lrely',0.18, ...
                'mu0',0.8,'As',0.0,'k_rllsp',2,'c_rllsp',1);
lb = ...; ub = ...;

% 3) Simulator handle (calls Modelica FMU or MATLAB clone)
simfun = @(theta) simulate_rover_hifi(theta, D, delta_cmd, t);

% 4) Cost function
function J = costfun(theta)
  y_hat = simfun(theta);
  J = wv*sum((y_hat.vx - y_meas.vx).^2) + ...
      wr*sum((y_hat.r  - y_meas.r ).^2) + ...
      womega*sum((y_hat.omega_rl - y_meas.omega_rl).^2 + ...);
end

% 5) Optimize
theta_hat = fmincon(@(th) costfun(pack(th)), pack(theta0), [],[],[],[], lb, ub);

% 6) Validate on hold-out data and generate plots
```

> You can swap `fmincon` for `lsqnonlin`, `patternsearch`, or Bayesian methods. If sim speed is critical, start with a **frozen subset** (e.g., motor only on straight segments), then grow to tires & roll.

### 6.5 Injecting Identified Parameters Back into the Model
**Option A — Parameter record (recommended):**
```modelica
record RoverHiFiParams
  parameter Real R_phi, Le, Kt_q, Kb_q, J, b, gratio;
  parameter Real c_kappa, c_alpha, Lrelx, Lrely, mu0, As;
  parameter Real I_xx, I_zz, k_rllsp, c_rllsp, hs, huf, hur;
  // ... plus geometry/mass from CAD/spec
end RoverHiFiParams;

parameter RoverHiFiParams P = RoverHiFiParams(
  R_phi=0.11, Le=9.5e-3, Kt_q=0.052, Kb_q=0.053, J=1.2e-4, b=5.8e-4, gratio=2.45,
  c_kappa=4.5, c_alpha=23, Lrelx=0.17, Lrely=0.19, mu0=0.82, As=0.015,
  I_xx=..., I_zz=..., k_rllsp=2.1, c_rllsp=0.95, hs=0.061, huf=0.010, hur=0.010);
```
Then replace usages (e.g., `R_phi` → `P.R_phi`).

**Option B — `.mos` or FMU:**  
Set parameters at runtime:
```mos
setComponentModifier(Model实例, "R_phi=0.11,Le=0.0095,Kt_q=0.052,...");
```

---

## 7. Updating for a New Rover (Hi-Fi Playbook)

### 7.1 From CAD/Scale/Spec (set before SI)
- Geometry: `l_total, l_front, l_rear (sum = l_total), tw, r_tire, w_tire`.
- Mass partition: `mass_total, mass_wheel`, infer `mass_unsprung_*`, then `mass_sprung`.
- Inertias (approx.): 
  \[
  I_{zz}\approx \tfrac{1}{12} m (l_{total}^2 + tw^2),\quad
  I_{xx}\approx \tfrac{1}{2} m (tw/2)^2
  \]
  Replace with CAD if available.
- CG heights: `hs, huf, hur` (from CAD or measurement).
- Motor nameplate: `Vs, Np, Nw, A, B, eta_mech` → initial `Kt_*, Kb_*`.
- Gear ratio `gratio` (mechanical design).

### 7.2 Identify in Stages (recommended order)
1) **Motor/Drivetrain only**: straight-line accel/brake (fit `R_phi, Le, Kt_q, Kb_q, J, b`). Lock `c_kappa,c_alpha` temporarily; use low steering.
2) **Longitudinal tire**: add moderate accel/decel shaping (fit `c_kappa, Lrelx, mu0`).
3) **Lateral tire + roll**: constant-radius turns & weave (fit `c_alpha, Lrely, k_rllsp, c_rllsp`, optionally refine `I_xx, I_zz` if not from CAD).
4) **Cross-validation**: mixed maneuvers; adjust `As` (friction reduction vs slip speed) if high-speed under-prediction appears.

### 7.3 Sanity Checks After Update
- **Wheel speed consistency**: check `4*omega/gratio ≈ Σω_wheels`.
- **Force limits**: Dugoff `f_z∈[0,1]`, no persistent saturation at `z<1` unless operating near friction limit.
- **Rollover flag**: only toggles near aggressive maneuvers; normal loads stay ≫ `normal_lim`.
- **Servo time constant**: verify step response of `δ(t)` vs `δ_cmd(t)`.

---

## 8. Practical Tips & Pitfalls

- **Initialization**: If you re-enable initial equations, avoid referencing symbolic `vx` directly (per your note). Use small non-zero seeds for `omega_*` and `omega` to break static equilibria.
- **Stiffness**: `Le` and `Lrelx/Lrely` can make the system stiff. Use a robust DAE solver; scale units reasonably.
- **Bounds & Clips**: Keep `kappa` and `alpha` bounds tight to avoid division by `(1−kappa)`. Your `clip( tanh|v|, 0.001, 1 )` guards are good—don’t remove them.
- **Sign conventions**: Ensure `delta>0` turns left and yields `r>0` under your axes.
- **Aero**: Only fit `c_aero` if you see consistent high-speed residuals on straights.
- **EMI**: When using `EMIVulnerability`, keep it decoupled from SI unless you have magnetometer/current data specifically for that fit.

---

## 9. Minimal “Hi-Fi New Rover” Checklist

- [ ] Set geometry: `l_total, l_front, l_rear, tw, r_tire, w_tire`  
- [ ] Set mass split & CG heights: `mass_*`, `hs,huf,hur`  
- [ ] Set motor/gear: `Vs, gratio, initial Kt_q, Kb_q, R_phi, Le, J, b`  
- [ ] Run Stage-1 SI (motor) with straight-line data  
- [ ] Run Stage-2 SI (longitudinal tire)  
- [ ] Run Stage-3 SI (lateral + roll)  
- [ ] Validate on mixed maneuvers; lock parameters into a `RoverHiFiParams` record  
- [ ] Re-run safety flags (normal loads, rollover) and servo step tests

---

## 10. Snippets to Keep in Your Repo

### 10.1 Parameter Record (Modelica)
```modelica
package RoverHiFi
  record Params
    parameter Real mass_total, mass_wheel, mass_unsprung_front, mass_unsprung_rear, mass_sprung;
    parameter Real l_total, l_front, l_rear, tw, r_tire, w_tire;
    parameter Real I_wheel, I_xx, I_zz;
    parameter Real R_phi, Le, Kt_q, Kb_q, J, b, gratio;
    parameter Real c_kappa, c_alpha, Lrelx, Lrely, mu0, As;
    parameter Real k_rllsp, c_rllsp, hs, huf, hur;
    parameter Real tau_servo, c_aero;
  end Params;
end RoverHiFi;
```

### 10.2 `.mos` Parameter Injection (example)
```mos
// Load model & set parameters
setComponentModifier(RoverHighFidelity, "
  R_phi=0.108, Le=0.009, Kt_q=0.051, Kb_q=0.052, J=1.1e-4, b=5.7e-4, gratio=2.47,
  c_kappa=4.4, c_alpha=22.5, Lrelx=0.17, Lrely=0.20, mu0=0.81, As=0.012,
  k_rllsp=2.0, c_rllsp=0.95, hs=0.060
");
```

### 10.3 MATLAB: Packing/Unpacking Between SI and Model
```matlab
function P = pack_params(theta)
P = struct('R_phi',theta(1), 'Le',theta(2), 'Kt_q',theta(3), 'Kb_q',theta(4), ...
           'J',theta(5), 'b',theta(6), 'gratio',theta(7), ...
           'c_kappa',theta(8), 'c_alpha',theta(9), 'Lrelx',theta(10), 'Lrely',theta(11), ...
           'mu0',theta(12), 'As',theta(13), 'k_rllsp',theta(14), 'c_rllsp',theta(15));
end
```

---
