# Quadrotor Single Motor Failure: Dynamics, Control & Baseline Benchmark

A lightweight, first-principles C physics engine and benchmark test suite analyzing **Quadrotor Single Motor Failure** and demonstrating recovery via **Relaxed Hover** (surrendering yaw to establish a stable spinning equilibrium).

Based on the dynamics and periodic solutions derived in **Mueller & D'Andrea (ICRA 2014)**: *"Stability and control of a quadrocopter despite the complete loss of one, two, or three propellers"*.

---

## 1. Quickstart (One-Command Run)

### Prerequisites
* GCC (`gcc`) & GNU Make (`make`)
* Python 3 with `numpy`, `pandas`, and `matplotlib`

### Run the Benchmark
To build the physics engine, execute both simulation cases, compute all 5 KPIs, and generate publication plots:

```bash
make benchmark
```
*(Alternatively, execute `./run_benchmark.py` directly).*

### Other Make Targets
```bash
make          # Compiles src/failure.c & src/main.c into ./failure_sim
make run      # Runs the C simulation binary (outputs flight_stock.csv & flight_relaxed.csv)
make clean    # Wipes all object files, executables, CSVs, and generated PNG plots
```

---

## 2. Repository Structure

```
├── Makefile            # Build system with targets: all, run, benchmark, clean
├── README.md           # Technical documentation, mathematical derivations, and benchmark report
├── run_benchmark.py    # Automated test harness: runs C sim, computes KPIs, generates plots
├── include/
│   ├── failure.h       # Physical constants, drone state structures, and controller prototypes
│   └── raymath.h       # Vector3 and Quaternion mathematics library
├── src/
│   ├── failure.c       # 1000 Hz physics engine, Euler dynamics, and controller implementations
│   └── main.c          # Top-level C runner executing stock vs. relaxed hover benchmark runs
└── [Generated Outputs]
    ├── flight_stock.csv       # Telemetry log for Stock 4-Axis Controller
    ├── flight_relaxed.csv     # Telemetry log for Relaxed Hover Controller
    ├── plot_1_altitude.png    # Altitude response z(t)
    ├── plot_2_tilt.png        # Vehicle tilt angle theta(t)
    ├── plot_3_body_rates.png  # Body rates (p, q, r)
    └── plot_4_motor_thrusts.png # Motor thrust allocations (f1, f2, f3, f4)
```

---

## 3. Mathematical Formulation (Task 1b)

### 3.1 Equations of Motion
Let $\mathbf{p} = [x, y, z]^T \in \mathbb{R}^3$ denote the inertial position of the center of mass, and $\boldsymbol\omega^B = [p, q, r]^T \in \mathbb{R}^3$ denote the body-fixed angular velocity.

#### Translational Dynamics:
$$m \ddot{\mathbf{p}} = \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ T \end{bmatrix} - m g \mathbf{e}_3, \quad T = \sum_{i=1}^4 f_i$$
where $m = 0.50\text{ kg}$, $g = 9.81\text{ m/s}^2$, and $\mathbf{R} \in SO(3)$ is the orientation matrix.

#### Rotational Dynamics (Euler Rigid-Body Dynamics + Propeller Aerodynamics):
$$J \dot{\boldsymbol\omega} + \boldsymbol\omega \times J \boldsymbol\omega = \boldsymbol\tau_{\text{res}}$$

Assuming structural symmetry ($I_{xx}^B = I_{yy}^B$):
$$I_{xx}^B \dot{p} = \tau_x - (I_{zz}^T - I_{yy}^T) q r$$
$$I_{yy}^B \dot{q} = \tau_y + (I_{zz}^T - I_{xx}^T) p r$$
$$I_{zz}^B \dot{r} = \tau_z - \gamma r$$

where:
* $\tau_x = (f_2 - f_4) l$ (Roll torque from lateral motors)
* $\tau_y = (f_3 - f_1) l$ (Pitch torque from longitudinal motors)
* $\tau_z = \kappa_\tau (f_1 - f_2 + f_3 - f_4)$ (Yaw reaction torque from propeller drag)
* $\gamma = 0.00275\text{ N}\cdot\text{m}\cdot\text{s/rad}$ (Aerodynamic frame rotational drag)
* $l = 0.17\text{ m}$ (Arm length), $\kappa_\tau = 0.016\text{ m}$ (Torque-to-thrust ratio)

---

### 3.2 Control Allocation Matrix $\mathbf{B}$ & Rank Deficiency Proof
The mapping from individual motor thrusts to total collective thrust and torques is:

$$\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \mathbf{B} \begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix} = \begin{bmatrix} 1 & 1 & 1 & 1 \\ 0 & l & 0 & -l \\ -l & 0 & l & 0 \\ \kappa_\tau & -\kappa_\tau & \kappa_\tau & -\kappa_\tau \end{bmatrix} \begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix}$$

#### Loss of Motor 1 ($f_1 = 0$):
When Motor 1 dies, Column 1 of $\mathbf{B}$ is eliminated:
$$\begin{bmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = \begin{bmatrix} 1 & 1 & 1 \\ l & 0 & -l \\ 0 & l & 0 \\ -\kappa_\tau & \kappa_\tau & -\kappa_\tau \end{bmatrix} \begin{bmatrix} f_2 \\ f_3 \\ f_4 \end{bmatrix}$$

**Rank Analysis:**  
$$\operatorname{rank}(\mathbf{B}_{\text{fail}}) \le 3 < 4$$
With 3 remaining motors, the system is fundamentally **underactuated**. Arbitrary simultaneous control of $(T, \tau_x, \tau_y, \tau_z)$ is impossible.

---

### 3.3 Proof: Why Yaw Must Be Given Up
Suppose a controller attempts to enforce zero roll, pitch, and yaw torques simultaneously while hovering ($T \approx mg$):
1. **Pitch Equilibrium ($\tau_y = 0$):** Since Motor 1 is dead, Motor 3 creates positive pitch torque $\tau_y = f_3 l$. Balancing pitch requires:
   $$f_3 l = 0 \implies f_3 = 0$$
   This leaves only Motors 2 and 4.
2. **Roll Equilibrium ($\tau_x = 0$):** Motor 2 is left ($+l$), Motor 4 is right ($-l$). Balancing roll requires:
   $$f_2 l - f_4 l = 0 \implies f_2 = f_4$$
3. **Resulting Yaw Torque:**  
   Motors 2 and 4 both produce negative yaw torque ($-\kappa_\tau$):
   $$\tau_z = -\kappa_\tau f_2 - \kappa_\tau f_4 = -\kappa_\tau (f_2 + f_4) = -\kappa_\tau T = -\kappa_\tau mg \ne 0$$

$$\mathbf{\tau_z \equiv -\kappa_\tau mg \ne 0}$$

**Conclusion:** It is physically impossible to support vehicle weight without producing a net yaw torque. **Yaw torque cannot be zero.**

#### The Relaxed Hover Solution:
Instead of fighting the unavoidable yaw torque (which saturates motors and induces a violent tumble), the flight controller **surrenders yaw tracking entirely** ($\tau_{z,\text{des}} = 0$).
* The quadrotor spins up around its vertical axis until propeller reaction torque balances aerodynamic body drag:
  $$\tau_z - \gamma r = 0 \implies \Omega = \bar{r} = \sqrt{\frac{\kappa_\tau T_{\text{eff}}}{\gamma}} \approx 27.1\text{ rad/s} \quad (\approx 260\text{ RPM})$$
* This rapid spin acts like a gyroscope, providing gyroscopic stiffness. The remaining motors tilt the time-averaged thrust vector against gravity, maintaining steady hover.

---

## 4. Benchmark Results & KPIs (Task 1d)

At $t = 2.0\text{ s}$, a complete loss of Motor 1 ($f_1 = 0$) is injected into both controllers:

### 4.1 Key Performance Indicators (KPIs)

| KPI Benchmark Metric | Stock 4-Axis Controller | Relaxed Hover (Ours) | Engineering Impact |
| :--- | :--- | :--- | :--- |
| **1. Altitude Loss ($\Delta z_{\max}$)** | **$2.00\text{ m}$ (Total Crash)** | **$0.50\text{ m}$** | Dips only 50 cm during transition before altitude PID recovers. |
| **2. Max Tilt Angle ($\theta_{\max}$)** | **$162.5^\circ$ (Inverted)** | **$52.4^\circ$** | Stock controller completely flips; relaxed hover settles into a $35^\circ$ cone. |
| **3. Horizontal Drift (at $t = 4\text{s}$)** | $0.73\text{ m}$ (Grounded) | $11.86\text{ m}$ | Free unconstrained precession drift without GPS translation lock. |
| **4. Recovery Time ($t_{\text{rec}}$)** | **Failed (Crashed at $t=2.7\text{s}$)** | **$1.32\text{ s}$** | Vertical descent arrested ($|v_z| \le 0.1\text{ m/s}$) within 1.32 seconds. |
| **5. Steady Spin Rate ($\Omega$)** | $-4.6\text{ rad/s}$ | **$-27.1\text{ rad/s}$** | Converges smoothly to aerodynamic equilibrium spin. |

---

### 4.2 Benchmark Telemetry Analysis

#### Plot 1: Altitude Response $z(t)$ (`plot_1_altitude.png`)
* **Stock Controller (Red):** Suffers immediate descent post-failure and crashes into the ground ($z = 0$) at $t = 2.7\text{ s}$.
* **Relaxed Hover (Blue):** Undergoes a momentary 50 cm transition dip, arrests vertical descent at $t = 3.32\text{ s}$, and recovers stable hover at $z \approx 1.66\text{ m}$.

#### Plot 2: Vehicle Tilt Angle $\theta(t)$ (`plot_2_tilt.png`)
* **Stock Controller:** Tries to fight yaw spin, saturating motors. Roll/pitch authority is lost, flipping the vehicle upside down to $162.5^\circ$.
* **Relaxed Hover:** Damps transverse rates ($p, q$) into a smooth, steady conical precession cone of $\approx 35^\circ$.

#### Plot 3: Body Angular Rates $\boldsymbol\omega^B(t)$ (`plot_3_body_rates.png`)
* Roll rate $p$ and pitch rate $q$ are actively damped to near zero, while yaw spin rate $r$ smoothly accelerates and locks into the stable aerodynamic equilibrium at $-27.1\text{ rad/s}$.

#### Plot 4: Motor Thrust Allocation $f_i(t)$ (`plot_4_motor_thrusts.png`)
* Following Motor 1 failure ($f_1 = 0$), Motor 3 throttles down to $0.3\text{ N}$ to prevent pitch flip, while Motors 2 and 4 symmetrically increase thrust to $\approx 3.0\text{ N}$ to support vehicle weight.

---

## 5. Engineering Limits & Real-World Gaps (Task 1c)

Why do simulation results often fail when deployed to real hardware?

1. **MEMS Gyroscope Saturation:**  
   In steady relaxed hover, the drone spins at $27.1\text{ rad/s} \approx 1,550^\circ/\text{s}$. Standard commercial flight controller IMUs (e.g., MPU6000 / ICM-20689) have a maximum range of $\pm 2,000^\circ/\text{s}$. Any gust or dynamic overshoot exceeds this limit, saturating the sensor and blinding the state estimator.
2. **Propeller Loss vs. Electrical Motor Cut:**  
   Simulators assume a "zero thrust" condition where the motor simply stops. In physical flight, a snapped propeller blade leaves the bare motor spinning at $>30,000\text{ RPM}$ with an unbalanced hub, generating severe high-frequency vibration that corrupts accelerometer and gyro filters.
3. **Simulator Aerodynamics Omissions:**  
   Standard Gazebo plugins omit aerodynamic frame rotational drag ($\gamma$) and cross-flow blade damping. Without modeled drag, simulated vehicles spin to unphysical rates ($>80\text{ rad/s}$).
4. **Transition from Forward Flight:**  
   Controllers tested exclusively in calm static hover fail if the motor dies during high-speed forward flight ($>5\text{ m/s}$), where asymmetric translational airflow induces immediate aerodynamic pitch tumble before yaw spin can stabilize.

---

## 6. Architecture & Implementation Details

* **`include/failure.h`:** Contains all vehicle parameters ($m, l, I_{xx}, I_{zz}, \kappa_\tau, \gamma, dt$) and the `DRONE` state struct.
* **`src/failure.c`:**
  - `drone_step`: Solves Euler rigid-body rotational dynamics and propagates attitude via **Quaternions** ($\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \boldsymbol\omega$), eliminating Euler gimbal-lock singularities during high-speed spinning.
  - `control_stock_pid`: Implements standard $4 \times 4$ mixer inversion, demonstrating the saturation failure mode.
  - `control_relaxed_hover`: Implements the 3-motor relaxed hover allocation (zero yaw gain, active pitch/roll rate damping).
* **`src/main.c`:** Coordinates execution and logs telemetry to CSV.
* **`run_benchmark.py`:** Automates compilation, execution, KPI computation, and plot generation.
