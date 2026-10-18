# Quadrotor Single Motor Failure Simulation

A lightweight C physics simulation and benchmark tool demonstrating quadrotor behavior after a single motor failure. It compares a standard 4-axis flight controller (which crashes) against a relaxed hover controller (which surrenders yaw to maintain altitude while spinning).

## Repository Structure

```
├── Makefile            # Build script for compilation and automation
├── README.md           # Project overview and run instructions
├── run_benchmark.py    # Python benchmark script: runs sim, calculates KPIs, plots data
├── include/
│   ├── failure.h       # Drone parameters, struct definitions, and function prototypes
│   └── raymath.h       # Vector and quaternion mathematics header
├── src/
│   ├── failure.c       # 1000 Hz physics engine, quaternion kinematics, and controllers
│   └── main.c          # Top-level runner executing stock vs relaxed hover flights
└── [Generated Outputs]
    ├── flight_stock.csv         # Flight telemetry for stock controller
    ├── flight_relaxed.csv       # Flight telemetry for relaxed hover controller
    ├── plot_1_altitude.png      # Altitude response over time
    ├── plot_2_tilt.png          # Vehicle tilt angle over time
    ├── plot_3_body_rates.png    # Angular body rates (roll, pitch, yaw)
    └── plot_4_motor_thrusts.png # Individual motor thrust commands
```

## Prerequisites

- C Compiler: `gcc`
- Build Tool: `make`
- Python 3 with the following libraries:
  - `matplotlib`
  - `pandas`
  - `numpy`

To install the Python libraries:
```bash
pip install matplotlib pandas numpy
```

## Compilation and Execution

### 1. Compile the Simulation
```bash
make
```
This builds the C simulation into a binary named `failure_sim`.

### 2. Run the Simulation
```bash
make run
```
This runs the simulation and logs two telemetry CSV files:
- `flight_stock.csv`: Flight data of the standard controller failing and crashing.
- `flight_relaxed.csv`: Flight data of the relaxed hover controller stabilizing.

### 3. Run Benchmark and Generate Plots (One-Command)
```bash
make benchmark
```
This compiles the code, runs both flight simulations, prints the 5 KPI comparison metrics in the terminal, and saves the 4 analysis plots as PNG files.

### 4. Clean Build Files
```bash
make clean
```
Removes all object files (`src/*.o`), the executable (`failure_sim`), CSV logs, and generated PNG plots.
