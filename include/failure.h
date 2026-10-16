#ifndef FAILURE_H
#define FAILURE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "raymath.h"

#define GRAVITY          9.81f      // m/s^2
#define MASS             0.500f     // kg (Total vehicle mass)
#define ARM_LENGTH       0.170f     // m (Distance from CoM to rotor center, l)
#define I_XX             0.0032f    // I_xx^B
#define I_YY             0.0032f    // I_yy^B (symmetric: I_xx = I_yy)
#define I_ZZ             0.0055f    // I_zz^B
#define KAPPA_TAU        0.016f     // Torque-to-thrust ratio: c_tau / c_f (m)
#define GAMMA_DRAG       0.00275f   // Yaw aerodynamic drag coefficient (N*m*s/rad)
#define F_MIN            0.0f       // Minimum thrust (N)
#define F_MAX            3.5f       // Maximum thrust (N) (4 * 3.5 = 14 N > mg = 4.9 N)
#define SIM_DT           0.001f     // Timestep dt = 1 ms (1000 Hz physics loop)
#define SIM_DURATION     6.0f       // Total run duration (seconds)
#define FAILURE_TIME     2.0f       // Time at which Motor 1 fails (seconds)
#define CONTROLLER_STOCK   0        // Stock PID attempting 4-axis control (crashes)
#define CONTROLLER_RELAXED 1        // Relaxed Hover (surrenders yaw, stabilizes spin)
#define FAILED_MOTOR_ID 0

typedef struct drone {
    Vector3    pos;        // Inertial position: x, y, z [m]
    Vector3    velo;       // Inertial velocity: vx, vy, vz [m/s]
    Quaternion quat;       // Attitude quaternion: [x, y, z, w]
    Vector3    attitude;   // Euler/tilt: roll (phi), pitch (theta), yaw (psi) [rad]
    Vector3    ang_velo;   // Body rates: p (roll rate), q (pitch rate), r (yaw rate) [rad/s]
    float      thrusts[4]; // Current motor thrust forces f1, f2, f3, f4 [N]
} DRONE;

typedef struct motor_cmd {
    float f[4];         // Thrusts commanded to each motor [N]
			// Basically what we want each motor to thrust
} MOTOR_CMD;

void drone_init(DRONE *d, Vector3 init_pos);

void drone_step(DRONE *d, const MOTOR_CMD *cmd, float dt, bool inject_failure);

void control_stock_pid(const DRONE *d, MOTOR_CMD *cmd, float target_z, float dt);
void control_relaxed_hover(const DRONE *d, MOTOR_CMD *cmd, float target_z, float dt);

void run_benchmark(const char *csv_filename, int controller_type);

#endif
