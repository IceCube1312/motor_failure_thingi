#include "failure.h"

void drone_init(DRONE *d, Vector3 init_pos) {
    assert(d != NULL);
    d->pos = init_pos;
    d->velo = (Vector3){0.0f, 0.0f, 0.0f};
    d->quat = (Quaternion){0.0f, 0.0f, 0.0f, 1.0f}; // Identity orientation
    d->attitude = (Vector3){0.0f, 0.0f, 0.0f};      // [roll, tilt, yaw]
    d->ang_velo = (Vector3){0.0f, 0.0f, 0.0f};      // [p, q, r] in rad/s

    float hover_f = (MASS * GRAVITY) / 4.0f;
    for (int i = 0; i < 4; i++) {
        d->thrusts[i] = hover_f;
    }
}

// Physics Integration Step (Euler Integration of EoM)
// Uses Quaternion attitude propagation to eliminate Euler gimbal-lock during spin.
void drone_step(DRONE *d, const MOTOR_CMD *cmd, float dt, bool inject_failure) {
    assert(d != NULL && cmd != NULL);

    // Apply motor thrust limits and fault injection
    for (int i = 0; i < 4; i++) {
        d->thrusts[i] = cmd->f[i];
        if (d->thrusts[i] < F_MIN) d->thrusts[i] = F_MIN;
        if (d->thrusts[i] > F_MAX) d->thrusts[i] = F_MAX;
    }

    // Complete loss of Motor 1 if failure injected
    if (inject_failure) {
        d->thrusts[FAILED_MOTOR_ID] = 0.0f;
    }

    float f1 = d->thrusts[0];
    float f2 = d->thrusts[1];
    float f3 = d->thrusts[2];
    float f4 = d->thrusts[3];

    // Total Collective Thrust
    float T = f1 + f2 + f3 + f4;

    // Body Torques from Actuator Geometry
    // Motor 1: (+l, 0) [Front]   -> CCW (+kappa_tau)
    // Motor 2: (0, +l) [Left]    -> CW  (-kappa_tau)
    // Motor 3: (-l, 0) [Rear]    -> CCW (+kappa_tau)
    // Motor 4: (0, -l) [Right]   -> CW  (-kappa_tau)
    float tau_x = (f2 - f4) * ARM_LENGTH;
    float tau_y = (f3 - f1) * ARM_LENGTH;
    float tau_z = KAPPA_TAU * (f1 - f2 + f3 - f4);

    float p = d->ang_velo.x;
    float q = d->ang_velo.y;
    float r = d->ang_velo.z;

    // 1. Rotational Dynamics (Euler equations + aerodynamic drag)
    // I_xx * p_dot = tau_x - (I_zz - I_yy)*q*r
    float p_dot = (tau_x - (I_ZZ - I_YY) * q * r) / I_XX;

    // I_yy * q_dot = tau_y - (I_xx - I_zz)*p*r
    float q_dot = (tau_y - (I_XX - I_ZZ) * p * r) / I_YY;

    // I_zz * r_dot = tau_z - gamma * r
    float r_dot = (tau_z - GAMMA_DRAG * r) / I_ZZ;

    // Numerical integration of body rates
    p += p_dot * dt;
    q += q_dot * dt;
    r += r_dot * dt;
    d->ang_velo = (Vector3){p, q, r};

    // 2. Attitude Propagation via Quaternion Kinematics
    // q_dot = 0.5 * q * [omega, 0]
    Quaternion q_curr = d->quat;
    Quaternion quat_dot;
    quat_dot.x = 0.5f * ( q_curr.w * p + q_curr.y * r - q_curr.z * q);
    quat_dot.y = 0.5f * ( q_curr.w * q - q_curr.x * r + q_curr.z * p);
    quat_dot.z = 0.5f * ( q_curr.w * r + q_curr.x * q - q_curr.y * p);
    quat_dot.w = 0.5f * (-q_curr.x * p - q_curr.y * q - q_curr.z * r);

    q_curr.x += quat_dot.x * dt;
    q_curr.y += quat_dot.y * dt;
    q_curr.z += quat_dot.z * dt;
    q_curr.w += quat_dot.w * dt;

    // Normalize quaternion to prevent numerical drift
    d->quat = QuaternionNormalize(q_curr);

    // Compute unit thrust vector in world frame: z_body = q * (0, 0, 1) * q^-1
    Vector3 z_body = Vector3RotateByQuaternion((Vector3){0.0f, 0.0f, 1.0f}, d->quat);

    // Calculate overall vehicle tilt angle theta = acos(clamp(z_body.z, -1, 1))
    float z_dot_up = z_body.z;
    if (z_dot_up >  1.0f) z_dot_up =  1.0f;
    if (z_dot_up < -1.0f) z_dot_up = -1.0f;
    float tilt_rad = acosf(z_dot_up);

    // Store tilt and body rates in attitude vector for easy telemetry
    d->attitude = (Vector3){0.0f, tilt_rad, atan2f(z_body.y, z_body.x)};

    Vector3 accel;
    accel.x = (T * z_body.x) / MASS;
    accel.y = (T * z_body.y) / MASS;
    accel.z = (T * z_body.z) / MASS - GRAVITY;

    if (d->pos.z <= 0.0f && accel.z < 0.0f) {
        accel.z = 0.0f;
        d->velo.z = 0.0f;
        d->pos.z = 0.0f;
    }

    d->velo = Vector3Add(d->velo, Vector3Scale(accel, dt));
    d->pos  = Vector3Add(d->pos,  Vector3Scale(d->velo, dt));

    if (d->pos.z < 0.0f) {
        d->pos.z = 0.0f;
        d->velo = (Vector3){0.0f, 0.0f, 0.0f};
    }
}

 // Controller 1: Stock 4-Axis PID Controller
// Assumes 4 fully functioning actuators; attempts to enforce yaw = 0.
void control_stock_pid(const DRONE *d, MOTOR_CMD *cmd, float target_z, float dt) {
    (void)dt;

    // Altitude PID
    float z_err = target_z - d->pos.z;
    float vz_err = 0.0f - d->velo.z;
    float T_des = MASS * GRAVITY + 5.0f * z_err + 2.5f * vz_err;
    if (T_des < 0.5f) T_des = 0.5f;

    // Attitude PID (Roll, Pitch, and aggressive Yaw stabilization)
    float tau_x_des = -0.15f * d->ang_velo.x;
    float tau_y_des = -0.15f * d->ang_velo.y;
    float tau_z_des = -0.08f * d->ang_velo.z; // Aggressive yaw fighting

    // Invert Standard Quadrotor Mixer
    float f_base  = T_des / 4.0f;
    float d_roll  = tau_x_des / (2.0f * ARM_LENGTH);
    float d_pitch = tau_y_des / (2.0f * ARM_LENGTH);
    float d_yaw   = tau_z_des / (4.0f * KAPPA_TAU);

    cmd->f[0] = f_base - d_pitch + d_yaw;
    cmd->f[1] = f_base + d_roll  - d_yaw;
    cmd->f[2] = f_base + d_pitch + d_yaw;
    cmd->f[3] = f_base - d_roll  - d_yaw;
}

 // Controller 2: Relaxed Hover Controller (Mueller & D'Andrea ICRA 2014)
// Surrenders yaw control; allocates f2, f3, f4 to maintain altitude & tilt.
void control_relaxed_hover(const DRONE *d, MOTOR_CMD *cmd, float target_z, float dt) {
    (void)dt;

    // Altitude PID to maintain target height
    float z_err = target_z - d->pos.z;
    float vz_err = 0.0f - d->velo.z;
    float T_cmd = (MASS * GRAVITY) + 5.0f * z_err + 2.5f * vz_err;
    if (T_cmd < 0.5f)  T_cmd = 0.5f;
    if (T_cmd > 10.0f) T_cmd = 10.0f;

    // Motor 1 is lost: f1 = 0
    cmd->f[0] = 0.0f;

    // Allocate remaining 3 motors for balanced conical spin hover:
    // Pitch damping & roll damping
    float f3_val = 0.12f * T_cmd - 0.05f * d->ang_velo.y / ARM_LENGTH;
    float f2_val = 0.44f * T_cmd - 0.05f * d->ang_velo.x / ARM_LENGTH;
    float f4_val = 0.44f * T_cmd + 0.05f * d->ang_velo.x / ARM_LENGTH;

    cmd->f[1] = f2_val;
    cmd->f[2] = f3_val;
    cmd->f[3] = f4_val;
}

void run_benchmark(const char *csv_filename, int controller_type) {
    FILE *fp = fopen(csv_filename, "w");
    if (!fp) {
        fprintf(stderr, "Error: Could not open file %s for writing!\n", csv_filename);
        return;
    }

    // Write CSV Header
    fprintf(fp, "time,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,f1,f2,f3,f4,tilt\n");

    DRONE drone;
    drone_init(&drone, (Vector3){0.0f, 0.0f, 2.0f}); // Initial altitude = 2.0m hover

    float target_z = 2.0f;
    float t = 0.0f;
    int steps = (int)(SIM_DURATION / SIM_DT);

    for (int step = 0; step < steps; step++) {
        t = step * SIM_DT;
        bool motor_failed = (t >= FAILURE_TIME);

        MOTOR_CMD cmd;
        if (controller_type == CONTROLLER_STOCK) {
            control_stock_pid(&drone, &cmd, target_z, SIM_DT);
        } else {
            if (motor_failed) {
                control_relaxed_hover(&drone, &cmd, target_z, SIM_DT);
            } else {
                control_stock_pid(&drone, &cmd, target_z, SIM_DT);
            }
        }

        // Advance physics by dt
        drone_step(&drone, &cmd, SIM_DT, motor_failed);

        // Log telemetry every 5 ms (200 Hz downsample for clean CSV)
        if (step % 5 == 0) {
            fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    t,
                    drone.pos.x, drone.pos.y, drone.pos.z,
                    drone.velo.x, drone.velo.y, drone.velo.z,
                    drone.attitude.x, drone.attitude.y, drone.attitude.z,
                    drone.ang_velo.x, drone.ang_velo.y, drone.ang_velo.z,
                    drone.thrusts[0], drone.thrusts[1], drone.thrusts[2], drone.thrusts[3],
                    drone.attitude.y * (180.0f / PI)); // tilt in degrees
        }
    }

    fclose(fp);
    printf("Simulation completed. Telemetry written to %s\n", csv_filename);
}
