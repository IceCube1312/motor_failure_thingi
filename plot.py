#!/usr/bin/env python3
"""
Quadrotor Single Motor Failure - Benchmark & KPI Evaluation
Implements Task 1d of the Inter-IIT Mock Problem Statement.
Compares Stock 4-Axis Controller against Mueller & D'Andrea Relaxed Hover.
"""

import os
import subprocess
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def run_c_simulation():
    print("[1/3] Building and running C physics engine...")
    subprocess.run(["make", "clean"], capture_output=True, text=True)
    res = subprocess.run(["make", "run"], capture_output=True, text=True)
    if res.returncode != 0:
        print("Build failed:\n", res.stderr)
        raise RuntimeError("C simulation compilation failed")
    print("      C simulation completed successfully.")

def compute_kpis(df_stock, df_relaxed, fail_time=2.0):
    print("\n[2/3] Extracting Key Performance Indicators (KPIs)...")

    z_init = 2.0
    stock_post = df_stock[df_stock['time'] >= fail_time]
    relaxed_post = df_relaxed[df_relaxed['time'] >= fail_time]

    # 1. Altitude Loss (m)
    kpi_alt_loss_stock = z_init - stock_post['z'].min()
    kpi_alt_loss_relaxed = z_init - relaxed_post['z'].min()

    # 2. Maximum Tilt Angle (deg)
    kpi_tilt_stock = stock_post['tilt'].max()
    kpi_tilt_relaxed = relaxed_post['tilt'].max()

    # 3. Horizontal Drift at t = 4.0s (m)
    t4_stock = df_stock.iloc[(df_stock['time'] - 4.0).abs().argsort()[:1]]
    t4_rel = df_relaxed.iloc[(df_relaxed['time'] - 4.0).abs().argsort()[:1]]
    drift_stock = np.sqrt(t4_stock['x'].values[0]**2 + t4_stock['y'].values[0]**2)
    drift_relaxed = np.sqrt(t4_rel['x'].values[0]**2 + t4_rel['y'].values[0]**2)

    # 4. Time to Recover (s)
    recovered = relaxed_post[(relaxed_post['time'] > 2.5) & (relaxed_post['vz'].abs() < 0.1)]
    if len(recovered) > 0:
        kpi_rec_time_relaxed = recovered['time'].iloc[0] - fail_time
    else:
        kpi_rec_time_relaxed = np.nan
    kpi_rec_time_stock = "Failed (Crashed)"

    # 5. Steady Spin Rate (rad/s)
    spin_stock = stock_post[stock_post['time'] >= 5.0]['r'].mean()
    spin_relaxed = relaxed_post[relaxed_post['time'] >= 5.0]['r'].mean()

    # Print KPI Table
    print("=" * 76)
    print(f"{'KPI Benchmark Metric':<32} | {'Stock Controller':<18} | {'Relaxed Hover (Ours)':<18}")
    print("-" * 76)
    print(f"{'1. Altitude Loss (Delta z_max)':<32} | {kpi_alt_loss_stock:<18.2f} m | {kpi_alt_loss_relaxed:<18.2f} m")
    print(f"{'2. Maximum Tilt Angle (theta_max)':<32} | {kpi_tilt_stock:<18.1f} deg | {kpi_tilt_relaxed:<18.1f} deg")
    print(f"{'3. Horizontal Drift (at t=4s)':<32} | {drift_stock:<18.2f} m | {drift_relaxed:<18.2f} m")
    rec_rel_str = f"{kpi_rec_time_relaxed:.2f} s" if not np.isnan(kpi_rec_time_relaxed) else "N/A"
    print(f"{'4. Recovery Time (t_rec)':<32} | {kpi_rec_time_stock:<18} | {rec_rel_str:<18}")
    print(f"{'5. Steady Spin Rate (Omega)':<32} | {spin_stock:<18.1f} rad/s | {spin_relaxed:<18.1f} rad/s")
    print("=" * 76)

def generate_plots(df_stock, df_relaxed, fail_time=2.0):
    print("\n[3/3] Generating separate publication-quality benchmark plots...")
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    c_stock = '#d62728'   # Crimson red
    c_rel = '#1f77b4'     # Deep blue
    c_fail = '#ff7f0e'    # Orange dashed line

    # -------------------------------------------------------------
    # Plot 1: Altitude Response z(t)
    # -------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(8, 5))
    ax1.plot(df_stock['time'], df_stock['z'], label='Stock Controller (Crashes)', color=c_stock, linewidth=2.2)
    ax1.plot(df_relaxed['time'], df_relaxed['z'], label='Relaxed Hover (Stabilized)', color=c_rel, linewidth=2.2)
    ax1.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label=f'Motor 1 Failure (t = {fail_time}s)')
    ax1.set_title('Quadrotor Altitude Response $z(t)$', fontsize=13, fontweight='bold')
    ax1.set_xlabel('Time [s]', fontsize=11)
    ax1.set_ylabel('Altitude [m]', fontsize=11)
    ax1.set_ylim(-0.2, 2.5)
    ax1.legend(loc='lower left', frameon=True)
    ax1.grid(True, linestyle=':', alpha=0.6)
    fig1.tight_layout()
    fig1.savefig('plot_1_altitude.png', dpi=300)
    print("      Saved: plot_1_altitude.png")

    # -------------------------------------------------------------
    # Plot 2: Vehicle Tilt Angle theta(t)
    # -------------------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(8, 5))
    ax2.plot(df_stock['time'], df_stock['tilt'], label='Stock Controller', color=c_stock, linewidth=2.2)
    ax2.plot(df_relaxed['time'], df_relaxed['tilt'], label='Relaxed Hover', color=c_rel, linewidth=2.2)
    ax2.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label=f'Motor 1 Failure (t = {fail_time}s)')
    ax2.axhline(y=90.0, color='gray', linestyle=':', label='Inversion / Flip (90°)')
    ax2.set_title('Vehicle Tilt Angle $\\theta(t)$', fontsize=13, fontweight='bold')
    ax2.set_xlabel('Time [s]', fontsize=11)
    ax2.set_ylabel('Tilt Angle [deg]', fontsize=11)
    ax2.legend(loc='upper left', frameon=True)
    ax2.grid(True, linestyle=':', alpha=0.6)
    fig2.tight_layout()
    fig2.savefig('plot_2_tilt.png', dpi=300)
    print("      Saved: plot_2_tilt.png")

    # -------------------------------------------------------------
    # Plot 3: Body Angular Rates (Relaxed Hover)
    # -------------------------------------------------------------
    fig3, ax3 = plt.subplots(figsize=(8, 5))
    ax3.plot(df_relaxed['time'], df_relaxed['p'], label='Roll rate $p$ (Damped)', color='#2ca02c', linewidth=1.8)
    ax3.plot(df_relaxed['time'], df_relaxed['q'], label='Pitch rate $q$ (Damped)', color='#9467bd', linewidth=1.8)
    ax3.plot(df_relaxed['time'], df_relaxed['r'], label='Yaw rate $r$ (Stable Spin)', color=c_rel, linewidth=2.2)
    ax3.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label=f'Motor 1 Failure (t = {fail_time}s)')
    ax3.set_title('Body Angular Rates $\\boldsymbol{\\omega}^B(t)$ [Relaxed Hover]', fontsize=13, fontweight='bold')
    ax3.set_xlabel('Time [s]', fontsize=11)
    ax3.set_ylabel('Angular Velocity [rad/s]', fontsize=11)
    ax3.legend(loc='lower left', frameon=True)
    ax3.grid(True, linestyle=':', alpha=0.6)
    fig3.tight_layout()
    fig3.savefig('plot_3_body_rates.png', dpi=300)
    print("      Saved: plot_3_body_rates.png")

    # -------------------------------------------------------------
    # Plot 4: Motor Thrust Allocation (Relaxed Hover)
    # -------------------------------------------------------------
    fig4, ax4 = plt.subplots(figsize=(8, 5))
    ax4.plot(df_relaxed['time'], df_relaxed['f1'], label='$f_1$ (Front - Dead)', color='black', linestyle=':', linewidth=2.2)
    ax4.plot(df_relaxed['time'], df_relaxed['f2'], label='$f_2$ (Left)', color='#1f77b4', linewidth=1.8)
    ax4.plot(df_relaxed['time'], df_relaxed['f3'], label='$f_3$ (Rear - Throttled)', color='#ff7f0e', linewidth=1.8)
    ax4.plot(df_relaxed['time'], df_relaxed['f4'], label='$f_4$ (Right)', color='#2ca02c', linewidth=1.8)
    ax4.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label=f'Motor 1 Failure (t = {fail_time}s)')
    ax4.set_title('Motor Thrust Allocation $f_i(t)$ [Relaxed Hover]', fontsize=13, fontweight='bold')
    ax4.set_xlabel('Time [s]', fontsize=11)
    ax4.set_ylabel('Thrust Force [N]', fontsize=11)
    ax4.set_ylim(-0.2, 4.0)
    ax4.legend(loc='center right', frameon=True)
    ax4.grid(True, linestyle=':', alpha=0.6)
    fig4.tight_layout()
    fig4.savefig('plot_4_motor_thrusts.png', dpi=300)
    print("      Saved: plot_4_motor_thrusts.png")



if __name__ == '__main__':
    run_c_simulation()
    df_stock = pd.read_csv('flight_stock.csv')
    df_relaxed = pd.read_csv('flight_relaxed.csv')
    compute_kpis(df_stock, df_relaxed)
    generate_plots(df_stock, df_relaxed)
