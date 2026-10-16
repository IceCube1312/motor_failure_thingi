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
    res = subprocess.run(["make", "clean"], capture_output=True, text=True)
    res = subprocess.run(["make", "run"], capture_output=True, text=True)
    if res.returncode != 0:
        print("Build failed:\n", res.stderr)
        raise RuntimeError("C simulation compilation failed")
    print("      C simulation completed successfully.")

def compute_kpis(df_stock, df_relaxed, fail_time=2.0):
    print("\n[2/3] Extracting Key Performance Indicators (KPIs)...")

    # 1. Altitude Loss (m): Drop from initial hover height (2.0m) to minimum z post-failure
    z_init = 2.0
    stock_post = df_stock[df_stock['time'] >= fail_time]
    relaxed_post = df_relaxed[df_relaxed['time'] >= fail_time]

    kpi_alt_loss_stock = z_init - stock_post['z'].min()
    kpi_alt_loss_relaxed = z_init - relaxed_post['z'].min()

    # 2. Maximum Tilt Angle (degrees) post-failure
    kpi_tilt_stock = stock_post['tilt'].max()
    kpi_tilt_relaxed = relaxed_post['tilt'].max()

    # 3. Horizontal Drift at t = 4.0s (m)
    t4_stock = df_stock.iloc[(df_stock['time'] - 4.0).abs().argsort()[:1]]
    t4_rel = df_relaxed.iloc[(df_relaxed['time'] - 4.0).abs().argsort()[:1]]
    drift_stock = np.sqrt(t4_stock['x'].values[0]**2 + t4_stock['y'].values[0]**2)
    drift_relaxed = np.sqrt(t4_rel['x'].values[0]**2 + t4_rel['y'].values[0]**2)

    # 4. Time to Recover (s)
    # Defined as time until vertical velocity |vz| <= 0.1 m/s and altitude stabilizes
    recovered = relaxed_post[(relaxed_post['time'] > 2.5) & (relaxed_post['vz'].abs() < 0.1)]
    if len(recovered) > 0:
        kpi_rec_time_relaxed = recovered['time'].iloc[0] - fail_time
    else:
        kpi_rec_time_relaxed = np.nan
    kpi_rec_time_stock = "Failed (Crashed)"

    # 5. Steady-State Spin Rate (rad/s)
    # Average yaw rate r over final 1.0 second (t = 5.0 to 6.0)
    spin_stock = stock_post[stock_post['time'] >= 5.0]['r'].mean()
    spin_relaxed = relaxed_post[relaxed_post['time'] >= 5.0]['r'].mean()

    # Print Pristine KPI Table
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
    print("\n[3/3] Generating publication-quality benchmark plots...")
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    # Color palette
    c_stock = '#d62728'   # Crimson red
    c_rel = '#1f77b4'     # Deep blue
    c_fail = '#ff7f0e'    # Orange dashed line

    fig, axs = plt.subplots(2, 2, figsize=(14, 10))

    # --- Panel 1: Altitude Response z(t) ---
    axs[0, 0].plot(df_stock['time'], df_stock['z'], label='Stock Controller (Crashes)', color=c_stock, linewidth=2.2)
    axs[0, 0].plot(df_relaxed['time'], df_relaxed['z'], label='Relaxed Hover (Stabilized)', color=c_rel, linewidth=2.2)
    axs[0, 0].axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label='Motor 1 Fails (t = 2.0s)')
    axs[0, 0].set_title('Altitude Response $z(t)$', fontsize=13, fontweight='bold')
    axs[0, 0].set_xlabel('Time [s]', fontsize=11)
    axs[0, 0].set_ylabel('Altitude [m]', fontsize=11)
    axs[0, 0].set_ylim(-0.2, 2.5)
    axs[0, 0].legend(loc='lower left', frameon=True)
    axs[0, 0].grid(True, linestyle=':', alpha=0.6)

    # --- Panel 2: Vehicle Tilt Angle theta(t) ---
    axs[0, 1].plot(df_stock['time'], df_stock['tilt'], label='Stock Controller', color=c_stock, linewidth=2.2)
    axs[0, 1].plot(df_relaxed['time'], df_relaxed['tilt'], label='Relaxed Hover', color=c_rel, linewidth=2.2)
    axs[0, 1].axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8)
    axs[0, 1].axhline(y=90.0, color='gray', linestyle=':', label='Inversion (90°)')
    axs[0, 1].set_title('Vehicle Tilt Angle $\\theta(t)$', fontsize=13, fontweight='bold')
    axs[0, 1].set_xlabel('Time [s]', fontsize=11)
    axs[0, 1].set_ylabel('Tilt Angle [deg]', fontsize=11)
    axs[0, 1].legend(loc='upper left', frameon=True)
    axs[0, 1].grid(True, linestyle=':', alpha=0.6)

    # --- Panel 3: Body Angular Rates (Relaxed Hover) ---
    axs[1, 0].plot(df_relaxed['time'], df_relaxed['p'], label='Roll rate $p$', color='#2ca02c', linewidth=1.8)
    axs[1, 0].plot(df_relaxed['time'], df_relaxed['q'], label='Pitch rate $q$', color='#9467bd', linewidth=1.8)
    axs[1, 0].plot(df_relaxed['time'], df_relaxed['r'], label='Yaw rate $r$ (Spin)', color=c_rel, linewidth=2.2)
    axs[1, 0].axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8)
    axs[1, 0].set_title('Body Angular Rates $\\boldsymbol{\\omega}^B(t)$ [Relaxed Hover]', fontsize=13, fontweight='bold')
    axs[1, 0].set_xlabel('Time [s]', fontsize=11)
    axs[1, 0].set_ylabel('Angular Velocity [rad/s]', fontsize=11)
    axs[1, 0].legend(loc='lower left', frameon=True)
    axs[1, 0].grid(True, linestyle=':', alpha=0.6)

    # --- Panel 4: Motor Thrust Allocation (Relaxed Hover) ---
    axs[1, 1].plot(df_relaxed['time'], df_relaxed['f1'], label='$f_1$ (Front - Dead)', color='black', linestyle=':', linewidth=2.0)
    axs[1, 1].plot(df_relaxed['time'], df_relaxed['f2'], label='$f_2$ (Left)', color='#1f77b4', linewidth=1.8)
    axs[1, 1].plot(df_relaxed['time'], df_relaxed['f3'], label='$f_3$ (Rear)', color='#ff7f0e', linewidth=1.8)
    axs[1, 1].plot(df_relaxed['time'], df_relaxed['f4'], label='$f_4$ (Right)', color='#2ca02c', linewidth=1.8)
    axs[1, 1].axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8)
    axs[1, 1].set_title('Motor Thrust Allocation $f_i(t)$ [Relaxed Hover]', fontsize=13, fontweight='bold')
    axs[1, 1].set_xlabel('Time [s]', fontsize=11)
    axs[1, 1].set_ylabel('Thrust Force [N]', fontsize=11)
    axs[1, 1].set_ylim(-0.2, 4.0)
    axs[1, 1].legend(loc='center right', frameon=True)
    axs[1, 1].grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    output_png = 'benchmark_summary.png'
    plt.savefig(output_png, dpi=300)
    print(f"      Saved comprehensive figure to '{output_png}'.")

    # Also save individual high-res figures for modular insertion into report
    fig_alt, ax_alt = plt.subplots(figsize=(7, 4.5))
    ax_alt.plot(df_stock['time'], df_stock['z'], label='Stock Controller (Crashes)', color=c_stock, linewidth=2.2)
    ax_alt.plot(df_relaxed['time'], df_relaxed['z'], label='Relaxed Hover (Stabilized)', color=c_rel, linewidth=2.2)
    ax_alt.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8, label='Motor 1 Fails')
    ax_alt.set_title('Altitude vs Time', fontsize=12, fontweight='bold')
    ax_alt.set_xlabel('Time [s]')
    ax_alt.set_ylabel('Altitude $z$ [m]')
    ax_alt.legend()
    ax_alt.grid(True, linestyle=':', alpha=0.6)
    fig_alt.tight_layout()
    fig_alt.savefig('benchmark_altitude.png', dpi=300)

    fig_rates, ax_rates = plt.subplots(figsize=(7, 4.5))
    ax_rates.plot(df_relaxed['time'], df_relaxed['p'], label='Roll rate $p$', color='#2ca02c', linewidth=1.8)
    ax_rates.plot(df_relaxed['time'], df_relaxed['q'], label='Pitch rate $q$', color='#9467bd', linewidth=1.8)
    ax_rates.plot(df_relaxed['time'], df_relaxed['r'], label='Yaw spin rate $r$', color=c_rel, linewidth=2.2)
    ax_rates.axvline(x=fail_time, color=c_fail, linestyle='--', linewidth=1.8)
    ax_rates.set_title('Body Rates vs Time (Relaxed Hover)', fontsize=12, fontweight='bold')
    ax_rates.set_xlabel('Time [s]')
    ax_rates.set_ylabel('Body Rates [rad/s]')
    ax_rates.legend()
    ax_rates.grid(True, linestyle=':', alpha=0.6)
    fig_rates.tight_layout()
    fig_rates.savefig('benchmark_body_rates.png', dpi=300)

    print("All benchmark plots generated successfully!")

if __name__ == '__main__':
    run_c_simulation()
    df_stock = pd.read_csv('flight_stock.csv')
    df_relaxed = pd.read_csv('flight_relaxed.csv')
    compute_kpis(df_stock, df_relaxed)
    generate_plots(df_stock, df_relaxed)
