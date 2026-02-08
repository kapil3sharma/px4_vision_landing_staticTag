#!/usr/bin/env python3

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from px4_vision_landing_staticTag.utils.metrics import (
    landing_error,
    tracking_rmse,
    tag_position_stability,
    rmse,
    mean_absolute_error
)

# =========================================================
# CONFIG
# =========================================================
CSV_FILE = "../px4_logs/px4_state_raw_tag_kf_filtered.csv"


# =========================================================
# DATA LOADING
# =========================================================
def load_and_clean(csv_file):
    print(f"[INFO] Loading data from {csv_file}")
    df = pd.read_csv(csv_file)
    df = df.apply(pd.to_numeric, errors="coerce")
    return df


# =========================================================
# METRICS (LOG EVERYTHING)
# =========================================================
def log_all_metrics(df):
    print("\n================ METRICS SUMMARY ================\n")

    # Final landing error
    drone_xy = df[['drone_enu_x','drone_enu_y']].dropna().iloc[-1].values
    tag_xy   = df[['tag_map_x','tag_map_y']].dropna().mean().values
    print(f"[Landing Error] Final XY error: {landing_error(drone_xy, tag_xy):.3f} m")

    # Tracking RMSE (relative pose)
    rel_xyz = df[['rel_x','rel_y','rel_z']].dropna().values
    if len(rel_xyz) > 0:
        r = tracking_rmse(rel_xyz)
        print(f"[Tracking RMSE] X={r[0]:.3f}, Y={r[1]:.3f}, Z={r[2]:.3f}")

    # Tag stability
    raw = df[['tag_map_x','tag_map_y','tag_map_z']].dropna().values
    kf  = df[['tag_map_kf_x','tag_map_kf_y','tag_map_kf_z']].dropna().values

    raw_std = tag_position_stability(raw)
    kf_std  = tag_position_stability(kf)

    print(f"[Tag Stability RAW] std = {raw_std}")
    print(f"[Tag Stability KF ] std = {kf_std}")

    # RMSE & MAE per axis
    for i, axis in enumerate(['X','Y','Z']):
        raw_err = raw[:,i] - np.mean(raw[:,i])
        kf_err  = kf[:,i]  - np.mean(kf[:,i])

        print(f"[{axis} Error] RAW  RMSE={rmse(raw_err):.4f}, MAE={mean_absolute_error(raw_err):.4f}")
        print(f"[{axis} Error] KF   RMSE={rmse(kf_err):.4f}, MAE={mean_absolute_error(kf_err):.4f}")


# =========================================================
# FIGURE 1 — 3D TRAJECTORY
# =========================================================
def plot_trajectory_3d(df):
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    if 'drone_enu_x' in df:
        enu = df[['drone_enu_x','drone_enu_y','drone_enu_z']].dropna()
        ax.plot(enu.iloc[:,0], enu.iloc[:,1], enu.iloc[:,2], label='Drone ENU')

    if 'drone_ned_x' in df:
        ned = df[['drone_ned_x','drone_ned_y','drone_ned_z']].dropna()
        ax.plot(ned.iloc[:,0], ned.iloc[:,1], ned.iloc[:,2], label='Drone NED')

    # if 'rel_x' in df:
    #     rel = df[['rel_x','rel_y','rel_z']].dropna()
    #     ax.plot(rel.iloc[:,0], rel.iloc[:,1], rel.iloc[:,2], label='Relative Pose')

    tag = df[['tag_map_x','tag_map_y','tag_map_z']].dropna()
    ax.scatter(tag.iloc[:,0], tag.iloc[:,1], tag.iloc[:,2], c='red', label='AprilTag (Map)')

    ax.set_title("PX4 Precision Landing – Trajectory Comparison")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# =========================================================
# FIGURE 2 — RAW vs KF 
# =========================================================
def plot_tag_pose_grid(df):
    '''
        4x3 subplot grid

        Rows:
            1 - Raw AprilTag pose
            2 - KF-filtered pose
            3 - Raw vs KF overlay
            4 - Estimation error (Raw - KF)

        Columns:
            X, Y, Z
    '''
    t = df['time']

    raw_cols = ['tag_map_x', 'tag_map_y', 'tag_map_z']
    kf_cols  = ['tag_map_kf_x', 'tag_map_kf_y', 'tag_map_kf_z']
    axes_lbl = ['X', 'Y', 'Z']

    fig, ax = plt.subplots(4, 3, figsize=(18, 12), sharex=True)
    fig.suptitle("AprilTag Pose Estimation: Raw vs Kalman Filtered (Map Frame)", fontsize=16)

    for i in range(3):
        raw = df[raw_cols[i]]
        kf  = df[kf_cols[i]]

        # Row 1 — Raw
        ax[0, i].plot(t, raw)
        ax[0, i].set_title(f"{axes_lbl[i]} (m)")
        ax[0, i].set_ylabel("Raw")

        # Row 2 — KF
        ax[1, i].plot(t, kf)
        ax[1, i].set_ylabel("KF")

        # Row 3 — Overlay
        ax[2, i].plot(t, raw, linestyle=':', alpha=0.6, label='Raw')
        ax[2, i].plot(t, kf, linewidth=2, label='KF')
        ax[2, i].set_ylabel("Raw vs KF")

        if i == 0:
            ax[2, i].legend()

        # Row 4 — Error (Raw − KF)
        ax[3, i].plot(t, raw - kf)
        ax[3, i].axhline(0.0, color='k', linewidth=0.8)
        ax[3, i].set_ylabel("Error (Raw − KF)")

    # Grid everywhere
    for a in ax.flat:
        a.grid(True)

    # X label only on last row
    for i in range(3):
        ax[3, i].set_xlabel("Time [s]")

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


# =========================================================
# FIGURE 3 — ALTITUDE + LANDING ERRORS
# =========================================================
def plot_landing_performance(df):
    drone_xy = df[['drone_enu_x','drone_enu_y']]
    tag_raw  = df[['tag_map_x','tag_map_y']]
    tag_kf   = df[['tag_map_kf_x','tag_map_kf_y']]

    valid = pd.concat(
        [
            df['time'],
            df['drone_enu_z'],
            drone_xy,
            tag_raw,
            tag_kf
        ],
        axis=1
    ).dropna()

    t = valid['time']

    err_raw = np.linalg.norm(
        valid[['drone_enu_x','drone_enu_y']].values -
        valid[['tag_map_x','tag_map_y']].values,
        axis=1
    )

    err_kf = np.linalg.norm(
        valid[['drone_enu_x','drone_enu_y']].values -
        valid[['tag_map_kf_x','tag_map_kf_y']].values,
        axis=1
    )

    fig, ax = plt.subplots(3,1, figsize=(10,9), sharex=True)
    fig.suptitle("Landing Performance Overview")

    ax[0].plot(t, valid['drone_enu_z'])
    ax[0].set_ylabel("Altitude [m]")

    ax[1].plot(t, err_raw)
    ax[1].set_ylabel("Landing Error (Raw)")

    ax[2].plot(t, err_raw, linestyle=':', label='Raw')
    ax[2].plot(t, err_kf, label='KF')
    ax[2].set_ylabel("Raw vs KF Error")
    ax[2].legend()

    for a in ax:
        a.grid(True)

    plt.tight_layout()
    plt.show()


# =========================================================
# FIGURE 4 — KF DELAY ANALYSIS
# =========================================================
def plot_kf_delay_analysis(df, axis='x'):
    '''
        KF delay analysis using numpy-only cross-correlation.
        No SciPy dependency.
    '''
    col_raw = f'tag_map_{axis}'
    col_kf = f'tag_map_kf_{axis}'
    
    valid = df[['time', col_raw, col_kf]].dropna()
    t = valid['time'].values
    raw = valid[col_raw].values
    kf = valid[col_kf].values
    
    # Ensure same length
    min_len = min(len(raw), len(kf), len(t))
    if min_len < 10:
        print(f"Warning: Not enough data points for {axis} axis ({min_len})")
        return 0.0
    
    t = t[:min_len]
    raw = raw[:min_len]
    kf = kf[:min_len]
    
    # Calculate sampling period
    if len(t) > 1:
        dt = np.mean(np.diff(t))
    else:
        dt = 0.05  # Assume 20Hz
    
    # Method 1: Simple peak-to-peak delay
    # Find peaks in both signals
    def find_simple_peaks(signal, threshold=0.001):
        peaks = []
        for i in range(1, len(signal)-1):
            if signal[i] > signal[i-1] and signal[i] > signal[i+1]:
                if signal[i] > np.mean(signal) + threshold:
                    peaks.append(i)
        return np.array(peaks)
    
    peaks_raw = find_simple_peaks(raw)
    peaks_kf = find_simple_peaks(kf)
    
    delays = []
    if len(peaks_raw) > 0 and len(peaks_kf) > 0:
        # Match nearest peaks
        for i in range(min(len(peaks_raw), len(peaks_kf))):
            time_raw = t[peaks_raw[i]]
            time_kf = t[peaks_kf[i]]
            delays.append(time_raw - time_kf)
    
    # Method 2: Cross-correlation using numpy
    def numpy_xcorr(x, y):
        '''
            Simple cross-correlation using numpy
        '''
        n = len(x)
        # Normalize
        x_norm = (x - np.mean(x)) / np.std(x)
        y_norm = (y - np.mean(y)) / np.std(y)
        
        corr = np.correlate(x_norm, y_norm, mode='full')
        lags = np.arange(-n + 1, n)
        return corr, lags
    
    correlation, lags = numpy_xcorr(raw, kf)
    max_corr_idx = np.argmax(np.abs(correlation))
    delay_samples_xcorr = lags[max_corr_idx]
    delay_seconds_xcorr = delay_samples_xcorr * dt
    
    # Choose the most reliable delay
    if delays:
        avg_delay = np.mean(delays)
        std_delay = np.std(delays)
        if std_delay < 0.1:  # If consistent
            delay_seconds = avg_delay
            method = "peak matching"
        else:
            delay_seconds = delay_seconds_xcorr
            method = "cross-correlation"
    else:
        delay_seconds = delay_seconds_xcorr
        method = "cross-correlation"
    
    print(f"\n=== KF Delay Analysis ({axis.upper()} axis) ===")
    print(f"Method: {method}")
    print(f"Time delay: {delay_seconds:.3f} seconds")
    print(f"Sampling period: {dt:.3f} seconds ({1/dt:.1f} Hz)")
    print(f"Delay direction: {'RAW leads KF' if delay_seconds > 0 else 'KF leads RAW'}")
    
    # Simple plot (2 rows)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(f"KF Delay Analysis - {axis.upper()} Axis\nDelay: {abs(delay_seconds):.3f}s ({'RAW leads' if delay_seconds > 0 else 'KF leads'})", fontsize=14)
    
    # 1. Raw vs KF
    axes[0].plot(t, raw, 'b-', alpha=0.7, label='Raw', linewidth=1)
    axes[0].plot(t, kf, 'r-', alpha=0.7, label='KF', linewidth=1)
    
    # Mark peaks if available
    if len(peaks_raw) > 0 and len(peaks_kf) > 0:
        axes[0].plot(t[peaks_raw], raw[peaks_raw], 'bo', markersize=4, label='Raw peaks')
        axes[0].plot(t[peaks_kf], kf[peaks_kf], 'ro', markersize=4, label='KF peaks')
    
    axes[0].set_ylabel('Position [m]')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Raw vs KF Filtered Pose')
    
    # 2. Cross-correlation
    axes[1].plot(lags * dt, correlation, 'g-', linewidth=1)
    axes[1].axvline(delay_seconds_xcorr, color='red', linestyle='--', 
                   linewidth=2, label=f'Max correlation at {delay_seconds_xcorr:.3f}s')
    axes[1].axvline(0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    axes[1].set_xlabel('Time Lag [s]')
    axes[1].set_ylabel('Correlation')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Cross-Correlation')
    
    plt.tight_layout()
    plt.show()
    
    return delay_seconds


# =========================================================
# MAIN
# =========================================================
def main():
    df = load_and_clean(CSV_FILE)
    log_all_metrics(df)

    plot_trajectory_3d(df)
    plot_tag_pose_grid(df)
    plot_landing_performance(df)
    
    # Proper delay analysis
    delay_x = plot_kf_delay_analysis(df, axis='x')
    delay_y = plot_kf_delay_analysis(df, axis='y')
    delay_z = plot_kf_delay_analysis(df, axis='z')

    print(f"\n=== SUMMARY ===")
    print(f"X-axis delay: {delay_x:.3f}s")
    print(f"Y-axis delay: {delay_y:.3f}s") 
    print(f"Z-axis delay: {delay_z:.3f}s")
    print(f"Average delay: {(delay_x + delay_y + delay_z)/3:.3f}s")

if __name__ == "__main__":
    main()
