import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

log_dir = os.path.join(os.path.dirname(__file__), '../build/logs/actuator-test')

latest_dir = max([os.path.join(log_dir, d) for d in os.listdir(log_dir)], key=os.path.getmtime)

csv_files = [f for f in os.listdir(latest_dir) if f.endswith('.csv')]

"""
csv template:
# actuator-test log
# timestamp, 2026-07-07T11:05:13
# joint_name, l_elbow
# driver, MT_Device
# alias, 1204
# model, X6-60
# operation_mode, OP_PVT
# encoder_bits, 17
# pvt_kp, 40000
# pvt_kd, 5000
# min_counts, -32768
# max_counts, 32768
# start_counts, 0
# min_deg, -90.0000000000
# max_deg, 90.0000000000
# start_deg, 0.0000000000
# trajectory_mode, sin
# sin_centre_counts, 0
# sin_amp_counts, 26214
# sin_centre_deg, 0
# sin_amp_deg, 71.9989
# sin_freq_hz, 0.5
# traj_safety_factor, 0.8
# loop_rate_hz, 1000.0000000000
# lpf_cutoff_hz, 10.0000000000
# approach_seconds, 0.0000000000
# pre_ramp_hold_seconds, 0.5000000000
# max_approach_speed_deg_s, 100.0000000000
# temp_warn_celsius, 60
# temp_abort_celsius, 80
t_s,phase,phase_t_s,ref_raw_counts,ref_filt_counts,actual_counts,ref_raw_deg,ref_filt_deg,actual_deg,motor_temp_c,drive_temp_c,error_code
"""

def plot_csv(csv_file):
    df = pd.read_csv(csv_file, comment='#')
    t_s = df['t_s']
    ref_deg = df['ref_filt_deg']
    actual_deg = df['actual_deg']

    plt.figure(figsize=(10, 6))
    plt.plot(t_s, ref_deg, label='Reference (Filtered)', color='blue')
    plt.plot(t_s, actual_deg, label='Actual', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Degrees')
    plt.title(f'Actuator Test: {os.path.basename(csv_file)}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.plot()
    # save the plot as a PNG file in the same directory as the CSV
    plot_filename = os.path.splitext(csv_file)[0] + '.png'
    plot_path = os.path.join(os.path.dirname(csv_file), plot_filename)
    plt.savefig(plot_path)

for csv_file in csv_files:
    csv_path = os.path.join(latest_dir, csv_file)
    print(f"Plotting {csv_path}...")
    plot_csv(csv_path)

plt.show()