import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

cn0_files = [
    'cn0_ch1.txt',
    'cn0_ch2.txt',
]

min_cn0 = 18
for data_file in cn0_files:
    # Load and downsample
    df = pd.read_csv(data_file)
    df_downsampled = df.iloc[::10].reset_index(drop=True)

    # Set 't' as the x-axis
    x = df_downsampled['t'] - df_downsampled['t'][0]
    t_max = x.values.max()

    # === Plot 1: C/N0 for each channel ===
    plt.figure(figsize=(10, 5))
    for col in df_downsampled.columns[1:]:
        plt.plot(x.values, df_downsampled[col].values, label=col)

    plt.xlim((0, t_max))
    plt.ylim(bottom=min_cn0)
    y_max = np.nanmax(df_downsampled.iloc[:, 1:].values)
    plt.yticks(np.arange(min_cn0, y_max + 1, 3))  # adjust step as needed
    plt.xlabel('Time [s]')
    plt.ylabel('C/N0 [dB-Hz]')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()
    plt.grid(True)
    output_file = os.path.splitext(data_file)[0] + '.png'
    plt.savefig(output_file)
    plt.close()

    # === Plot 2: Number of channels > min_cn0 ===
    above_threshold = (df_downsampled.iloc[:, 1:] > min_cn0).sum(axis=1)

    plt.figure(figsize=(10, 3))
    plt.plot(x.values, above_threshold.values, color='purple')
    plt.xlabel('Time [s]')
    plt.ylabel(f'Channels > {min_cn0} dB-Hz')
    plt.xlim((0, t_max))
    plt.ylim(bottom=0)
    plt.grid(True)
    plt.tight_layout()
    count_plot_file = os.path.splitext(data_file)[0] + '_count.png'
    plt.savefig(count_plot_file)
    plt.close()

    # Create subplots with shared x-axis
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 6), height_ratios=[4, 2])

    # --- Plot 1: C/N0 for each channel ---
    for col in df_downsampled.columns[1:]:
        ax1.plot(x, df_downsampled[col].to_numpy(), label=col)
    ax1.set_ylim(bottom=min_cn0)
    ax1.set_ylabel(f'C/N0 [dB-Hz]')
    ax1.grid(True)
    ax1.legend(loc='upper left', bbox_to_anchor=(1, 1))

    # Optional: force y-axis ticks to start from min_cn0
    ax1.set_yticks(np.arange(min_cn0, y_max + 1, 3))

    # --- Plot 2: Number of channels > min_cn0 ---
    ax2.plot(x, above_threshold, color='purple')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel(f'Channels > {min_cn0} dB-Hz')
    ax2.set_ylim(bottom=0)
    ax2.grid(True)

    # Adjust layout and save
    plt.xlim((0, t_max))
    plt.tight_layout()
    output_file = os.path.splitext(data_file)[0] + '_merged.png'
    plt.savefig(output_file, dpi=300)
    plt.close()