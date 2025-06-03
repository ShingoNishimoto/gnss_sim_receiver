import os

import matplotlib.pyplot as plt
import pandas as pd

cn0_files = [
    'cn0_ch1.txt',
    'cn0_ch2.txt',
]

min_cn0 = 18
for i in range(len(cn0_files)):
    data_file = cn0_files[i]
    df = pd.read_csv(data_file)
    df_downsampled = df.iloc[::10].reset_index(drop=True)

    # Set 't' as the x-axis
    x = df_downsampled['t']

    # Plot all other columns
    plt.figure(figsize=(10, 5))
    for col in df_downsampled.columns[1:]:
        plt.plot(x.values, df_downsampled[col].values, label=col)

    plt.ylim(bottom=min_cn0)
    plt.xlabel('Time [s]')
    plt.ylabel('C/N0 [dB-Hz]')
    # plt.title('Data over Time')
    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()
    plt.grid(True)
    output_file = os.path.splitext(data_file)[0] + '.png'
    plt.savefig(output_file)
    # plt.show()
