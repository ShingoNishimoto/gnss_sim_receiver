import sys
from pathlib import Path

import pandas as pd

if len(sys.argv) != 3:
    print("Usage: python merge_files.py <ecef_txt_file> <eci_csv_file>")
    sys.exit(1)

ecef_file = Path(sys.argv[1])
eci_csv_file = sys.argv[2]

# Read time column
time_df = pd.read_csv(ecef_file, usecols=[0])

# Read full CSV
data_df = pd.read_csv(eci_csv_file, header=None, skiprows=1).reindex()  # the first row is not in ecef log.

# Trim to the shorter (time) length
min_len = len(time_df)
time_df = time_df.iloc[:min_len]
data_df = data_df.iloc[:min_len]

# Combine time as first column
merged = pd.concat([time_df, data_df], axis=1)

# Save to output
output_file = ecef_file.parent / f"user_states_eci_{ecef_file.name[-7:]}"  # chx.txt
merged.to_csv(output_file, index=False, header=['t', 'gmst_sec', 'x', 'y', 'z', 'vx', 'vy', 'vz'], sep=',')  # or sep=',' for CSV style
print(f"Merged file saved to {output_file}")
