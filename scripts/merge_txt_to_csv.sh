#!/bin/bash

# Input files (in desired column order)
files=("dt0.txt" "dt0_current.txt" "dt_current.txt" "dt_gnssr_aowr.txt" "dt_gnssr_aowr_cp.txt")

# Output CSV file
output="merged_dt_data.csv"

# Header row (remove .txt from each file)
header=""
for f in "${files[@]}"; do
    name="${f%.txt}"
    header+="$name,"
done
header=${header%,}  # remove trailing comma

# Write header to output file
echo "$header" > "$output"

# Use paste to merge files line by line, separating with commas
paste -d ',' "${files[@]}" >> "$output"

echo "Summary written to $output"
