#!/usr/bin/env python3
"""
Post-process explore_area_*.csv files to add area_per_sec column (m^2/s).
Usage: python add_area_per_sec.py <input.csv> [<output.csv>]
If output.csv is not given, overwrites input.csv with new column.
"""
import sys
import csv

if len(sys.argv) < 2:
    print("Usage: python add_area_per_sec.py <input.csv> [<output.csv>]")
    sys.exit(1)

input_path = sys.argv[1]
output_path = sys.argv[2] if len(sys.argv) > 2 else input_path

rows = []
with open(input_path, 'r', newline='') as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames + ['area_per_sec']
    prev_area = None
    prev_time = None
    for row in reader:
        area = float(row['area_m2'])
        t = float(row['elapsed_s'])
        if prev_area is not None and t > prev_time:
            area_per_sec = (area - prev_area) / (t - prev_time)
        else:
            area_per_sec = 0.0
        row['area_per_sec'] = f"{area_per_sec:.4f}"
        rows.append(row)
        prev_area = area
        prev_time = t

with open(output_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

print(f"Wrote area_per_sec to {output_path}")
