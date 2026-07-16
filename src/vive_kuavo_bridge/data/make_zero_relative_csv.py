#!/usr/bin/env python3
import argparse
import csv
import os
import sys

DELTA_COLUMNS = {
    "dsteamvr_x",
    "dsteamvr_y",
    "dsteamvr_z",
    "drobot_x",
    "drobot_y",
    "drobot_z",
}

POSE_COLUMNS = {
    "field.pose.position.x",
    "field.pose.position.y",
    "field.pose.position.z",
    "field.pose.orientation.x",
    "field.pose.orientation.y",
    "field.pose.orientation.z",
    "field.pose.orientation.w",
}

def default_output_path(input_csv):
    root, ext = os.path.splitext(input_csv)
    return root + "_zero" + ext

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("-o", "--output")
    args = parser.parse_args()

    input_csv = args.input_csv
    output_csv = args.output or default_output_path(input_csv)

    with open(input_csv, "r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)

    if not fieldnames:
        print("ERROR: CSV 没有表头", file=sys.stderr)
        return 2
    if not rows:
        print("ERROR: CSV 没有数据行", file=sys.stderr)
        return 2

    first = rows[0]
    fixed_rows = []

    for row in rows:
        new_row = dict(row)

        for name in DELTA_COLUMNS:
            if name in fieldnames:
                new_row[name] = "0.0"

        for name in POSE_COLUMNS:
            if name in fieldnames:
                new_row[name] = first.get(name, row.get(name, ""))

        fixed_rows.append(new_row)

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(fixed_rows)

    print("created:", output_csv)
    print("rows:", len(fixed_rows))
    print("zeroed:", ", ".join([c for c in sorted(DELTA_COLUMNS) if c in fieldnames]))
    print("fixed pose:", ", ".join([c for c in sorted(POSE_COLUMNS) if c in fieldnames]))

if __name__ == "__main__":
    raise SystemExit(main())
