#!/usr/bin/env python3
"""Compare benchmark CSVs without machine-dependent pass/fail time thresholds.

  python3 libs/filters/bench/compare.py RESULTS.csv
  python3 libs/filters/bench/compare.py RESULTS.csv --baseline BASELINE.csv

Each Q row is compared to the matching floating row from the *same* run.
An optional historical baseline reports Q-before/Q-now, not a cross-CPU claim.
A speed ratio above 1 means Q/current is faster; a RAM ratio below 1 means
Q/current uses less explicitly counted memory. Stack is not in the CSV.
"""
import argparse
import csv
from pathlib import Path


def rows(path):
    with Path(path).open(newline="") as stream:
        return {(r["family"], r["format"], r["N"], r["n"], r["m"]): r for r in csv.DictReader(stream)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results")
    parser.add_argument("--baseline")
    args = parser.parse_args()
    data = rows(args.results)
    baseline = rows(args.baseline) if args.baseline else {}
    print("family,N,n,m,Q,median_us,float_over_Q_speed,Q_over_float_RAM,Q_vs_float_checksum_delta,baseline_over_current_speed,current_over_baseline_RAM")
    for key, q in data.items():
        family, fmt, count, n, m = key
        if fmt not in ("q15", "q31"):
            continue
        fp = data[(family, "f32" if fmt == "q15" else "f64", count, n, m)]
        q_time, fp_time = float(q["median_ns"]), float(fp["median_ns"])
        q_ram, fp_ram = int(q["total_bytes"]), int(fp["total_bytes"])
        old = baseline.get(key)
        previous_speed = f'{float(old["median_ns"]) / q_time:.4f}' if old else ""
        previous_ram = f'{q_ram / int(old["total_bytes"]):.4f}' if old else ""
        print(f'{family},{count},{n},{m},{fmt},{q_time / 1000:.3f},{fp_time / q_time:.4f},'
              f'{q_ram / fp_ram:.4f},{float(q["checksum"]) - float(fp["checksum"]):.7g},'
              f'{previous_speed},{previous_ram}')


if __name__ == "__main__":
    main()
