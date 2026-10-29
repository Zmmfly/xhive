#!/usr/bin/env python3
"""Check actual integer-kernel execution using a retained GCC benchmark build.

  python3 libs/filters/bench/coverage.py /tmp/filters-xhive-current-...

The input must be a GCC run.py output containing its copied source, driver and
linears_*.o objects. Only filter objects and the driver are rebuilt with GCC -O1
--coverage. This run is NOT a latency measurement. Generated files are retained
in a new /tmp/*-xhive-* directory, and no production files are modified.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    args = parser.parse_args()
    base = args.build.resolve()
    metadata = json.loads((base / "metadata.json").read_text())
    cc = metadata["commands"][0][0]
    cxx = metadata["commands"][-1][0]
    source = base / "source"
    build = Path(tempfile.mkdtemp(prefix="filters-xhive-q-coverage-"))
    includes = [f"-I{source / 'libs/filters/include'}", f"-I{source / 'libs/linears/include'}"]
    flags = ["-O1", "--coverage", "-fno-fast-math"]
    objects = sorted(base.glob("linears_*.o"))
    commands = []
    for name in ("ukf", "pf"):
        obj = build / (name + ".o")
        command = [cc, "-std=c99", *flags, *includes,
                   str(source / "libs/filters/src" / (name + ".c")), "-c", "-o", str(obj)]
        subprocess.run(command, check=True)
        commands.append(command)
        objects.append(obj)
    driver_flags = ["-DFILTERS_BENCH_NONLINEAR=1"] if metadata.get("mode") == "nonlinear" else []
    command = [cxx, "-std=c++17", *flags, *driver_flags, *includes, str(base / "nonlinear.cc"),
               *map(str, objects), "-lm", "-o", str(build / "nonlinear")]
    subprocess.run(command, check=True)
    commands.append(command)
    result = subprocess.run([str(build / "nonlinear")], check=True, capture_output=True, text=True)
    (build / "instrumented-results.csv").write_text(result.stdout)
    result = subprocess.run(["gcov", "-b", "-c", "ukf.gcno", "pf.gcno"], cwd=build,
                            check=True, capture_output=True, text=True)
    (build / "gcov-summary.txt").write_text(result.stdout)
    counts, product_branches = {}, {}
    for name in ("ukf_q_work_impl.h.gcov", "pf_q_internal.h.gcov"):
        function = ""
        collect = False
        for line in (build / name).read_text().splitlines():
            match = re.match(r"function (\S+) called (\d+) returned", line)
            if match:
                function = match.group(1)
                if "_q15_" in function or "_q31_" in function:
                    counts[function] = int(match.group(2))
            # Both forms select the actual integer dot, not just the first
            # screening tier: legacy direct bound or the resolved two-tier gate.
            if ("if (Q_LOCAL(product_fits)" in line or "if (fits)" in line) and function.endswith(("_covariance", "_cross")):
                collect = True
                product_branches[function] = []
            elif collect:
                branch = re.match(r"branch\s+\d+ taken (\d+)", line)
                if branch:
                    product_branches[function].append(int(branch.group(1)))
                    collect = len(product_branches[function]) < 2
    expected = {f"ukf_q_{tag}_{operation}" for tag in ("q15", "q31")
                for operation in ("covariance", "cross")}
    if not expected.issubset(product_branches) or any(len(product_branches[key]) != 2 for key in expected):
        raise RuntimeError("missing integer/floating branch counts; inspect the retained .gcov source")
    commands = json.loads(json.dumps(commands).replace(str(base), "${BENCHMARK_BUILD}")
                          .replace(str(build), "${COVERAGE_BUILD}"))
    evidence = {"method": "GCC -O1 --coverage; counts include warmup and benchmark initialization; not timings",
                "mode": metadata.get("mode", "linear"),
                "source_sha256": metadata["source_sha256"], "driver_sha256": metadata["driver_sha256"],
                "commands": commands, "function_calls": counts,
                "product_fits_branch_counts": product_branches,
                "branch_order": "GCC gcov branch 0 (true/integer), branch 1 (false/local floating); verify against retained .gcov source"}
    (build / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({"function_calls": counts, "product_fits_branch_counts": product_branches}, indent=2))
    print(f"Coverage artifacts retained: {build}")


if __name__ == "__main__":
    main()
