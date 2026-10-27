#!/usr/bin/env python3
"""Compile and run the nonlinear host benchmark without touching Xmake state.

Examples (from any directory):
  python3 libs/filters/bench/run.py --revision 470206b --label baseline
  python3 libs/filters/bench/run.py --cc clang --cxx clang++ --label current
  python3 libs/filters/bench/run.py --nonlinear --label current-nonlinear

All generated files live in a retained /tmp/*-xhive-* directory printed at
completion. --revision exports immutable source with git archive; only production
sources come from that revision, and the benchmark driver is always this copy.
No compiler/package downloads, privileged changes or hardware accesses occur.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile


def run(command, **kwargs):
    print("+", shlex.join(map(str, command)), flush=True)
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision", help="build production sources from this Git revision")
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--label", default="current")
    parser.add_argument("--nonlinear", action="store_true", help="add small quadratic terms to transition/observation")
    parser.add_argument("--cpu", type=int, help="pin the timed process to one permitted Linux CPU")
    args = parser.parse_args()
    driver = Path(__file__).resolve().with_name("nonlinear.cc")
    repo = driver.parents[3]
    build = Path(tempfile.mkdtemp(prefix=f"filters-xhive-{args.label}-"))
    source = repo
    revision = run(["git", "-C", repo, "rev-parse", args.revision or "HEAD"],
                   capture_output=True, text=True).stdout.strip()
    if args.revision:
        source = build / "source"
        source.mkdir()
        archive = build / "source.tar"
        with archive.open("wb") as output:
            run(["git", "-C", repo, "archive", revision, "libs/filters", "libs/linears"], stdout=output)
        run(["tar", "-xf", archive, "-C", source])
    else:
        source = build / "source"
        for lib in ("filters", "linears"):
            for folder in ("include", "src"):
                shutil.copytree(repo / "libs" / lib / folder, source / "libs" / lib / folder)
    flags = ["-O3", "-DNDEBUG", "-fno-fast-math", "-ffunction-sections", "-fdata-sections"]
    includes = [f"-I{source / 'libs/filters/include'}", f"-I{source / 'libs/linears/include'}"]
    sources = sorted((source / "libs/linears/src").glob("*.c"))
    sources += [source / "libs/filters/src/ukf.c", source / "libs/filters/src/pf.c"]
    objects = []
    commands = []
    for item in sources:
        obj = build / (item.parent.parent.name + "_" + item.stem + ".o")
        command = [args.cc, "-std=c99", *flags, *includes, "-c", item, "-o", obj]
        commands.append(list(map(str, command)))
        run(command)
        objects.append(obj)
    binary = build / "nonlinear"
    driver_flags = ["-DFILTERS_BENCH_NONLINEAR=1"] if args.nonlinear else []
    command = [args.cxx, "-std=c++17", *flags, *driver_flags, *includes, driver, *objects,
               "-Wl,--gc-sections", "-lm", "-o", binary]
    commands.append(list(map(str, command)))
    run(command)
    cpu = args.cpu
    if cpu is None and hasattr(os, "sched_getaffinity"):
        cpu = min(os.sched_getaffinity(0))
    command = (["taskset", "-c", str(cpu)] if cpu is not None else []) + [str(binary)]
    print("+", shlex.join(command), flush=True)
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    (build / "results.csv").write_text(result.stdout)
    (build / "stderr.txt").write_text(result.stderr)
    metadata = {
        "revision": revision,
        "source_mode": "git-archive" if args.revision else "working-tree-copy",
        "working_tree_status": run(["git", "-C", repo, "status", "--short"], capture_output=True, text=True).stdout,
        "compiler": run([args.cc, "--version"], capture_output=True, text=True).stdout,
        "cxx_compiler": run([args.cxx, "--version"], capture_output=True, text=True).stdout,
        "cpu": run(["lscpu"], capture_output=True, text=True, env={**os.environ, "LC_ALL": "C"}).stdout,
        "affinity_cpu": cpu,
        "mode": "nonlinear" if args.nonlinear else "linear",
        "commands": commands,
        "run_command": command,
        "exit_code": result.returncode,
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "source_sha256": {str(item.relative_to(source)): hashlib.sha256(item.read_bytes()).hexdigest()
                          for lib in ("filters", "linears")
                          for folder in ("include", "src")
                          for item in sorted((source / "libs" / lib / folder).iterdir()) if item.is_file()},
        "method": {
            "clock": "std::chrono::steady_clock",
            "timed_transactions_per_sample": 16 * 12,
            "warmup_samples": 2,
            "reported_samples": 9,
            "inputs": "n=m, dyadic x/P/Q/R/z; alpha=1 beta=2 kappa=0; identical q15-grid starting particles",
            "model": "f_j=.875*x_j+.0625*x_next; h_j=.75*x_j+.125*x_next"
                     + ("; add .03125*x_j^2 to f and .0625*x_j^2 to h" if args.nonlinear else ""),
            "draws": "mt19937 seed 0x51f15 + libstdc++ normal_distribution<double>, precomputed and q15-grid rounded",
            "timed": "successful whole transactions including native callbacks, staged commits and validation",
            "excluded": "allocation/init, initial-state restoration, normal RNG generation, output checks",
            "memory": "one instance: persistent arrays + state descriptor + max per-operation reusable workspace + PF estimate outputs; excludes stack, inputs, callback state, allocator overhead and benchmark lanes",
            "limits": "host wall-time only; no MCU execution; Linux scheduling/turbo variation is not controlled",
        },
    }
    # Portable provenance: avoid embedding local checkout or temporary paths.
    for key in ("commands", "run_command"):
        metadata[key] = json.loads(json.dumps(metadata[key]).replace(str(source), "${SOURCE}")
                                   .replace(str(repo), "${REPO}").replace(str(build), "${BUILD}"))
    (build / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (build / "nonlinear.cc").write_bytes(driver.read_bytes())
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    print(f"Artifacts retained: {build}")
    if result.returncode:
        raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
