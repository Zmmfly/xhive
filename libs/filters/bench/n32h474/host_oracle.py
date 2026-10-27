#!/usr/bin/env python3
"""Check the exact board workload off-board, without Xmake or hardware access.

Compile all four HOST_BENCH formats, run every case twice, and compare every
changed state/output element to the original bench/nonlinear.cc driver. The
reference driver retains its original inputs, RNG, init, reset and transaction;
only its main/measurement loop is replaced in a temporary source copy to export
one complete result per case. This is driver-equivalence validation, not an
independent mathematical oracle for the production algorithms.

Artifacts and immutable production-source copies remain in /tmp/*-xhive-*.
No packages are installed and no repository build/configuration state is used.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile


DRAW_CHECK = r'''
#include "draws.h"
#include <cmath>
#include <cstdio>
#include <random>
int main() {
    std::mt19937 random(0x51f15u);
    std::normal_distribution<double> normal(0.0, 1.0);
    for (std::size_t i = 0; i < filters_bench::normal_draw_count; ++i) {
        const double expected = std::round(normal(random) * 32768.0) / 32768.0;
        if (filters_bench::normal_draws<double>[i] != expected
            || filters_bench::normal_draws<float>[i] != expected) {
            std::fprintf(stderr, "draw mismatch at %zu; this generator is not the frozen libstdc++ sequence\n", i);
            return 1;
        }
    }
    std::puts("512 Gaussian draws match exactly in float and double");
}
'''

REFERENCE_MAIN = r'''
namespace {
#ifdef FILTERS_BENCH_NONLINEAR
constexpr unsigned nonlinear = 1;
#else
constexpr unsigned nonlinear = 0;
#endif
std::FILE *output;
template<class F> void emit(unsigned format, unsigned family, std::size_t count, std::size_t n,
                             const char *name, const std::vector<typename F::Raw> &values) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        uint64_t bits = 0;
        std::memcpy(&bits, values.data() + i, sizeof(typename F::Raw));
        const double value = F::decode(values[i]);
        if (!std::isfinite(value)) fail("nonfinite reference output", LINEARS_ERROR_RANGE);
        std::fprintf(output, "%u,%u,%u,%zu,%zu,%s,%zu,%016llx,%.17g\n",
                     format, nonlinear, family, n, count, name, i,
                     static_cast<unsigned long long>(bits), value);
    }
}
template<class F> void emit_ukf(unsigned format, std::size_t n) {
    Inputs<F> input(n);
    Workspace<F> ws(ukf_storage<F>(n));
    UkfLane<F> lane(n, input);
    lane.reset(input);
    checked(lane.run(input, ws), "reference UKF");
    emit<F>(format, 0, 1, n, "estimate", lane.x.data);
    emit<F>(format, 0, 1, n, "covariance", lane.p.data);
}
template<class F, bool Unscented> void emit_particles(unsigned format, std::size_t count, std::size_t n) {
    Inputs<F> input(n);
    Workspace<F> ws(Unscented ? upf_storage<F>(count, n) : pf_storage<F>(count, n));
    ParticleLane<F, Unscented> lane(count, n, input, ws);
    lane.reset(input);
    checked(lane.run(input, ws), "reference particles");
    constexpr unsigned family = Unscented ? 2 : 1;
    emit<F>(format, family, count, n, "particles", lane.particles.data);
    emit<F>(format, family, count, n, "weights", lane.weights.data);
    if constexpr (Unscented) emit<F>(format, family, count, n, "covariances", lane.covariances.data);
    else {
        emit<F>(format, family, count, n, "estimate", lane.mean.data);
        emit<F>(format, family, count, n, "covariance", lane.covariance.data);
    }
}
template<class F> void format_cases(unsigned format) {
    for (std::size_t n : {2u, 4u}) {
        emit_ukf<F>(format, n);
        for (std::size_t count : {32u, 128u}) {
            emit_particles<F, false>(format, count, n);
            emit_particles<F, true>(format, count, n);
        }
    }
}
}
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    output = std::fopen(argv[1], "w");
    if (!output) return 1;
    std::fputs("format,nonlinear,family,n,particles,array,index,raw_hex,decoded\n", output);
    format_cases<F32>(0); format_cases<F64>(1); format_cases<Q15>(2); format_cases<Q31>(3);
    const bool error = std::ferror(output) != 0;
    const int closed = std::fclose(output);
    return error || closed != 0;
}
'''


def read_elements(paths):
    elements = {}
    for path in paths:
        with path.open() as stream:
            for row in csv.DictReader(stream):
                key = tuple(int(row[field]) for field in ("format", "nonlinear", "family", "n", "particles"))
                key += (row["array"], int(row["index"]))
                if key in elements:
                    raise RuntimeError(f"Duplicate output element: {key}")
                value = float(row["decoded"])
                if not math.isfinite(value):
                    raise RuntimeError(f"Nonfinite output: {key}")
                elements[key] = (row["raw_hex"], value)
    return elements


def compare(reference, actual, build):
    if reference.keys() != actual.keys():
        raise RuntimeError(f"Output coverage differs: missing={len(reference.keys() - actual.keys())}, "
                           f"extra={len(actual.keys() - reference.keys())}")
    cases = {}
    for key, (expected_bits, expected) in reference.items():
        bits, value = actual[key]
        metrics = cases.setdefault(key[:5], {"elements": 0, "raw_mismatches": 0, "max_abs_error": 0.0})
        metrics["elements"] += 1
        metrics["raw_mismatches"] += bits != expected_bits
        metrics["max_abs_error"] = max(metrics["max_abs_error"], abs(value - expected))
    with (build / "equivalence.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["format", "nonlinear", "family", "n", "particles", "elements", "raw_mismatches", "max_abs_error"])
        for key, metrics in sorted(cases.items()):
            writer.writerow([*key, *metrics.values()])
    mismatches = sum(metrics["raw_mismatches"] for metrics in cases.values())
    if len(cases) != 80 or mismatches:
        raise RuntimeError(f"Driver comparison failed: {len(cases)} cases, {mismatches} differing raw elements")
    print(f"PASS: {len(cases)} cases, {len(actual)} raw output elements exactly match nonlinear.cc")
    return {"cases": len(cases), "elements": len(actual), "raw_mismatches": mismatches}


def pair_errors(elements, build):
    """Report Q-vs-floating drift; normalize each format's weight convention."""
    arrays = {}
    for key, (_, value) in sorted(elements.items()):
        arrays.setdefault(key[:6], []).append(value)
    with (build / "quantization.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["q_format", "reference_format", "nonlinear", "family", "n", "particles", "array",
                         "elements", "max_abs_error", "rms_error", "weight_normalized"])
        for key, values in sorted(arrays.items()):
            if key[0] not in (2, 3):
                continue
            reference_format = key[0] - 2
            other = arrays[(reference_format, *key[1:])]
            weights = key[-1] == "weights"
            if weights:
                q_sum, f_sum = sum(values), sum(other)
                values = [value / q_sum for value in values]
                other = [value / f_sum for value in other]
            errors = [abs(a - b) for a, b in zip(values, other)]
            writer.writerow([key[0], reference_format, *key[1:], len(errors), max(errors),
                             math.sqrt(sum(error * error for error in errors) / len(errors)), int(weights)])


def failure_adapter(fmt):
    """Substitute only driver API pointers, leaving production objects intact."""
    tag = ("f32", "f64", "q15", "q31")[fmt]
    raw = ("float", "double", "int16_t", "int32_t")[fmt]
    operations = ("ukf_init", "ukf_predict", "ukf_update", "pf_init", "pf_predict", "pf_update",
                  "pf_estimate", "upf_init", "upf_step")
    code = r'''
#include "filters_upf.h"
#include <limits>
#include <tuple>
#include <type_traits>
using FaultRaw = RAW_TYPE;
int selected_operation, attempts, fail_at, fault_mode;
template<int Operation, auto Function> struct Inject;
template<int Operation, class... Args, linears_status_t (*Function)(Args...)>
struct Inject<Operation, Function> {
    static linears_status_t call(Args... args) {
        const bool selected = selected_operation == Operation;
        if (selected && ++attempts == fail_at && fault_mode == 0) return LINEARS_ERROR_WORKSPACE;
        const auto status = Function(args...);
        if constexpr (Operation == 2) {
            if (selected && attempts == fail_at && status == LINEARS_OK && fault_mode != 0) {
                auto *state = std::get<0>(std::tuple<Args...>(args...));
                auto *out = static_cast<FaultRaw *>(state->estimate.data);
                if (fault_mode == 1) *out += FaultRaw(1);
                else *out = std::numeric_limits<FaultRaw>::quiet_NaN();
            }
        }
        return status;
    }
};
'''.replace("RAW_TYPE", raw)
    for index, operation in enumerate(operations):
        family, action = operation.split("_", 1)
        name = f"filters_{family}_{tag}_{action}"
        code += f"#define {name} Inject<{index}, &{name}>::call\n"
    code += r'''
#define HOST_BENCH 1
#define main normal_host_main
#include "workload.cc"
#undef main
int main() {
    constexpr int case_index[] = {0, 0, 0, 1, 1, 1, 1, 2, 2};
    const int tests = std::is_floating_point<FaultRaw>::value ? 11 : 10;
    for (int test = 0; test < tests; ++test) {
        selected_operation = test < 9 ? test : 2;
        fault_mode = test < 9 ? 0 : test - 8;
        attempts = 0;
        const bool init = selected_operation == 0 || selected_operation == 3 || selected_operation == 7;
        fail_at = init ? 1 : 5;  // Two valid retained trials precede a runtime failure.
        const int expected_status = fault_mode == 0 ? LINEARS_ERROR_WORKSPACE
                                  : fault_mode == 1 ? LINEARS_ERROR_NO_CONVERGENCE : LINEARS_ERROR_RANGE;
        const auto index = static_cast<uint32_t>(case_index[selected_operation]);
        host_marker_pairs = 0;
        host_marker_error = false;
        bench_run();
        const auto &record = g_bench_results.cases[index];
        bool valid = g_bench_results.phase == 4 && g_bench_results.active_case == index
                  && g_bench_results.case_count == index + 1 && record.status == expected_status
                  && record.checksum_lo == 0 && record.checksum_hi == 0 && record.output_hash == 0
                  && attempts == fail_at && !host_marker_error && host_active_marker == UINT32_MAX
                  && host_marker_pairs == index * (warmups + BENCH_TRIALS) + (init ? 0 : 5);
        for (auto cycles : record.cycles) valid = valid && cycles == 0;
        if (!valid) {
            std::fprintf(stderr, "fault test %d failed: phase=%u case=%u status=%d pairs=%u attempts=%d\n",
                         test, g_bench_results.phase, g_bench_results.active_case,
                         record.status, host_marker_pairs, attempts);
            return 1;
        }
    }
    std::printf("PASS: format %u, %d API/validation failure-publication checks\n", BENCH_FORMAT, tests);
}
'''
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--revision", help="export production sources from this commit; default copies the working tree")
    parser.add_argument("--sanitize", action="store_true", help="enable host AddressSanitizer and UndefinedBehaviorSanitizer")
    args = parser.parse_args()
    if sys.byteorder != "little":
        parser.error("The raw-output mailbox oracle requires a little-endian host")
    driver_dir = Path(__file__).resolve().parent
    repo = driver_dir.parents[3]
    build = Path(tempfile.mkdtemp(prefix="filters-xhive-board-oracle-"))
    print(f"Artifacts retained: {build}", flush=True)
    commands = []

    def run(command, **kwargs):
        command = list(map(str, command))
        commands.append(command)
        print("+", shlex.join(command), flush=True)
        try:
            return subprocess.run(command, check=True, **kwargs)
        except subprocess.CalledProcessError as error:
            if error.stdout:
                print(error.stdout, file=sys.stderr)
            if error.stderr:
                print(error.stderr, file=sys.stderr)
            raise

    revision = run(["git", "-C", repo, "rev-parse", args.revision or "HEAD"], capture_output=True, text=True).stdout.strip()
    source = build / "source"
    if args.revision:
        source.mkdir()
        archive = build / "source.tar"
        with archive.open("wb") as stream:
            run(["git", "-C", repo, "archive", revision, "libs/filters/include", "libs/filters/src",
                 "libs/linears/include", "libs/linears/src"], stdout=stream)
        run(["tar", "-xf", archive, "-C", source])
    else:
        for lib in ("filters", "linears"):
            for folder in ("include", "src"):
                shutil.copytree(repo / "libs" / lib / folder, source / "libs" / lib / folder)
    drivers = build / "driver"
    drivers.mkdir()
    for name in ("workload.cc", "draws.h", "bench_platform.h", "bench_result.h"):
        shutil.copy2(driver_dir / name, drivers / name)
    original = driver_dir.parent / "nonlinear.cc"
    original_text = original.read_text()
    (drivers / "nonlinear.cc").write_text(original_text)
    if original_text.count("int main() {") != 1:
        raise RuntimeError("Original nonlinear.cc main changed; review the reference export adapter")
    reference = drivers / "reference.cc"
    reference.write_text(original_text.split("int main() {", 1)[0] + REFERENCE_MAIN)
    draw_check = drivers / "draw_check.cc"
    draw_check.write_text(DRAW_CHECK)
    flags = ["-O1" if args.sanitize else "-O3", "-g", "-fno-fast-math", "-ffp-contract=off",
             "-ffunction-sections", "-fdata-sections"]
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-fno-omit-frame-pointer", "-fno-pie"]
    includes = [f"-I{source / 'libs/filters/include'}", f"-I{source / 'libs/linears/include'}"]
    link = ["-Wl,--gc-sections", "-lm"] + (["-no-pie"] if args.sanitize else [])
    draw_binary = build / "draw_check"
    run([args.cxx, "-std=c++17", *flags, draw_check, *link, "-o", draw_binary])
    run([draw_binary])
    sources = sorted((source / "libs/linears/src").glob("*.c"))
    sources += [source / "libs/filters/src/ukf.c", source / "libs/filters/src/pf.c"]
    objects = []
    for item in sources:
        obj = build / (item.parent.parent.name + "_" + item.stem + ".o")
        run([args.cc, "-std=c99", *flags, *includes, "-c", item, "-o", obj])
        objects.append(obj)
    summaries = []
    actual_paths = []
    for fmt in range(4):
        binary = build / f"host_{fmt}"
        run([args.cxx, "-std=c++17", *flags, "-Wall", "-Wextra", "-Werror", "-fno-exceptions", "-fno-rtti",
             f"-DBENCH_FORMAT={fmt}", "-DHOST_BENCH=1", *includes, drivers / "workload.cc", *objects, *link, "-o", binary])
        output = build / f"host_{fmt}_elements.csv"
        result = run([binary, output], capture_output=True, text=True)
        (build / f"host_{fmt}.csv").write_text(result.stdout)
        (build / f"host_{fmt}.stderr").write_text(result.stderr)
        records = list(csv.DictReader(io.StringIO("\n".join(line for line in result.stdout.splitlines() if not line.startswith("#")))))
        if len(records) != 20:
            raise RuntimeError(f"Format {fmt}: expected 20 summary records, got {len(records)}")
        print(result.stdout.splitlines()[0], flush=True)
        summaries.extend(records)
        actual_paths.append(output)
        adapter = drivers / f"failure_{fmt}.cc"
        adapter.write_text(failure_adapter(fmt))
        failure_binary = build / f"failure_{fmt}"
        run([args.cxx, "-std=c++17", *flags, "-Wall", "-Wextra", "-Werror", f"-DBENCH_FORMAT={fmt}",
             *includes, adapter, *objects, *link, "-o", failure_binary])
        run([failure_binary])
    reference_paths = []
    for nonlinear in range(2):
        binary = build / f"reference_{nonlinear}"
        macros = ["-DFILTERS_BENCH_NONLINEAR=1"] if nonlinear else []
        run([args.cxx, "-std=c++17", *flags, *macros, *includes, reference, *objects, *link, "-o", binary])
        output = build / f"reference_{nonlinear}_elements.csv"
        run([binary, output])
        reference_paths.append(output)
    actual = read_elements(actual_paths)
    reference_elements = read_elements(reference_paths)
    comparison = compare(reference_elements, actual, build)
    pair_errors(actual, build)
    metadata = {
        "revision": revision,
        "source_mode": "git-archive" if args.revision else "working-tree-copy",
        "sanitizers": args.sanitize,
        "compiler": run([args.cc, "--version"], capture_output=True, text=True).stdout,
        "cxx_compiler": run([args.cxx, "--version"], capture_output=True, text=True).stdout,
        "commands": commands,
        "comparison": comparison,
        "failure_checks": 42,
        "summaries": summaries,
        "sha256": {str(path.relative_to(build)): hashlib.sha256(path.read_bytes()).hexdigest()
                   for root in (source, drivers) for path in sorted(root.rglob("*")) if path.is_file()},
        "limits": "Host-only driver equivalence; no MCU timing, no independent production math oracle. "
                  "quantization.csv reports, but does not impose a new Q-vs-float accuracy threshold.",
    }
    (build / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"PASS: 512 shared draws, 4 formats x 20 cases x 2 runs, 42 failure checks; full results and comparisons in {build}")


if __name__ == "__main__":
    main()
