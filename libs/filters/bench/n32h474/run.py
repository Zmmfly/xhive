#!/usr/bin/env python3
"""Run one explicitly authorized N32H474 benchmark image over J-Link.

Requires an analyzer-only board, no power-stage or actuator loads. Flashing is
never implicit: pass --flash to overwrite the application flash with the given
ELF. All commands and raw SWD results are retained. No credentials are used.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct
import subprocess
import tempfile

FORMATS = ("f32", "f64", "q15", "q31")
FAMILIES = ("ukf_predict_update", "pf_predict_update_estimate", "upf_step")
HEADER_WORDS, CASE_WORDS, MAX_CASES = 12, 21, 20
MAILBOX_BYTES = 4 * (HEADER_WORDS + MAX_CASES * CASE_WORDS)


def parse_words(text):
    words = {}
    for line in text.splitlines():
        match = re.fullmatch(r"([0-9A-Fa-f]{8}) =\s*((?:[0-9A-Fa-f]{8}\s*)+)", line)
        if match:
            address = int(match[1], 16)
            for i, value in enumerate(match[2].split()):
                words[address + i * 4] = int(value, 16)
    return words


class Probe:
    def __init__(self, executable, directory):
        self.executable, self.directory, self.sequence = executable, directory, 0

    def execute(self, commands, label):
        self.sequence += 1
        base = self.directory / f"{self.sequence:03d}-{label}"
        script = base.with_suffix(".jlink")
        script.write_text("\n".join(commands) + "\nexit\n")
        command = [str(self.executable), "-device", "N32H474RC", "-if", "SWD",
                   "-speed", "4000", "-autoconnect", "1", "-CommandFile", str(script)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        text = result.stdout + result.stderr
        base.with_suffix(".log").write_text(text)
        if result.returncode:
            raise RuntimeError(f"J-Link exit {result.returncode}; see {base}.log")
        return text

    def read(self, address, count):
        text = self.execute([f"mem32 0x{address:08X} 0x{count:X}"], "read")
        words = parse_words(text)
        return [words[address + 4 * i] for i in range(count)]

    def start(self, mailbox, quiet_seconds):
        if not 0 < quiet_seconds <= 24:
            raise ValueError("quiet window must be in (0,24] seconds for the bounded J-Link operation")
        # This rig's connect/disconnect sequence left CYCCNT stopped despite
        # CTRL bit 0 reading as set. Rearm while halted, then retain the probe
        # connection without SWD polling through the entire transaction suite.
        return self.execute(["h", "w4 0xE000EDFC 0x01000000", "w4 0xE0001000 0",
                             "w4 0xE0001004 0", "w4 0xE0001000 1",
                             f"w4 0x{mailbox + 16:08X} 1", "g",
                             f"sleep {int(quiet_seconds * 1000)}"], "start")


def symbol(elf, name, nm):
    output = subprocess.check_output([nm, "-S", str(elf)], text=True)
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3] == name:
            return int(fields[0], 16), int(fields[1], 16)
    raise RuntimeError(f"missing ELF symbol {name}")


def decode(blob):
    if len(blob) != MAILBOX_BYTES:
        raise ValueError(f"mailbox size {len(blob)} != {MAILBOX_BYTES}")
    words = struct.unpack(f"<{len(blob) // 4}I", blob)
    keys = ("magic", "version", "format", "phase", "command", "core_hz", "cpacr",
            "cycle_read_overhead", "case_count", "active_case", "fault_cfsr", "fault_hfsr")
    result = dict(zip(keys, words[:HEADER_WORDS]))
    if result["magic"] != 0x51424E43 or result["version"] != 1:
        raise ValueError("unknown mailbox ABI")
    if result["format"] not in range(4) or result["case_count"] > MAX_CASES:
        raise ValueError("invalid mailbox dimensions")
    if result["core_hz"] == 0:
        raise ValueError("mailbox clock frequency is zero")
    result["format"] = FORMATS[result["format"]]
    result["cases"] = []
    for i in range(result["case_count"]):
        values = words[HEADER_WORDS + i * CASE_WORDS:HEADER_WORDS + (i + 1) * CASE_WORDS]
        family, nonlinear, n, particles, status = values[:5]
        if family not in range(3):
            raise ValueError("unknown benchmark family")
        checksum = struct.unpack("<d", struct.pack("<II", values[18], values[19]))[0]
        entry = {"family": FAMILIES[family], "format": result["format"],
                 "nonlinear": nonlinear, "n": n, "m": n, "N": particles,
                 "status": status if status < 2**31 else status - 2**32,
                 "cycles": list(values[5:14]),
                 "persistent_bytes": values[14], "descriptor_bytes": values[15],
                 "workspace_bytes": values[16], "output_bytes": values[17],
                 "checksum": checksum, "output_hash": f"{values[20]:08x}"}
        entry["total_bytes"] = sum(values[14:18])
        entry["median_cycles"] = statistics.median(entry["cycles"])
        entry["min_cycles"], entry["max_cycles"] = min(entry["cycles"]), max(entry["cycles"])
        entry["median_us"] = entry["median_cycles"] * 1e6 / result["core_hz"]
        result["cases"].append(entry)
    return result


def validate(result):
    failures = []
    if result["phase"] != 3 or result["case_count"] != MAX_CASES:
        failures.append("run did not complete all 20 cases")
    expected_cpacr = 0xF00000 if result["format"] in ("f32", "f64") else 0
    if result["cpacr"] & 0xF00000 != expected_cpacr:
        failures.append("CPACR does not match requested floating-point mode")
    if result["fault_cfsr"] or result["fault_hfsr"]:
        failures.append("processor fault recorded")
    if result["core_hz"] != 240_000_000:
        failures.append("clock readback differs from the fixed 240 MHz configuration")
    expected = {(family, nonlinear, n, count) for nonlinear in (0, 1) for n in (2, 4)
                for family in FAMILIES for count in ((1,) if family == FAMILIES[0] else (32, 128))}
    actual = {(case["family"], case["nonlinear"], case["n"], case["N"]) for case in result["cases"]}
    if actual != expected:
        failures.append("case keys differ from the complete 20-case matrix")
    for case in result["cases"]:
        if case["status"] or not math.isfinite(case["checksum"]) or not all(case["cycles"]):
            failures.append(f"invalid case {case['family']} n={case['n']} N={case['N']}")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--jlink", type=Path, required=True)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--flash", action="store_true", help="explicitly authorize image programming")
    parser.add_argument("--start", action="store_true", help="start the ready firmware without reflashing")
    parser.add_argument("--prepare-only", action="store_true", help="leave verified firmware waiting for an external capture/start")
    parser.add_argument("--quiet-seconds", type=float, default=24,
                        help="leave SWD idle after start before checking completion")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    elf = args.elf.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix="filters-xhive-board-run-"))
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("run output directory must be empty; preserve previous run evidence")
    address, size = symbol(elf, "g_bench_results", args.nm)
    if size != MAILBOX_BYTES:
        raise RuntimeError(f"ELF mailbox size {size} differs from host ABI {MAILBOX_BYTES}")
    probe = Probe(args.jlink, output)
    if args.flash:
        # An immediate reconnect on this rig observed a zero mailbox before
        # main initialized it. Let startup run on the same connection; the
        # first read below must still validate readiness without polling.
        text = probe.execute([f"loadfile {elf}", "r", "g", "sleep 250"], "flash")
        if not ("Program & Verify speed" in text or "Contents already match" in text or text.count("O.K.") >= 3):
            raise RuntimeError("J-Link did not report a verified flash")
    header = probe.read(address, HEADER_WORDS)
    if header[:2] != [0x51424E43, 1]:
        raise RuntimeError(f"firmware mailbox not initialized: {header}")
    if header[3] == 2:
        raise RuntimeError("measurement rejected: initial SWD read reached a running transaction; reset and repeat")
    if args.prepare_only:
        if header[3] not in (1, 3):
            raise RuntimeError(f"firmware cannot be prepared in phase {header[3]}")
        print(f"READY mailbox=0x{address:08x} command=0x{address + 16:08x} phase={header[3]}")
        print("ARTIFACT_DIR=" + str(output))
        return
    if args.flash or args.start:
        if header[3] not in (1, 3):
            raise RuntimeError(f"firmware not ready (phase {header[3]})")
        probe.start(address, args.quiet_seconds)
        # A still-running image is rejected instead of continuing to poll it.
        header = probe.read(address, HEADER_WORDS)
    if header[3] not in (3, 4):
        raise RuntimeError(f"measurement rejected: SWD observed phase={header[3]} active_case={header[9]} "
                           "before completion; reset and repeat with a longer quiet/capture window")
    raw = output / "mailbox.bin"
    text = probe.execute([f"savebin {raw},0x{address:08X},0x{size:X}",
                          "mem32 0xE000ED88 1", "mem32 0xE000ED28 2",
                          "mem32 0x40021000 8"], "results")
    result = decode(raw.read_bytes())
    observed = parse_words(text)
    result["registers"] = {f"0x{key:08x}": value for key, value in observed.items()}
    if observed[0xE000ED88] != result["cpacr"]:
        raise RuntimeError("live CPACR differs from the firmware mailbox")
    result["elf_sha256"] = hashlib.sha256(elf.read_bytes()).hexdigest()
    result["mailbox_address"] = f"0x{address:08x}"
    result["failures"] = validate(result)
    (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    fields = ["family", "format", "nonlinear", "n", "m", "N", "status", "persistent_bytes",
              "descriptor_bytes", "workspace_bytes", "output_bytes", "total_bytes", "median_cycles",
              "min_cycles", "max_cycles", "median_us", "checksum", "output_hash"]
    with (output / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(result["cases"])
    print((output / "results.csv").read_text(), end="")
    print("ARTIFACT_DIR=" + str(output))
    if result["failures"]:
        raise RuntimeError("; ".join(result["failures"]))


if __name__ == "__main__":
    main()
