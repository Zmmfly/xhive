#!/usr/bin/env python3
"""Capture the analyzer-connected GPIO timing markers, then read the mailbox.

The matching ELF must already be flashed and waiting (run.py --prepare-only).
No UART, PWM or motor-control peripheral is started. Uses the installed PXView
Python client and the same J-Link command-file protocol as run.py.
"""
import argparse
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import struct
import subprocess
import sys
import tempfile

PINS = ("PA8", "PA9", "PA10", "PA11", "PB12", "PB13", "PB14", "PB15",
        "PC8", "PC9", "PC6", "PC7")


def pulse_edges(data):
    """Yield high pulse ranges from a packed lsb0 bitmap, without expansion."""
    previous, rise = 0, None
    for i, byte in enumerate(data):
        transitions = byte ^ (((byte << 1) & 255) | previous)
        while transitions:
            low = transitions & -transitions
            bit = low.bit_length() - 1
            position = i * 8 + bit
            if (byte >> bit) & 1:
                rise = position
            elif rise is not None:
                yield rise, position
                rise = None
            transitions ^= low
        previous = byte >> 7


def read_packed(client, channel, sample_count, destination):
    """Read every snapshot block; PXView binary export may return only block 0."""
    position, pages = 0, 0
    with destination.open("wb") as stream:
        while position < sample_count:
            end = min(sample_count, position + 8 * 1024 * 1024)
            page = client.get_samples_meta(channel, "logic", position, end - 1)
            data = base64.b64decode(page["data"], validate=True)
            count = page["sample_count"]
            if (page["first_sample"] != position or not 0 < count <= end-position
                    or page["bit_order"] != "lsb0" or page["bits_per_sample"] != 1
                    or len(data) != (count+7)//8):
                raise RuntimeError(f"invalid packed sample page: channel={channel} position={position}")
            if count % 8 and position+count != sample_count:
                raise RuntimeError("non-final snapshot page is not byte-aligned")
            stream.write(data)
            position += count
            pages += 1
    return {"samples": position, "pages": pages, "bytes": destination.stat().st_size}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--jlink", type=Path, required=True)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--python-client", type=Path, default=Path("/usr/share/pxview/python"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--sample-rate", type=int, default=5_000_000)
    parser.add_argument("--seconds", type=float, default=28)
    args = parser.parse_args()
    directory = args.output or Path(tempfile.mkdtemp(prefix="filters-xhive-board-capture-"))
    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        raise ValueError("capture output directory must be empty; preserve previous run evidence")
    spec = importlib.util.spec_from_file_location("board_run", Path(__file__).with_name("run.py"))
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    address, _ = runner.symbol(args.elf, "g_bench_results", args.nm)
    probe = runner.Probe(args.jlink, directory)
    header = probe.read(address, runner.HEADER_WORDS)
    if header[:2] != [0x51424E43, 1] or header[3] not in (1, 3):
        raise RuntimeError(f"board not ready: {header}")
    format_index = header[2]
    sys.path.insert(0, str(args.python_client))
    from pxview_automation import McpClient
    client = McpClient(max_retries=1, timeout=60)
    client.connect()
    devices = client.get_devices()
    device = next(item for item in devices if item.get("is_active") and item.get("is_hardware_logic"))
    if 0 not in client.get_supported_work_modes():
        raise RuntimeError("active analyzer has no logic mode")
    client.switch_work_mode(0)
    for channel in client.get_channels():
        index = channel["index"]
        client.configure_channel(index, enabled=index < 12,
                                 name=(f"{index}:{PINS[index]}" if index < 12 else str(index)))
    original_trigger = client.configure_trigger()
    trigger = json.loads(original_trigger["config_json"])
    trigger["enabled"] = 0
    client.configure_trigger(config_json=json.dumps(trigger), stage_count=original_trigger["stage_count"])
    client.configure_glitch_filter(channels=[], thresholds=[])
    client.configure_signal_invert(channels=[], invert_states=[])
    client.set_sample_config(sample_rate=args.sample_rate,
                             sample_limit=int(args.sample_rate * args.seconds), collect_mode=0)
    client.start_capture(device_id=device["id"], captureMode="single", channelMode="Buffer",
                         durationSeconds=args.seconds, digitalChannels=list(range(12)),
                         digitalSampleRate=args.sample_rate)
    probe.start(address, min(24, max(1, args.seconds - 4)))
    # Waiting for a real acquisition avoids SWD reads during timed algorithms.
    capture_status = client.wait_capture(timeout_seconds=args.seconds + 10, timeout=args.seconds + 20)
    status = client.get_capture_status()
    if not status.get("have_hardware_data"):
        raise RuntimeError(f"capture did not provide hardware samples: {status}")
    config = client.get_sample_config()
    raw = directory / "raw"
    raw.mkdir(exist_ok=True)
    channels = [4 * family + format_index for family in range(3)]
    export = {str(channel): read_packed(client, channel, config["sample_limit"], raw / f"channel_{channel}.bin")
              for channel in channels}
    subprocess.run([sys.executable, str(Path(__file__).with_name("run.py")), str(args.elf),
                    "--jlink", str(args.jlink), "--nm", args.nm, "--output", str(directory / "board")], check=True)
    result = json.loads((directory / "board/results.json").read_text())
    rate = config["sample_rate"]
    evidence = {"device": device, "sample_config": config, "capture_status": status,
                "wait_result": capture_status, "export_result": export, "channels": {},
                "pin_map": dict(enumerate(PINS)), "sample_encoding": "packed lsb0", "failures": []}
    for family, channel in enumerate(channels):
        files = list(raw.glob(f"channel_{channel}.*"))
        if len(files) != 1:
            raise RuntimeError(f"missing/ambiguous raw channel {channel}: {files}")
        data = files[0].read_bytes()
        pulses = list(pulse_edges(data))
        wrap_seconds = 2**32 / result["core_hz"]
        if any((fall-rise) / rate >= wrap_seconds for rise, fall in pulses):
            evidence["failures"].append(f"channel {channel}: pulse exceeds one DWT counter period")
        cases = [case for case in result["cases"] if case["family"] == runner.FAMILIES[family]]
        expected = len(cases) * (2 + 9)
        info = {"raw_sha256": hashlib.sha256(data).hexdigest(), "raw_bytes": len(data),
                "pulse_count": len(pulses), "expected_pulse_count": expected,
                "pulses": [{"rise": rise, "fall": fall, "width_us": (fall-rise) * 1e6 / rate}
                           for rise, fall in pulses], "cases": []}
        if len(pulses) != expected:
            evidence["failures"].append(f"channel {channel}: {len(pulses)} pulses, expected {expected}")
        else:
            for i, case in enumerate(cases):
                selected = pulses[i * 11 + 2:i * 11 + 11]
                widths = [(fall-rise) * 1e6 / rate for rise, fall in selected]
                median_us = statistics.median(widths)
                # Separate physical clocks: allow 2% HSI frequency tolerance,
                # plus 1 us for marker function/bus overhead and sample edges.
                for width, cycles in zip(widths, case["cycles"]):
                    dwt_us = cycles * 1e6 / result["core_hz"]
                    if abs(width - dwt_us) > 1.0 + 0.02 * dwt_us:
                        evidence["failures"].append(f"channel {channel} case {i}: GPIO/DWT mismatch "
                                                    f"{width:.3f} us vs {dwt_us:.3f} us")
                        break
                info["cases"].append({"nonlinear": case["nonlinear"], "n": case["n"], "N": case["N"],
                                      "logic_median_us": median_us, "dwt_median_us": case["median_us"],
                                      "ratio": median_us / case["median_us"]})
        evidence["channels"][str(channel)] = info
    (directory / "capture.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print("ARTIFACT_DIR=" + str(directory))
    print(json.dumps({key: {"pulse_count": value["pulse_count"], "cases": value["cases"]}
                      for key, value in evidence["channels"].items()}, indent=2))
    if evidence["failures"]:
        raise RuntimeError("; ".join(evidence["failures"]))


if __name__ == "__main__":
    main()
