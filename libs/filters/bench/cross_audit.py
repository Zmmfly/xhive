#!/usr/bin/env python3
"""Build ARM objects and record call relocations; this does not execute MCU code.

  python3 libs/filters/bench/cross_audit.py --revision 470206b

Compare like-for-like objects and call edges, not the size of a tiny Q wrapper
against the size of the floating kernel it calls. Unresolved ARM soft-float
helpers are static evidence of arithmetic requirements, not measured cycles.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--revision", help="Git revision; default: working tree")
    parser.add_argument("--cc", default="arm-none-eabi-gcc")
    parser.add_argument("--objdump", default="arm-none-eabi-objdump")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--size", default="arm-none-eabi-size")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    build = Path(tempfile.mkdtemp(prefix="filters-xhive-arm-audit-"))
    source = repo
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", args.revision or "HEAD"], text=True).strip()
    if args.revision:
        source = build / "source"
        source.mkdir()
        archive = subprocess.check_output(["git", "-C", str(repo), "archive", revision, "libs/filters", "libs/linears"])
        subprocess.run(["tar", "-xf", "-", "-C", str(source)], input=archive, check=True)
    else:
        source = build / "source"
        for lib in ("filters", "linears"):
            for folder in ("include", "src"):
                shutil.copytree(repo / "libs" / lib / folder, source / "libs" / lib / folder)
    metadata = {"revision": revision, "source_mode": "git-archive" if args.revision else "working-tree-copy",
                "compiler": subprocess.check_output([args.cc, "--version"], text=True),
                "commands": [], "q_direct_calls": {}, "entry_stack_frames": {},
                "source_sha256": {str(item.relative_to(source)): hashlib.sha256(item.read_bytes()).hexdigest()
                                  for lib in ("filters", "linears")
                                  for folder in ("include", "src")
                                  for item in sorted((source / "libs" / lib / folder).iterdir()) if item.is_file()},
                "measurement": "compile only; no hardware execution or cycle estimate",
                "stack_limit": "Single-function compiler frame only, not a transitive call-chain peak or hardware measurement"}
    for name, abi in (("m4-soft", ["-mfloat-abi=soft"]),
                      ("m4-hard", ["-mfloat-abi=hard", "-mfpu=fpv4-sp-d16"])):
        target = build / name
        target.mkdir()
        for filename in ("ukf.c", "pf.c"):
            obj = target / (filename[:-2] + ".o")
            command = [args.cc, "-std=c99", "-mcpu=cortex-m4", "-mthumb", *abi,
                       "-O3", "-DNDEBUG", "-fno-fast-math", "-ffunction-sections", "-fdata-sections", "-fstack-usage",
                       f"-I{source / 'libs/filters/include'}", f"-I{source / 'libs/linears/include'}",
                       "-c", str(source / "libs/filters/src" / filename), "-o", str(obj)]
            metadata["commands"].append(command)
            subprocess.run(command, check=True)
            for suffix, tool in (("disasm.txt", [args.objdump, "-dr", str(obj)]),
                                 ("symbols.txt", [args.nm, "-S", "--size-sort", str(obj)]),
                                 ("undefined.txt", [args.nm, "-u", str(obj)]),
                                 ("size.txt", [args.size, "-A", str(obj)])):
                (target / (filename[:-2] + "." + suffix)).write_text(subprocess.check_output(tool, text=True))
            calls = {}
            function = ""
            for line in (target / (filename[:-2] + ".disasm.txt")).read_text().splitlines():
                match = re.match(r"[0-9a-f]+ <([^>]+)>:", line)
                if match:
                    function = match.group(1)
                if "_q15_" in function or "_q31_" in function:
                    match = re.search(r"R_ARM_THM_(?:CALL|JUMP24)\s+(\S+)", line)
                    if match:
                        calls.setdefault(function, []).append(match.group(1))
            metadata["q_direct_calls"][name + "/" + filename] = {k: sorted(set(v)) for k, v in calls.items()}
            frames = {}
            for line in obj.with_suffix(".su").read_text().splitlines():
                fields = line.split("\t")
                symbol = fields[0].split(":")[-1]
                if re.match(r"filters_(ukf|pf|upf)_(f32|f64|q15|q31)_(predict|update|estimate|step)(?:_work)?$", symbol):
                    frames[symbol] = {"bytes": int(fields[1]), "classification": fields[2]}
            metadata["entry_stack_frames"][name + "/" + filename] = frames
    metadata["commands"] = json.loads(json.dumps(metadata["commands"]).replace(str(source), "${SOURCE}")
                                       .replace(str(build), "${BUILD}"))
    (build / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Compile-only artifacts retained: {build}")


if __name__ == "__main__":
    main()
