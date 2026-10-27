#!/usr/bin/env python3
"""Build one N32H474RC benchmark image, without Xmake state or hardware access.

Examples (from any directory):
  python3 libs/filters/bench/n32h474/build.py --format f32 --revision 3ee417a --label baseline
  python3 libs/filters/bench/n32h474/build.py --format q15 --label current
  python3 libs/filters/bench/n32h474/build.py --format q31 --platform-only

Production sources use git archive with --revision, otherwise a working-tree
snapshot. Platform/workload/vendor CMSIS always use this checkout's snapshot.
Artifacts are retained only in an exclusive /tmp/*-xhive-* directory. This
script never downloads tools, configures Xmake, flashes or accesses hardware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile


FORMATS = {"f32": 0, "f64": 1, "q15": 2, "q31": 3}
CC = Path("/usr/bin/arm-none-eabi-gcc")
CXX = Path("/usr/bin/arm-none-eabi-g++")
TOOLS = {name: Path(f"/usr/bin/arm-none-eabi-{name}")
         for name in ("objdump", "readelf", "size", "nm", "objcopy")}
VENDOR = Path("vendor/nation/n32h47x_48x/firmware/CMSIS")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", required=True, choices=FORMATS,
                        help="one scalar format per image")
    parser.add_argument("--fpu", choices=("on", "off"),
                        help="default: on for f32/f64, off for q15/q31; Q images require off")
    parser.add_argument("--revision", help="Git revision for production filters/linears sources")
    parser.add_argument("--label", default="current", help="artifact directory label")
    parser.add_argument("--platform-only", action="store_true",
                        help="boot/command smoke image with zero benchmark cases")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.label):
        parser.error("--label must contain only letters, digits, '_', '.' or '-'")
    fpu_enabled = args.fpu != "off" if FORMATS[args.format] < 2 else False
    if FORMATS[args.format] >= 2 and args.fpu == "on":
        parser.error("Q images require --fpu off")

    driver = Path(__file__).resolve().parent
    repo = driver.parents[3]
    build = Path(tempfile.mkdtemp(prefix=f"filters-xhive-n32h474-{args.label}-{args.format}-",
                                  dir="/tmp"))
    source = build / "source"
    source.mkdir()
    commands = []
    metadata = {
        "format": args.format,
        "bench_format": FORMATS[args.format],
        "image_kind": "platform-only" if args.platform_only else "filter-benchmark",
        "source_mode": "git-archive" if args.revision else "working-tree-copy",
        "hardware_access": False,
        "flash_bytes": 256 * 1024,
        "ram_bytes": 96 * 1024,
        "stack_reserve_bytes": 8 * 1024,
        "clock": "HSI 8 MHz x30 PLL; SystemCoreClockUpdate readback in mailbox",
        "fpu": "CP10/CP11 full access" if fpu_enabled else "CPACR=0; software float ABI",
        "fpu_mode": "on" if fpu_enabled else "off",
        "float_abi": "hard" if fpu_enabled else "soft",
        "commands": commands,
        "build_succeeded": False,
    }
    # GCC's own temporary files must not escape this exclusive directory.
    environment = {**os.environ, "TMPDIR": str(build), "LC_ALL": "C"}
    print(f"Artifacts: {build}", flush=True)

    def run(command, *, output=None, binary=False, input_text=None):
        command = list(map(str, command))
        commands.append(command)
        (build / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        text = "+ " + shlex.join(command)
        print(text, flush=True)
        with (build / "build.log").open("a") as log:
            log.write(text + "\n")
            if binary:
                with output.open("wb") as stream:
                    result = subprocess.run(command, cwd=build, env=environment,
                                            stdout=stream, stderr=subprocess.PIPE)
                stderr = result.stderr.decode("utf-8", errors="replace")
                log.write(stderr)
                if stderr:
                    print(stderr, end="", file=sys.stderr)
                result.check_returncode()
                return ""
            result = subprocess.run(command, cwd=build, env=environment, input=input_text,
                                    capture_output=True, text=True)
            log.write(result.stdout)
            log.write(result.stderr)
            if output is not None:
                output.write_text(result.stdout)
            elif result.stdout:
                print(result.stdout, end="")
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)
            result.check_returncode()
            return result.stdout

    try:
        for tool in (CC, CXX, *TOOLS.values()):
            if not tool.is_file():
                raise RuntimeError(f"required installed tool is missing: {tool}")
        metadata["compiler"] = run([CC, "--version"])
        metadata["cxx_compiler"] = run([CXX, "--version"])
        metadata["binutils"] = run([TOOLS["objdump"], "--version"])
        metadata["revision"] = run(["git", "-C", repo, "rev-parse", args.revision or "HEAD"]).strip()
        metadata["working_tree_status"] = run(["git", "-C", repo, "status", "--short"])
        abi = (["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"] if fpu_enabled
               else ["-mfloat-abi=soft"])
        architecture = ["-mcpu=cortex-m4", "-mthumb", *abi]
        metadata["newlib"] = {}
        for library in ("libc.a", "libm.a", "libgcc.a"):
            location = Path(run([CC, *architecture, f"-print-file-name={library}"]).strip())
            if not location.is_absolute() or not location.is_file():
                raise RuntimeError(f"required matching-ABI library is unavailable: {library}: {location}")
            metadata["newlib"][library] = {"path": str(location.resolve()), "sha256": sha256(location)}
        run([CC, *architecture, "-x", "c", "-fsyntax-only", "-"],
            input_text="#include <math.h>\n#include <string.h>\n#include <stdint.h>\n")

        production = [f"libs/{lib}/{folder}" for lib in ("filters", "linears")
                      for folder in ("include", "src")]
        if args.revision:
            archive = build / "production.tar"
            run(["git", "-C", repo, "archive", metadata["revision"], *production],
                output=archive, binary=True)
            run(["tar", "-xf", archive, "-C", source])
        else:
            for folder in production:
                shutil.copytree(repo / folder, source / folder)

        # Snapshot every input before compilation; never build out of mutable
        # source directories and never reuse the SDK or application .xmake cache.
        local = source / driver.relative_to(repo)
        local.mkdir(parents=True)
        files = ["build.py", "platform.c", "startup.S", "link.ld", "xhive_config.h",
                 "bench_platform.h", "bench_result.h"]
        if not args.platform_only:
            files += ["workload.cc", "draws.h"]
        for name in files:
            shutil.copy2(driver / name, local / name)
        shutil.copytree(repo / VENDOR / "core", source / VENDOR / "core")
        (source / VENDOR / "device").mkdir(parents=True)
        for name in ("system_n32h47x_48x.c", "system_n32h47x_48x.h", "n32h47x_48x.h"):
            shutil.copy2(repo / VENDOR / "device" / name, source / VENDOR / "device" / name)
        metadata["source_sha256"] = {
            str(item.relative_to(source)): sha256(item)
            for item in sorted(source.rglob("*")) if item.is_file()
        }
        metadata["system_sha256"] = sha256(source / VENDOR / "device/system_n32h47x_48x.c")
        flags = [*architecture, "-O3", "-g3", "-DNDEBUG", "-fno-fast-math",
                 "-ffp-contract=off", "-fno-lto", "-ffunction-sections", "-fdata-sections",
                 "-fstack-usage", f"-DBENCH_FORMAT={FORMATS[args.format]}",
                 f"-DBENCH_FPU_ENABLED={int(fpu_enabled)}",
                 "-DSYSCLK_FREQ=240000000", "-DSYSCLK_SRC=2"]
        if args.platform_only:
            flags += ["-DBENCH_PLATFORM_ONLY=1"]
        includes = [f"-I{path}" for path in
                    (local, source / "libs/filters/include", source / "libs/linears/include",
                     source / VENDOR / "core", source / VENDOR / "device")]
        metadata["compile_flags"] = flags
        objects_dir = build / "objects"
        objects_dir.mkdir()
        sources = [local / "startup.S", local / "platform.c",
                   source / VENDOR / "device/system_n32h47x_48x.c"]
        if not args.platform_only:
            sources += sorted((source / "libs/linears/src").glob("*.c"))
            sources += [source / "libs/filters/src/ukf.c", source / "libs/filters/src/pf.c",
                        local / "workload.cc"]
        objects = []
        for item in sources:
            name = "_".join(item.relative_to(source).parts).replace(".", "_") + ".o"
            obj = objects_dir / name
            if item.suffix == ".cc":
                compiler = CXX
                extra = ["-std=c++17", "-fno-exceptions", "-fno-rtti", "-fno-use-cxa-atexit"]
            else:
                compiler = CC
                extra = [] if item.suffix == ".S" else ["-std=c11"]
            if item.name == "platform.c":
                extra += ["-mgeneral-regs-only", "-Wall", "-Wextra", "-Werror"]
            run([compiler, *flags, *extra, *includes, "-MMD", "-MP", "-c", item, "-o", obj])
            objects.append(obj)

        elf = build / "benchmark.elf"
        run([CC, *architecture, "-fno-lto", "-nostartfiles", *objects,
             f"-T{local / 'link.ld'}", f"-Wl,-Map={build / 'benchmark.map'}",
             "-Wl,--gc-sections", "-Wl,--print-memory-usage", "-Wl,--build-id=none",
             "-Wl,--start-group", "-lm", "-lc", "-lgcc", "-Wl,--end-group", "-o", elf])
        run([TOOLS["objdump"], "-d", "-S", elf], output=build / "benchmark.dis")
        run([TOOLS["readelf"], "-h", "-A", "-S", "-l", elf], output=build / "benchmark.attributes.txt")
        run([TOOLS["size"], "-A", "-x", elf], output=build / "benchmark.size.txt")
        metadata["size_summary"] = run([TOOLS["size"], elf])
        symbols = run([TOOLS["nm"], "-n", "-S", elf], output=build / "benchmark.symbols.txt")
        run([TOOLS["objcopy"], "-O", "binary", elf, build / "benchmark.bin"])
        addresses = {}
        for line in symbols.splitlines():
            fields = line.split()
            if len(fields) in (3, 4):
                addresses[fields[-1]] = int(fields[0], 16)
            if len(fields) == 4 and fields[3] == "g_bench_results":
                metadata["mailbox_address"] = int(fields[0], 16)
                metadata["mailbox_bytes"] = int(fields[1], 16)
        metadata["linked_memory"] = {
            "flash_used_bytes": addresses["__flash_end"] - 0x08000000,
            "static_ram_end": addresses["__bss_end"],
            "static_ram_bytes": addresses["__bss_end"] - 0x20000000,
            "free_below_reserved_stack_bytes": addresses["__stack_limit"] - addresses["__bss_end"],
            "stack_limit": addresses["__stack_limit"],
            "stack_top": addresses["__stack_top"],
        }
        if metadata.get("mailbox_bytes") != 1728:
            raise RuntimeError("mailbox does not match the frozen 1728-byte ABI")
        # A soft image must contain no hardware floating-point instructions,
        # including those from the actual linked newlib/libgcc multilibs.
        disassembly = run([TOOLS["objdump"], "-d", elf], output=build / "benchmark.instructions.txt")
        vfp = [line for line in disassembly.splitlines()
               if re.match(r"\s*[0-9a-f]+:\s+(?:[0-9a-f]{4}\s+){1,2}v[a-z]", line)]
        metadata["vfp_instruction_count"] = len(vfp)
        if not fpu_enabled and vfp:
            raise RuntimeError("soft-ABI image contains VFP instructions")
        metadata["elf_sha256"] = sha256(elf)
        metadata["build_succeeded"] = True
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        metadata["error"] = str(error)
        print(f"Build failed: {error}", file=sys.stderr)
        return 1
    finally:
        # Exact commands remain usable on this machine; a portable copy follows
        # the existing bench/run.py placeholder convention for comparisons.
        metadata["portable_commands"] = json.loads(json.dumps(commands).replace(str(source), "${SOURCE}")
                                                     .replace(str(repo), "${REPO}")
                                                     .replace(str(build), "${BUILD}"))
        (build / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Artifacts retained: {build}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
