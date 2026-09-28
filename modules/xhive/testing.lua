-- Remove only an owned test directory outside the SDK. Python's rmtree
-- unlinks nested symlinks instead of traversing their targets; never use
-- Xmake's recursive os.rm for these workspaces.
function remove_workdir(workdir)
    import("lib.detect.find_program")
    assert(path.is_absolute(workdir) and path.filename(workdir):find("-xhive-", 1, true),
           "Refusing to clean an unrecognized test directory: %s", workdir)
    local python = find_program("python3") or find_program("python")
    assert(python, "Python 3 is required for safe test-directory cleanup")
    local sdkdir = path.directory(path.directory(os.scriptdir()))
    os.vrunv(python, {"-c", [[
import pathlib
import shutil
import sys

root = pathlib.Path(sys.argv[1])
sdk = pathlib.Path(sys.argv[2]).resolve()
resolved = root.resolve()
if (not root.is_absolute() or "-xhive-" not in root.name or root.is_symlink()
        or resolved == sdk or resolved in sdk.parents or sdk in resolved.parents):
    raise RuntimeError("Refusing to remove a linked workspace or SDK-related path")
if root.exists():
    shutil.rmtree(root)
]], workdir, sdkdir}, {timeout = 10000})
end

-- Adapt script regressions to Xmake's native test reporting. Each script runs
-- in a subprocess, with its own bounded runtime and captured diagnostics.
function main(target, opt)
    import("core.project.config")
    import("lib.detect.find_tool")
    import("lib.detect.find_program")

    local passed = false
    try {
        function ()
            local script = target:values("xhive.test.script")
            local program = os.programfile()
            local argv = {"lua", script}
            if target:values("xhive.test.python") then
                local python = find_tool("python3") or find_tool("python")
                assert(python, "Python 3 is required for the C2 software tests")
                program = python.program
                argv = {script}
            end
            table.join2(argv, table.wrap(target:values("xhive.test.args")))
            if target:values("xhive.test.riscv_gcc") then
                local name = config.get("xhive_riscv_gcc") or "riscv-none-elf-gcc"
                -- find_tool canonicalizes GCC tool names and can fall back to
                -- the host gcc. Honor the exact configured executable instead.
                local compiler = find_program(name)
                assert(compiler, "RISC-V GCC not found: %s; configure --xhive_riscv_gcc=/path/to/gcc", name)
                table.insert(argv, compiler)
            end
            if target:values("xhive.test.clang") then
                table.insert(argv, "--clang")
                table.insert(argv, config.get("xhive_clang") or "clang")
            end
            opt.stdout, opt.stderr = os.iorunv(program, argv, {
                curdir = os.projectdir(), timeout = opt.run_timeout,
                envs = {XMAKE_STATS = "false"}
            })
            passed = true
        end,
        catch {function (errors)
            opt.errors = tostring(errors)
        end}
    }
    return passed
end
