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

-- Capture an existing script's result for Xmake's native test reporting.
-- Compiler selection and component-specific arguments belong to the caller.
function run(program, argv, opt)
    local passed = false
    try {
        function ()
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

-- Only SDK build-system regressions use this Lua-script adapter.
function main(target, opt)
    local argv = {"lua", target:values("xhive.test.script")}
    table.join2(argv, table.wrap(target:values("xhive.test.args")))
    return run(os.programfile(), argv, opt)
end
