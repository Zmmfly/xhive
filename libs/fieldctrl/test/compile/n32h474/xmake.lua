-- Compile/link validation only. Do not flash or run this fixture on hardware.
-- From this directory:
-- XHIVE_SDK_PATH=/path/to/xhive XMAKE_CONFIGDIR="$PWD" xmake f -P . -y --xhive_tests=n
-- XHIVE_SDK_PATH=/path/to/xhive XMAKE_CONFIGDIR="$PWD" xmake -P .

add_cflags("-fstack-usage", "-Wvla", "-Werror=vla", "-Walloca", "-Werror=alloca",
           "-Wstack-usage=4096", "-ffunction-sections", "-fdata-sections", {force = true})
includes(os.getenv("XHIVE_SDK_PATH"))

namespace("app", function()
    add_rules("xhive.embed")
    set_targetdir("dist")

    target("fieldctrl_compile")
        set_kind("binary")
        set_languages("c99")
        add_files("src/*.c")
        add_deps("xhive_embed::nation_n32h47x_48x", "xhive_embed::fieldctrl")
        add_ldflags("-Wl,--gc-sections", "-Wl,-Map=dist/fieldctrl_compile.map", {force = true})
        -- Keep every public algorithm variant even when the representative
        -- calls do not reach it, and fail the link if a named API is absent.
        for _, variant in ipairs({"f32", "f64", "q15", "q31"}) do
            for _, operation in ipairs({"add", "sub", "hadamard", "mul", "matvec",
                "transpose", "axpby", "scale", "copy", "zero", "identity",
                "dot", "norm", "trace", "lu", "solve", "inverse", "determinant",
                "cholesky", "qr", "ldlt", "svd"}) do
                add_ldflags("-Wl,--require-defined=linears_" .. operation .. "_" .. variant,
                            {force = true})
            end
            for _, filter in ipairs({"fir", "biquad", "lowpass", "highpass",
                "moving_average", "ema", "median", "complementary"}) do
                for _, operation in ipairs({"init", "reset", "step"}) do
                    add_ldflags("-Wl,--require-defined=filters_" .. filter .. "_" ..
                                variant .. "_" .. operation, {force = true})
                end
            end
            for _, filter in ipairs({"kalman_scalar", "kalman_matrix"}) do
                for _, operation in ipairs({"reset", "predict", "update"}) do
                    add_ldflags("-Wl,--require-defined=filters_" .. filter .. "_" ..
                                variant .. "_" .. operation, {force = true})
                end
            end
            for _, operation in ipairs({"wrap", "sincos", "atan2", "clarke",
                "inverse_clarke", "park", "inverse_park", "limit_voltage", "svpwm",
                "pid_init", "pid_reset", "pid_retune", "pid_step", "vvvf_init", "vvvf_step", "foc_init", "foc_step",
                "speed_foc_init", "speed_foc_step", "smo_init", "smo_step", "dtc_init",
                "dtc_step"}) do
                add_ldflags("-Wl,--require-defined=fieldctrl_" .. variant .. "_" .. operation,
                            {force = true})
            end
        end
    target_end()
end)
