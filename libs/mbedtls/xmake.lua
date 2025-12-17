--[[
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
--]]

add_moduledirs("modules")

target("mbedtls")
    set_kind("object")
    set_default(false)
    local dir         = path.join(os.scriptdir(), "mbedtls-4.0")
    local tls_dir     = path.join(dir, "library")
    local psa_dir     = path.join(dir, "tf-psa-crypto")
    local psa_drv_dir = path.join(psa_dir, "drivers")

    -- add base include dir
    add_includedirs(path.join(dir, "include"), {public = true})

    -- add tf-psa-crypto/core
    local psa_core_dir = path.join(psa_dir, "core")
    add_files(path.join(psa_core_dir, "*.c"))
    add_includedirs(psa_core_dir)

    -- add public tf-psa-crypto/include
    add_includedirs(path.join(psa_dir, "include"), {public = true})

    -- add tf-psa-crypto driver builtin
    local psa_builtin_dir = path.join(psa_drv_dir, "builtin")
    add_files(path.join(psa_builtin_dir, "src", "*.c"))
    add_includedirs(path.join(psa_builtin_dir, "src"))
    add_includedirs(path.join(psa_builtin_dir, "include"))
    on_load(function(target)
        import("xhive.base")
        local dirs = base.load_dirs()
        local conf = target:data("kconfig")
        if conf.MBEDTLS_USE_TF_PSA_CRYPTO_USER_CONFIG_FILE then
            local psa_user_config = conf.MBEDTLS_TF_PSA_CRYPTO_USER_CONFIG_FILE
            local conf_path = path.join(dirs.prjdir, psa_user_config)
            if os.isfile(conf_path) then
                target:add("defines", vformat([[MBEDTLS_TF_PSA_CRYPTO_USER_CONFIG_FILE="%s"]], conf_path), {public = true})
            end
        end

        if conf.MBEDTLS_USE_USER_CONFIG_FILE then
            local mbedtls_user_config = conf.MBEDTLS_USER_CONFIG_FILE
            local conf_path = path.join(dirs.prjdir, mbedtls_user_config)
            if os.isfile(conf_path) then
                target:add("defines", vformat([[MBEDTLS_USER_CONFIG_FILE="%s"]], conf_path), {public = true})
            end
        end
    end)
target_end()
