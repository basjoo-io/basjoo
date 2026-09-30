function install(package)
    import("package.tools.make")

    local cc = package:tool("cc") or ""
    local fc = package:tool("fc") or ""
    local is_clang_cc = cc:find("clang", 1, true) ~= nil
    local is_flang_fc = fc:find("flang", 1, true) ~= nil

    local function get_env_flags(name)
        local val = package:build_getenv(name)
        if type(val) == "table" then
            return table.concat(val, " ")
        end
        return val or ""
    end

    local function join_flags(base, extra)
        if extra == "" then return base end
        if base == "" then return extra end
        return base .. " " .. extra
    end

    local cflags_base = get_env_flags("cflags")
    local fflags_base = get_env_flags("fflags")
    local ldflags_base = is_clang_cc and "-rtlib=compiler-rt" or ""
    local ipo_cflags, ipo_ldflags
    local enable_lto = not is_mode("debug")

    if not is_clang_cc then
        -- GCC + GFortran
        ipo_cflags  = enable_lto and "-flto=auto -fno-fat-lto-objects" or "-fno-lto"
        ipo_ldflags = enable_lto and "-flto=auto -flto-partition=none" or "-fno-lto"
    elseif is_flang_fc then
        -- Clang + LLVMFlang
        ipo_cflags  = "-fno-lto"
        ipo_ldflags = "-fno-lto"
    else
        -- Clang + GFortran
        ipo_cflags  = "-fno-sanitize=address,undefined -fno-lto"
        ipo_ldflags = "-fno-lto"
    end

    local cflags  = join_flags(cflags_base, ipo_cflags)
    local fflags  = join_flags(fflags_base, ipo_cflags)
    local ldflags = join_flags(ldflags_base, ipo_ldflags)

    local configs = {
        "shared",
        "AR="       .. (package:tool("ar")     or "ar"),
        "RANLIB="   .. (package:tool("ranlib") or "ranlib"),
        "CC="       .. cc,
        "FC="       .. fc,
        "ASMFLAGS=" .. cflags,
        "CFLAGS="   .. cflags,
        "FFLAGS="   .. fflags,
        "LDFLAGS="  .. ldflags,
        "TARGET="   .. (package:is_arch("x64") and "HASWELL" or "ARMV8"),
        "DYNAMIC_ARCH=0",
        "NO_AFFINITY=0",
        "USE_THREAD=0",
        "USE_LOCKING=0",
        "USE_OPENMP=0",
        package:config("shared") and "NO_STATIC=1" or "NO_SHARED=1",
    }
    make.build(package, configs)
    make.make(package, table.join("install", "PREFIX=" .. package:installdir(), configs))
end
