includes("bicycle/xmake.lua")
includes("common/xmake.lua")
includes("cvxopm/xmake.lua")
includes("math/xmake.lua")

target("_core")
    set_kind("shared")
    add_includedirs("..")
    -- xmake-python (pyproject.xmake.toml) installs through an ephemeral
    -- project root that defines the pythondir config: route the module
    -- binary into the wheel's platlib (basjoo/) instead of the default
    -- C++ lib dir. Regular `xmake install` keeps the default layout.
    if get_config("pythondir") then
        set_prefixdir("$(prefixdir)/$(pythondir)", { libdir = "basjoo" })
    end
    set_prefixname("")
    set_targetdir(os.scriptdir())
    set_version("0.0.1", {soname = false})
    add_files("_core.cpp")
    add_deps("nanobind")
    add_rules("python.module")
    on_load(function (target)
        local pythondir = get_config("pythondir") or "install"
        target:add(
            "installfiles",
            path.join(os.scriptdir(), "__init__.py"),
            path.join(os.scriptdir(), "_core.pyi"),
            path.join(os.scriptdir(), "py.typed"),
            {prefixdir = path.join(pythondir, "basjoo")}
        )
    end)
    on_config(function (target)
        if not target:has_tool("cxx", "cl") then
            target:add("cxflags", "-fno-sanitize=address,undefined", {force = true})
            target:add("shflags", "-fno-sanitize=address,undefined", {force = true})
        end

        -- Python C API symbols (PyDict_SetItem, PyErr_Clear, ...) are resolved
        -- at load time by the embedding python interpreter, not at link time.
        -- Mach-O linking rejects undefined symbols by default, unlike ELF.
        if target:is_plat("macosx") then
            target:add("shflags", "-Wl,-undefined,dynamic_lookup", {force = true})
        end
    end)
    after_build(function (target)
        local python = target:data("python.venv_program")
        local targetdir = target:targetdir()
        os.vrunv(python, {
            "-m", "nanobind.stubgen",
            "-i", targetdir,
            "-m", "_core",
            "-O", targetdir,
            "-M", path.join(targetdir, "py.typed"),
        })
    end)
target_end()
