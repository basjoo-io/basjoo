target("nanobind")
    set_kind("static")
    set_default(false)
    on_load(function (target)
        local python = is_plat("windows")
            and path.join(path.directory(path.directory(os.scriptdir())), ".venv", "Scripts", "python.exe")
            or path.join(path.directory(path.directory(os.scriptdir())), ".venv", "bin", "python")
        local py_include = os.iorunv(python, {"-c", "import sysconfig; print(sysconfig.get_path('include'))"}):trim()
        local nb_include = os.iorunv(python, {"-m", "nanobind", "--include_dir"}):trim()
        local nb_root = path.directory(nb_include)

        target:add("files",
            path.join(nb_root, "src", "common.cpp"),
            path.join(nb_root, "src", "error.cpp"),
            path.join(nb_root, "src", "implicit.cpp"),
            path.join(nb_root, "src", "nb_enum.cpp"),
            path.join(nb_root, "src", "nb_func.cpp"),
            path.join(nb_root, "src", "nb_internals.cpp"),
            path.join(nb_root, "src", "nb_ndarray.cpp"),
            path.join(nb_root, "src", "nb_static_property.cpp"),
            path.join(nb_root, "src", "nb_type.cpp"),
            path.join(nb_root, "src", "trampoline.cpp")
        )
        target:add("includedirs", nb_include, path.join(nb_root, "ext", "robin_map", "include"), py_include, {public = true})
        target:add("cxflags", "-fno-strict-aliasing", "-fvisibility=hidden")
        if not is_plat("windows") then
            target:add("cxflags", "-ffunction-sections", "-fdata-sections")
        end
    end)
    on_config(function (target)
        if not target:has_tool("cxx", "cl") then
            target:add("cxflags", "-fno-sanitize=address,undefined", {force = true})
        end
    end)
target_end()
