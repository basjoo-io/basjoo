-- Project-local override of xmake's builtin rules/python/module rule (see
-- https://github.com/xmake-io/xmake/issues/1896), reworked to resolve the
-- interpreter and include directory from this repo's own .venv instead of
-- the builtin rule's find_tool("python3")-based PATH probe, which may
-- resolve a different interpreter than .venv (e.g. a system python3).
rule("python.module")
    on_load(function (target)
        local python = is_plat("windows")
            and path.join(path.directory(path.directory(os.scriptdir())), ".venv", "Scripts", "python.exe")
            or path.join(path.directory(path.directory(os.scriptdir())), ".venv", "bin", "python")
        assert(os.isfile(python), "venv python not found at " .. python .. ", run `uv sync --no-install-project` first")
        target:data_set("python.venv_program", python)

        local py_include = os.iorunv(python, {"-c", "import sysconfig; print(sysconfig.get_path('include'))"}):trim()
        target:add("includedirs", py_include)
    end)
    on_config(function (target)
        target:set("kind", "shared")
        target:set("prefixname", "")
        target:add("runenvs", "PYTHONPATH", target:targetdir())

        local python = target:data("python.venv_program")
        local ext_suffix = os.iorunv(python, {"-c", "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"}):trim()
        if ext_suffix ~= "None" then
            target:set("extension", ext_suffix)
        end
    end)
rule_end()
