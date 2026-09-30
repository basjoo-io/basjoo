toolchain("windows-msvc-x64")
    set_kind("standalone")
    set_homepage("https://visualstudio.microsoft.com/")
    set_description("Windows MSVC x86_64")

    on_check(function (toolchain)
        return is_host("windows") and is_arch("x64")
    end)

    on_load(function (toolchain)
        import("lib.detect.find_tool")

        toolchain:set("toolset", "cc", "cl.exe")
        toolchain:set("toolset", "cxx", "cl.exe")
        toolchain:set("toolset", "ld", "link.exe")
        toolchain:set("toolset", "sh", "link.exe")
        toolchain:set("toolset", "ar", "lib.exe")

        local ccache = find_tool("ccache")
        if ccache then
            toolchain:set("toolset", "ccache", ccache.program)
        end

        -- No -fPIC equivalent here: PE/COFF binaries are relocated at load
        -- time via the .reloc section's base relocations, not GOT/PLT-relative
        -- addressing, so MSVC has no compile-time position-independent-code
        -- flag to set.
        local base_cxflags = {
            "/arch:AVX2", "/EHsc", "/utf-8", "/GS",
            "/guard:cf-", "/wd4244", "/wd4267", "/wd4834",
            "/permissive-",
        }

        toolchain:add("cxflags", table.unpack(base_cxflags))
        toolchain:add("defines", "_USE_MATH_DEFINES")

        local shared = is_config("kind", "shared")

        if is_mode("debug") then
            toolchain:add("cxflags", "/Od", "/Zi")
            toolchain:set("runtimes", shared and "MDd" or "MTd")
        elseif is_mode("release") then
            toolchain:add("cxflags", "/O2", "/DNDEBUG", "/GL")
            toolchain:add("ldflags", "/LTCG")
            toolchain:add("shflags", "/LTCG")
            toolchain:set("runtimes", shared and "MD" or "MT")
        elseif is_mode("releasedbg") then
            toolchain:add("cxflags", "/O2", "/DNDEBUG", "/Zi", "/GL")
            toolchain:add("ldflags", "/LTCG")
            toolchain:add("shflags", "/LTCG")
            toolchain:set("runtimes", shared and "MD" or "MT")
        elseif is_mode("minsizerel") then
            toolchain:add("cxflags", "/O1", "/DNDEBUG", "/GL")
            toolchain:add("ldflags", "/LTCG")
            toolchain:add("shflags", "/LTCG")
            toolchain:set("runtimes", shared and "MD" or "MT")
        end
    end)
toolchain_end()
