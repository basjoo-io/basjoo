toolchain("linux-gcc-x64")
    set_kind("standalone")
    set_homepage("https://gcc.gnu.org/")
    set_description("Linux GCC x86_64-pc-linux-gnu")

    on_check(function (toolchain)
        return is_host("linux") and is_arch("x86_64")
    end)

    on_load(function (toolchain)
        import("lib.detect.find_tool")

        toolchain:set("toolset", "cc", "gcc")
        toolchain:set("toolset", "cxx", "g++")
        toolchain:set("toolset", "fc", "gfortran")
        toolchain:set("toolset", "ld", "g++")
        toolchain:set("toolset", "sh", "g++")
        toolchain:set("toolset", "ar", "gcc-ar")
        toolchain:set("toolset", "ranlib", "gcc-ranlib")
        toolchain:set("toolset", "as", "gcc")

        local ccache = find_tool("ccache")
        if ccache then
            toolchain:set("toolset", "ccache", ccache.program)
        end

        local bfd = find_tool("ld.bfd")
        if bfd then
            toolchain:add("ldflags", "-fuse-ld=bfd")
            toolchain:add("shflags", "-fuse-ld=bfd")
        end

        local base_cxflags = {
            "-march=x86-64-v3", "-mtune=generic", "-pipe", "-fPIC", "-fexceptions",
            "-fstack-clash-protection", "-fcf-protection",
            "-fno-omit-frame-pointer", "-mno-omit-leaf-frame-pointer", "-fno-plt",
            "-Wpedantic", "-Wno-maybe-uninitialized", "-Wno-psabi",
        }
        local base_cxxflags = {"-Wp,-D_GLIBCXX_ASSERTIONS"}
        local base_fcflags = {"-march=x86-64-v3", "-mtune=generic", "-fPIC", "-fno-omit-frame-pointer", "-frecursive"}
        local base_ldflags = {"-Wno-psabi"}

        toolchain:add("cxflags", table.unpack(base_cxflags))
        toolchain:add("cxxflags", table.unpack(base_cxxflags))
        toolchain:add("fcflags", table.unpack(base_fcflags))
        toolchain:add("ldflags", table.unpack(base_ldflags))
        toolchain:add("shflags", table.unpack(base_ldflags))

        if is_mode("debug") then
            toolchain:add("cxflags", "-O0", "-g", "-fsanitize=address,undefined")
            toolchain:add("fcflags", "-O0", "-g", "-fsanitize=address,undefined")
            toolchain:add("ldflags", "-fsanitize=address,undefined")
            toolchain:add("shflags", "-fsanitize=address,undefined")
        elseif is_mode("release") then
            toolchain:add("cxflags", "-O2", "-DNDEBUG", "-Wp,-D_FORTIFY_SOURCE=3", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("fcflags", "-O2", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("ldflags", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("shflags", "-flto=auto", "-fno-fat-lto-objects")
        elseif is_mode("releasedbg") then
            toolchain:add("cxflags", "-O2", "-g", "-DNDEBUG", "-Wp,-D_FORTIFY_SOURCE=3", "-fsanitize=address,undefined", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("fcflags", "-O2", "-g", "-DNDEBUG", "-fsanitize=address,undefined", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("ldflags", "-fsanitize=address,undefined", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("shflags", "-fsanitize=address,undefined", "-flto=auto", "-fno-fat-lto-objects")
        elseif is_mode("minsizerel") then
            toolchain:add("cxflags", "-Os", "-DNDEBUG", "-Wp,-D_FORTIFY_SOURCE=3", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("fcflags", "-Os", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("ldflags", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("shflags", "-flto=auto", "-fno-fat-lto-objects")
        end
    end)
toolchain_end()
