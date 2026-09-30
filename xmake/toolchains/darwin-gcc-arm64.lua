toolchain("darwin-gcc-arm64")
    set_kind("standalone")
    set_homepage("https://gcc.gnu.org/")
    set_description("Darwin GCC aarch64-apple-darwin")

    on_check(function (toolchain)
        return is_host("macosx") and is_arch("arm64")
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

        local base_cxflags = {
            "-march=armv8-a", "-mtune=generic", "-pipe", "-fPIC", "-fexceptions",
            "-fno-omit-frame-pointer", "-mno-omit-leaf-frame-pointer", "-fno-plt",
            "-Wpedantic", "-Wno-maybe-uninitialized", "-Wno-psabi",
        }
        local base_cxxflags = {"-Wp,-D_GLIBCXX_ASSERTIONS"}
        local base_fcflags = {"-march=armv8-a", "-mtune=generic", "-fPIC", "-fno-omit-frame-pointer", "-frecursive"}
        local base_ldflags = {"-Wno-psabi"}

        toolchain:add("cxflags", table.unpack(base_cxflags))
        toolchain:add("cxxflags", table.unpack(base_cxxflags))
        toolchain:add("fcflags", table.unpack(base_fcflags))
        toolchain:add("ldflags", table.unpack(base_ldflags))
        toolchain:add("shflags", table.unpack(base_ldflags))

        if is_mode("debug") then
            toolchain:add("cxflags", "-O0", "-g")
            toolchain:add("fcflags", "-O0", "-g")
        elseif is_mode("release") then
            toolchain:add("cxflags", "-O2", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("fcflags", "-O2", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("ldflags", "-flto=auto")
            toolchain:add("shflags", "-flto=auto")
        elseif is_mode("releasedbg") then
            toolchain:add("cxflags", "-O2", "-g", "-DNDEBUG", "-ftree-vectorize")
            toolchain:add("fcflags", "-O2", "-g", "-DNDEBUG", "-ftree-vectorize")
        elseif is_mode("minsizerel") then
            toolchain:add("cxflags", "-Os", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("fcflags", "-Os", "-DNDEBUG", "-ftree-vectorize", "-flto=auto", "-fno-fat-lto-objects")
            toolchain:add("ldflags", "-flto=auto")
            toolchain:add("shflags", "-flto=auto")
        end
    end)
toolchain_end()
