toolchain("darwin-clang-arm64")
    set_kind("standalone")
    set_homepage("https://llvm.org/")
    set_description("Darwin Clang aarch64-apple-darwin")

    on_check(function (toolchain)
        return is_host("macosx") and is_arch("arm64")
    end)

    on_load(function (toolchain)
        import("lib.detect.find_tool")

        toolchain:set("toolset", "cc", "clang")
        toolchain:set("toolset", "cxx", "clang++")
        toolchain:set("toolset", "fc", "flang")
        toolchain:set("toolset", "ld", "clang++")
        toolchain:set("toolset", "sh", "clang++")
        toolchain:set("toolset", "ar", "llvm-ar")
        toolchain:set("toolset", "ranlib", "llvm-ranlib")
        toolchain:set("toolset", "as", "clang")

        local ccache = find_tool("ccache")
        if ccache then
            toolchain:set("toolset", "ccache", ccache.program)
        end

        local base_cxflags = {
            "-march=armv8-a", "-mtune=generic", "-pipe", "-fPIC", "-fexceptions",
            "-fno-omit-frame-pointer", "-mno-omit-leaf-frame-pointer", "-fno-plt",
            "-Wpedantic", "-Wno-psabi",
        }
        local base_cxxflags = {"-stdlib=libc++"}
        local base_fcflags = {"-march=armv8-a", "-mtune=generic", "-fPIC", "-fno-omit-frame-pointer"}

        local llvm_prefix = try { function ()
            return os.iorun("brew --prefix llvm"):trim()
        end} or "/opt/homebrew/opt/llvm"

        local base_ldflags = {
            "-L" .. llvm_prefix .. "/lib/c++",
            "-L" .. llvm_prefix .. "/lib/unwind",
            "-stdlib=libc++", "-rtlib=compiler-rt",
            "--unwindlib=libunwind", "-lc++abi", "-Wno-psabi",
        }

        toolchain:add("cxflags", table.unpack(base_cxflags))
        toolchain:add("cxxflags", table.unpack(base_cxxflags))
        toolchain:add("fcflags", table.unpack(base_fcflags))
        toolchain:add("ldflags", table.unpack(base_ldflags))
        toolchain:add("shflags", table.unpack(base_ldflags))

        if is_mode("debug") then
            toolchain:add("cxflags", "-O0", "-g", "-fsanitize=address,undefined")
            toolchain:add("fcflags", "-O0", "-g")
        elseif is_mode("release") then
            toolchain:add("cxflags", "-O2", "-DNDEBUG", "-ftree-vectorize", "-flto=thin")
            toolchain:add("fcflags", "-O2", "-DNDEBUG", "-ftree-vectorize", "-flto=thin")
            toolchain:add("ldflags", "-flto=thin")
            toolchain:add("shflags", "-flto=thin")
        elseif is_mode("releasedbg") then
            toolchain:add("cxflags", "-O2", "-g", "-DNDEBUG", "-fsanitize=address,undefined", "-ftree-vectorize", "-flto=thin")
            toolchain:add("fcflags", "-O2", "-g", "-DNDEBUG", "-ftree-vectorize", "-flto=thin")
            toolchain:add("ldflags", "-flto=thin")
            toolchain:add("shflags", "-flto=thin")
        elseif is_mode("minsizerel") then
            toolchain:add("cxflags", "-Os", "-DNDEBUG", "-ftree-vectorize", "-flto=thin")
            toolchain:add("fcflags", "-Os", "-DNDEBUG", "-ftree-vectorize", "-flto=thin")
            toolchain:add("ldflags", "-flto=thin")
            toolchain:add("shflags", "-flto=thin")
        end
    end)
toolchain_end()
