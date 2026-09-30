function install(package)
    import("package.tools.cmake")

    local configs = {
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        "-DCMAKE_COMPILE_WARNING_AS_ERROR=OFF",
        "-DBUILD_STATIC_LIBS=" .. (package:config("shared") and "OFF" or "ON"),
        "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"),
        "-DBUILD_TESTING=OFF",
        "-DNOFORTRAN=ON",
        "-DC_LAPACK=ON",
        "-DTARGET=" .. (package:is_arch("x64") and "HASWELL" or "ARMV8"),
        "-DDYNAMIC_ARCH=OFF",
        "-DUSE_THREAD=OFF",
        "-DUSE_OPENMP=OFF",
    }
    cmake.install(package, configs)
end
