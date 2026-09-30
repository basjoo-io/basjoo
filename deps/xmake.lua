add_requires("cmake", {system = true})
add_requires("ninja", {system = true})
add_requires("openmp", {system = true})

if is_plat("linux") then
    add_syslinks("pthread")
end

includes("python/xmake.lua")
includes("nanobind/xmake.lua")

includes("cxxopts/xmake.lua")
includes("spdlog/xmake.lua")
includes("doctest/xmake.lua")
includes("msft_proxy4/xmake.lua")
includes("pocketfft/xmake.lua")
includes("zpp_bits/xmake.lua")

includes("qdldl/xmake.lua")
includes("osqp/xmake.lua")

includes("boost/xmake.lua")
includes("taskflow/xmake.lua")
includes("stdexec/xmake.lua")

target("blas_lapack")
    set_kind("phony")
target_end()
if is_config("basjoo_use_blas_lapack", "openblas") then
    includes("openblas/xmake.lua")
end
includes("matplotplusplus/xmake.lua")
