add_requires("boost 1.90.0", {configs = {
    cmake = true,
    asio = true,
    multiprecision = true,
    system = true,
    filesystem = false,
    shared = is_config("kind", "shared")
}})
