add_requires("boost 1.92.0", {configs = {
    cmake = true,
    asio = true,
    multiprecision = true,
    system = true,
    filesystem = false,
    shared = is_config("kind", "shared")
}})
