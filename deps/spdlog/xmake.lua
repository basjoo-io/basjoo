add_requires("spdlog 1.17.0", {configs = {
    std_format = true,
    shared = is_config("kind", "shared"),
}})
