load("//bzl:module.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "simple_components",
    srcs = [
        "SimpleApp.cpp",
        "SimpleControl.cpp"
    ],
    hdrs = [
        "SimpleApp.hpp",
        "SimpleControl.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [],
)

isaac_app(
    name = "simple_project_speed",
    modules = [
        "sight",
        "//apps/simple_project_speed:simple_components",
        "engine_tcp_udp",
    ],
)
