load("@pybind11_bazel//:build_defs.bzl", "pybind_library")

cc_library(
    name = "base_label_function_stl",
    hdrs = ["base_label_function.hpp"],
    visibility = ["//visibility:public"],
)

pybind_library(
    name = "label_function_stl",
    srcs = glob(["*.cpp", "*.hpp"]),
    deps = [
        ":base_label_function_stl",
        "//bark/world:include",
        "//bark/world/map:road_corridor",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/label:include",
    ],    
    linkopts = ["-lpython3.7m"],
    visibility = ["//visibility:public"],
)