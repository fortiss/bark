config_setting(
    name = "_ltl_rules",
    define_values = {
        "ltl_rules": "true",
    },
)

cc_library(
    name = "evaluator_stl",
    srcs = ["evaluator_stl.cpp"],
    hdrs = ["evaluator_stl.hpp"],
    copts = select({
        ":_ltl_rules": ["-D LTL_RULES"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/label_functions:label_function",        
        "//bark/world/evaluation/stl/label_functions:label_function_stl",        
    ] + select({
        ":_ltl_rules": [
            "@rule_monitor_project//ltl:rule_monitor",
        ],
        "//conditions:default": [],
    }),
)
