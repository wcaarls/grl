#cc_library(
cc_inc_library(
    name = "grl",
#    srcs = ["lib/libgrl.so"],
    hdrs = glob(["include/grl/**/*.h", "include/yaml-cpp/**/*.h"]),
#    strip_include_prefix = "include",
    prefix = "include",
)

cc_binary(
    name = "libaddon_tensorflow.so",
    srcs = [
        "src/representation.cpp",
        "src/representation.h",
    ],
    linkshared = 1,
    copts = ["-Wno-sign-compare"],
    deps = [
        "grl",
        "//tensorflow/core:tensorflow",
    ],
)
