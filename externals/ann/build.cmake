# Setup build environment
SET(TARGET ann)

# Build library
ADD_LIBRARY(${TARGET} SHARED
            ${SRC}/ANN.cpp
            ${SRC}/brute.cpp
            ${SRC}/kd_tree.cpp
            ${SRC}/kd_util.cpp
            ${SRC}/kd_split.cpp
            ${SRC}/kd_dump.cpp
            ${SRC}/kd_search.cpp
            ${SRC}/kd_pr_search.cpp
            ${SRC}/kd_fix_rad_search.cpp
            ${SRC}/bd_tree.cpp
            ${SRC}/bd_search.cpp
            ${SRC}/bd_pr_search.cpp
            ${SRC}/bd_fix_rad_search.cpp
            ${SRC}/perf.cpp
           )
