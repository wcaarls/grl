# Setup build environment
set(TARGET grl)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/grl.cpp
#            ${SRC}/agents/black_box.cpp
            ${SRC}/agents/td.cpp
            ${SRC}/discretizers/uniform.cpp
            ${SRC}/environments/modeled.cpp
            ${SRC}/environments/pendulum.cpp
            ${SRC}/experiments/online_learning.cpp
#            ${SRC}/policies/action.cpp
            ${SRC}/policies/q.cpp
#            ${SRC}/predictors/ac.cpp
#            ${SRC}/predictors/fqi.cpp
            ${SRC}/predictors/sarsa.cpp
            ${SRC}/projectors/identity.cpp
            ${SRC}/projectors/tile_coding.cpp
#            ${SRC}/representations/ann.cpp
#            ${SRC}/representations/dmp.cpp
            ${SRC}/representations/linear.cpp
            ${SRC}/samplers/greedy.cpp
            ${SRC}/traces/enumerated.cpp
            ${SRC}/visualizations/value_function.cpp
           )

# Dependencies
target_link_libraries(${TARGET} -lGL -lGLU -lpthread)
grl_link_yaml(${TARGET})

# Deployer
set (TARGET deployer)
add_executable(${TARGET} ${SRC}/deployer.cpp)
target_link_libraries(${TARGET} -ldl)
grl_link_libraries(${TARGET} base)
