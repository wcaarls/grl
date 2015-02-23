# Setup build environment
set(TARGET grl)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/grl.cpp
            ${SRC}/agents/fixed.cpp
            ${SRC}/agents/black_box.cpp
            ${SRC}/agents/td.cpp
            ${SRC}/agents/dyna.cpp
            ${SRC}/configurable.cpp
            ${SRC}/discretizers/uniform.cpp
            ${SRC}/environments/observation.cpp
            ${SRC}/environments/modeled.cpp
            ${SRC}/environments/pendulum.cpp
            ${SRC}/environments/cart_pole.cpp
            ${SRC}/experiments/approx_test.cpp
            ${SRC}/experiments/online_learning.cpp
            ${SRC}/policies/random.cpp
            ${SRC}/policies/action.cpp
            ${SRC}/policies/q.cpp
            ${SRC}/policies/bounded_q.cpp
            ${SRC}/policies/parameterized.cpp
            ${SRC}/policies/pid.cpp
            ${SRC}/predictors/model.cpp
            ${SRC}/predictors/sarsa.cpp
            ${SRC}/predictors/ggq.cpp
            ${SRC}/predictors/ac.cpp
#            ${SRC}/predictors/fqi.cpp
            ${SRC}/projectors/identity.cpp
            ${SRC}/projectors/tile_coding.cpp
            ${SRC}/projectors/fourier.cpp
            ${SRC}/representations/linear.cpp
            ${SRC}/representations/multisine.cpp
            ${SRC}/representations/ann.cpp
#            ${SRC}/representations/dmp.cpp
            ${SRC}/samplers/greedy.cpp
            ${SRC}/traces/enumerated.cpp
            ${SRC}/visualizations/pendulum.cpp
            ${SRC}/visualizations/cart_pole.cpp
           )

# Dependencies
target_link_libraries(${TARGET} -lpthread -ldl)
grl_link_yaml(${TARGET})
if (WIN32)
  grl_link_libraries(${TARGET} externals/win32)
endif()

# Deployer
set (TARGET deploy)
add_executable(${TARGET} ${SRC}/deployer.cpp)
grl_link_libraries(${TARGET} base)

# Requestgen
set (TARGET requestgen)
add_executable(${TARGET} ${SRC}/requestgen.cpp)
grl_link_libraries(${TARGET} base)

