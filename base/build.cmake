# Setup build environment
set(TARGET grl)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/grl.cpp
            ${SRC}/agents/fixed.cpp
#            ${SRC}/agents/black_box.cpp
            ${SRC}/agents/td.cpp
#            ${SRC}/agents/dyna.cpp
            ${SRC}/agents/exclusive.cpp
            ${SRC}/agents/sequential.cpp
#            ${SRC}/agents/compartmentalized.cpp
#            ${SRC}/agents/solver.cpp
            ${SRC}/configurable.cpp
            ${SRC}/discretizers/uniform.cpp
            ${SRC}/discretizers/peaked.cpp
#            ${SRC}/environments/acrobot.cpp
            ${SRC}/environments/observation.cpp
            ${SRC}/environments/modeled.cpp
            ${SRC}/environments/pendulum.cpp
            ${SRC}/environments/cart_pole.cpp
#            ${SRC}/environments/pinball.cpp
#            ${SRC}/environments/windy.cpp
            ${SRC}/environments/compass_walker/SWModel.cpp
            ${SRC}/environments/compass_walker/compass_walker.cpp
#            ${SRC}/environments/noise.cpp
#            ${SRC}/environments/swimmer.cpp
#            ${SRC}/experiments/approx_test.cpp
            ${SRC}/experiments/online_learning.cpp
#            ${SRC}/experiments/batch_learning.cpp
            ${SRC}/experiments/rpc_env.cpp
            ${SRC}/exporters/csv.cpp
            ${SRC}/importers/csv.cpp
            ${SRC}/policies/random.cpp
            ${SRC}/policies/action.cpp
            ${SRC}/policies/v.cpp
            ${SRC}/policies/q.cpp
            ${SRC}/policies/bounded_q.cpp
            ${SRC}/policies/parameterized.cpp
#            ${SRC}/policies/pid.cpp
#            ${SRC}/policies/mcts.cpp
#            ${SRC}/policies/ucb.cpp
            ${SRC}/policies/state_feedback.cpp
            ${SRC}/predictor.cpp
            ${SRC}/predictors/model.cpp
            ${SRC}/predictors/sarsa.cpp
#            ${SRC}/predictors/ggq.cpp
            ${SRC}/predictors/ac.cpp
#            ${SRC}/predictors/fqi.cppЗ
            ${SRC}/predictors/qv.cpp
            ${SRC}/predictors/td.cpp
            ${SRC}/predictors/vi.cpp
            ${SRC}/predictors/advantage.cpp
            ${SRC}/projectors/identity.cpp
            ${SRC}/projectors/normalizing.cpp
            ${SRC}/projectors/peaked.cpp
            ${SRC}/projectors/scaling.cpp
            ${SRC}/projectors/tile_coding.cpp
#            ${SRC}/projectors/fourier.cpp
            ${SRC}/projectors/grid.cpp
            ${SRC}/representations/linear.cpp
#            ${SRC}/representations/multisine.cpp
            ${SRC}/representations/ann.cpp
#            ${SRC}/representations/dmp.cpp
            ${SRC}/representations/additive.cpp
            ${SRC}/samplers/greedy.cpp
            ${SRC}/samplers/softmax.cpp
            ${SRC}/solvers/agent.cpp
            ${SRC}/solvers/vi.cpp
            ${SRC}/traces/enumerated.cpp
#            ${SRC}/visualizations/acrobot.cpp
            ${SRC}/visualizations/pendulum.cpp
            ${SRC}/visualizations/cart_pole.cpp
            ${SRC}/visualizations/compass_walker.cpp
#            ${SRC}/visualizations/pinball.cpp
#            ${SRC}/visualizations/windy.cpp
#            ${SRC}/visualizations/swimmer.cpp
           )

# Dependencies
target_link_libraries(${TARGET} -lpthread -ldl)
grl_link_libraries(${TARGET} externals/yaml-cpp externals/itc)
install(TARGETS ${TARGET} DESTINATION ${GRL_LIB_DESTINATION})
install(DIRECTORY ${SRC}/../include/grl DESTINATION ${GRL_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

# Deployer
set (TARGET grld)
add_executable(${TARGET} ${SRC}/deployer.cpp)
grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_BIN_DESTINATION})

# Requestgen
set (TARGET grlg)
add_executable(${TARGET} ${SRC}/requestgen.cpp)
grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_BIN_DESTINATION})

install(CODE "execute_process(COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/grlg ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/requests.yaml)")
install(FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/requests.yaml DESTINATION ${GRL_BIN_DESTINATION})
