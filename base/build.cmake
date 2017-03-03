# Setup build environment
set(TARGET grl)

# Make library
add_library(${TARGET} SHARED
            ${SRC}/grl.cpp
            ${SRC}/configurable.cpp
            ${SRC}/predictor.cpp
            ${SRC}/random_generator.cpp
            ${SRC}/signal.cpp
            ${SRC}/statistics.cpp
            ${SRC}/agents/fixed.cpp
            ${SRC}/agents/black_box.cpp
            ${SRC}/agents/td.cpp
            ${SRC}/agents/delayed_td.cpp
            ${SRC}/agents/dyna.cpp
            ${SRC}/agents/sequential.cpp
            ${SRC}/agents/smdp_master.cpp
            ${SRC}/agents/compartmentalized.cpp
            ${SRC}/agents/voluntary.cpp
            ${SRC}/agents/solver.cpp
            ${SRC}/agents/filtering.cpp
            ${SRC}/agents/communicator.cpp
            ${SRC}/discretizers/uniform.cpp
            ${SRC}/discretizers/peaked.cpp
            ${SRC}/discretizers/split.cpp
            ${SRC}/discretizers/policy.cpp
            ${SRC}/environments/acrobot.cpp
            ${SRC}/environments/observation.cpp
            ${SRC}/environments/modeled.cpp
            ${SRC}/environments/sandbox.cpp
            ${SRC}/environments/pendulum.cpp
            ${SRC}/environments/cart_pole.cpp
            ${SRC}/environments/cart_double_pole.cpp
            ${SRC}/environments/pinball.cpp
            ${SRC}/environments/windy.cpp
            ${SRC}/environments/compass_walker/SWModel.cpp
            ${SRC}/environments/compass_walker/compass_walker.cpp
            ${SRC}/environments/noise.cpp
            ${SRC}/environments/swimmer.cpp
            ${SRC}/environments/shaping.cpp
            ${SRC}/environments/flyer2d.cpp
            ${SRC}/environments/puddle.cpp
            ${SRC}/environments/mountain.cpp
            ${SRC}/environments/communicator.cpp
            ${SRC}/experiments/approx_test.cpp
            ${SRC}/experiments/online_learning.cpp
            ${SRC}/experiments/batch_learning.cpp
            ${SRC}/experiments/rpc_env.cpp
            ${SRC}/experiments/multi.cpp
            ${SRC}/exporters/csv.cpp
            ${SRC}/importers/csv.cpp
            ${SRC}/mappings/multisine.cpp
            ${SRC}/mappings/value.cpp
            ${SRC}/mappings/timeline.cpp
            ${SRC}/mappings/puddle.cpp
            ${SRC}/mappings/displacement.cpp
            ${SRC}/optimizers/rwa.cpp
            ${SRC}/policies/random.cpp
            ${SRC}/policies/action.cpp
            ${SRC}/policies/v.cpp
            ${SRC}/policies/q.cpp
            ${SRC}/policies/bounded_q.cpp
            ${SRC}/policies/parameterized.cpp
            ${SRC}/policies/feed_forward.cpp
            ${SRC}/policies/pid.cpp
            ${SRC}/policies/mcts.cpp
            ${SRC}/policies/ucb.cpp
            ${SRC}/policies/state_feedback.cpp
            ${SRC}/policies/noise.cpp
            ${SRC}/predictors/model.cpp
            ${SRC}/predictors/sarsa.cpp
            ${SRC}/predictors/ggq.cpp
            ${SRC}/predictors/ac.cpp
            ${SRC}/predictors/fqi.cpp
            ${SRC}/predictors/mbfqi.cpp
            ${SRC}/predictors/qv.cpp
            ${SRC}/predictors/td.cpp
            ${SRC}/predictors/vi.cpp
            ${SRC}/predictors/advantage.cpp
            ${SRC}/predictors/multi.cpp
            ${SRC}/predictors/dpg.cpp
            ${SRC}/projectors/identity.cpp
            ${SRC}/projectors/normalizing.cpp
            ${SRC}/projectors/peaked.cpp
            ${SRC}/projectors/scaling.cpp
            ${SRC}/projectors/tile_coding.cpp
            ${SRC}/projectors/fourier.cpp
            ${SRC}/projectors/monomial.cpp
            ${SRC}/projectors/grid.cpp
            ${SRC}/projectors/rbf.cpp
            ${SRC}/projectors/multi.cpp
            ${SRC}/projectors/split.cpp
            ${SRC}/representations/linear.cpp
            ${SRC}/representations/ann.cpp
#            ${SRC}/representations/dmp.cpp
            ${SRC}/representations/additive.cpp
            ${SRC}/representations/iterative.cpp
            ${SRC}/representations/dictionary.cpp
            ${SRC}/representations/communicator.cpp
            ${SRC}/samplers/greedy.cpp
            ${SRC}/samplers/softmax.cpp
            ${SRC}/samplers/pada.cpp
            ${SRC}/samplers/ornstein_uhlenbeck.cpp
            ${SRC}/solvers/agent.cpp
            ${SRC}/solvers/vi.cpp
            ${SRC}/traces/enumerated.cpp
            ${SRC}/visualizations/acrobot.cpp
            ${SRC}/visualizations/pendulum.cpp
            ${SRC}/visualizations/cart_pole.cpp
            ${SRC}/visualizations/cart_double_pole.cpp
            ${SRC}/visualizations/compass_walker.cpp
            ${SRC}/visualizations/pinball.cpp
            ${SRC}/visualizations/windy.cpp
            ${SRC}/visualizations/swimmer.cpp
            ${SRC}/visualizations/flyer2d.cpp
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

FIND_PACKAGE (Lua51)

if (LUA51_FOUND)
  message("-- Building Lua deployer")
  include_directories(${LUA_INCLUDE_DIR})

  # Lua deployer
  set (TARGET grll)
  add_executable(${TARGET} ${SRC}/lua_deployer.cpp)
  target_link_libraries(${TARGET} ${LUA_LIBRARIES})
  grl_link_libraries(${TARGET} base)
  install(TARGETS ${TARGET} DESTINATION ${GRL_BIN_DESTINATION})
endif()

# Requestgen
set (TARGET grlg)
add_executable(${TARGET} ${SRC}/requestgen.cpp)
grl_link_libraries(${TARGET} base)
install(TARGETS ${TARGET} DESTINATION ${GRL_BIN_DESTINATION})

install(CODE "execute_process(COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/grlg ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/requests.yaml)")
install(FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/requests.yaml DESTINATION ${GRL_BIN_DESTINATION})
