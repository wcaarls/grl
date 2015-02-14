# Setup build environment
set(TARGET addon_gl)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/sample.cpp
            ${SRC}/field.cpp
            ${SRC}/value_function.cpp
            ${SRC}/policy.cpp
           )

# Add dependencies
target_link_libraries(${TARGET} -lGL -lGLU)
grl_link_libraries(${TARGET} base)
