# Setup build environment
set(TARGET addon_glut)

# Build library
add_library(${TARGET} SHARED
            ${SRC}/glut.cpp
           )

# Add dependencies
target_link_libraries(${TARGET} -lglut)
grl_link_libraries(${TARGET} base)
