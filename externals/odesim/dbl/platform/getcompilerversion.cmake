IF (NOT DEFINED COMPILER_VERSION)
  IF (CMAKE_COMPILER_IS_GNUCXX)
    GET_FILENAME_COMPONENT(COMPILER_VERSION ${CMAKE_CXX_COMPILER} NAME)
    SET(COMPILER_VERSION "-${COMPILER_VERSION}")

    # Find GXX version -- this was not sufficient, since we got multiple gcc 4.1 (cross)compilers that compile against different glibc versions..
  #  EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER}  ARGS -dumpversion  OUTPUT_VARIABLE COMPILER_VERSION)
    # Remove the carriage return from -dumpversion and prepend "_gcc"
  #  STRING(REPLACE "\n" "" COMPILER_VERSION ${COMPILER_VERSION})
  #  SET(COMPILER_VERSION "-gcc${COMPILER_VERSION}")
    # Report
    MESSAGE (STATUS "Compiler version: " ${COMPILER_VERSION})
  ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF(NOT DEFINED COMPILER_VERSION)

