#
# CMake include file for qt library
# Wouter Caarls <w.caarls@tudelft.nl>
#
# 29-03-2010 (wcaarls): Initial revision
#

IF (WITH_GUI)
  IF (WITH_FFMPEG)
    INCLUDE (${WORKSPACE_DIR}/dbl/externals/qtffmpegwrapper/qtffmpegwrapper.cmake)
  ENDIF (WITH_FFMPEG)

  INCLUDE_DIRECTORIES(${WORKSPACE_DIR}/dbl/platform/gui/qt)
  TARGET_LINK_LIBRARIES(${TARGET} dbl_qt)

  # GLWidget needs OpenGL
  FIND_PACKAGE (OpenGL)
  IF ( OPENGL_FOUND)
         INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
  ENDIF( OPENGL_FOUND)
  TARGET_LINK_LIBRARIES(${TARGET} ${OPENGL_LIBRARIES})

  # Qt include dir
  FIND_PACKAGE (Qt4)
  IF (QT_FOUND)
  	INCLUDE_DIRECTORIES( ${QT_INCLUDE_DIR} )
  ENDIF(QT_FOUND)

  IF (NOT __QT_CMAKE_INCLUDED)
    SET(__QT_CMAKE_INCLUDED 1)

    ADD_SUBDIRECTORY(${WORKSPACE_DIR}/dbl/platform/gui/qt ${CMAKE_BINARY_DIR}/externals/dbl/platform/gui/qt)
  ENDIF (NOT __QT_CMAKE_INCLUDED)
ENDIF (WITH_GUI)

