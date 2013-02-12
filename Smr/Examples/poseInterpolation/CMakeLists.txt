PROJECT(SAMSARA_EXAMPLES_POSEINTERPOLATION)

INCLUDE_DIRECTORIES(
			${SAMSARA_CORE_SOURCE_DIR}
			${SAMSARA_MATH_SOURCE_DIR}
			${SAMSARA_COMMON_SOURCE_DIR}
			${SAMSARA_NEWMAT_SOURCE_DIR}
			${SAMSARA_IO_SOURCE_DIR}
			${SAMSARA_UTILS_SOURCE_DIR}
			${OPENGL_INCLUDE_DIR}
			${HORDE3D_INCLUDE_DIR}
			${GLFW_INCLUDE_DIR}
			${LOG4CPLUS_INCLUDE_DIR})

#
# LINK_DIRECTORIES() issue workaround
# (http://www.mail-archive.com/cmake@cmake.org/msg04341.html)
#

IF(WIN32)
	LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH}
		 ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/)
ENDIF(WIN32)

IF(UNIX)
	LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH}
		 ${SAMSARA_SOURCE_DIR}/thirdParty/linux_x86_64/lib/)
ENDIF(UNIX)

ADD_EXECUTABLE(poseInterpolation  MACOSX_BUNDLE
                                  poseInterpolation.cpp)

IF(UNIX)
  IF(APPLE)
    TARGET_LINK_LIBRARIES(poseInterpolation math newmat common core io utils ${OPENGL_LIBRARIES} ${HORDE3D_LIBRARIES} ${GLFW_LIBRARIES} ${X11_LIBRARIES} ${LOG4CPLUS_LIBRARIES} pthread 
    "-dylib_file /System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
  ELSE(APPLE)
    LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH} ${SAMSARA_SOURCE_DIR}/thirdParty/linux_x86_64/lib/)
    TARGET_LINK_LIBRARIES(poseInterpolation math common core io utils newmat Horde3D Horde3DUtils ${OPENGL_LIBRARIES} ${X11_LIBRARIES} glfw Xrandr pthread ${LOG4CPLUS_LIBRARIES})
  ENDIF(APPLE)
ENDIF(UNIX)
IF(WIN32)
  LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH} ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/)
  TARGET_LINK_LIBRARIES(poseInterpolation math newmat common core io utils glfw Horde3D Horde3DUtils ${OPENGL_LIBRARIES} ${LOG4CPLUS_LIBRARIES} )
ENDIF(WIN32)


