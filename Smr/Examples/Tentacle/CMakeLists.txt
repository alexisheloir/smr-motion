PROJECT(SAMSARA_TENTACLE)

INCLUDE_DIRECTORIES(
			${SAMSARA_CORE_SOURCE_DIR}
			${SAMSARA_MATH_SOURCE_DIR}
			${SAMSARA_COMMON_SOURCE_DIR}
			${SAMSARA_NEWMAT_SOURCE_DIR}
			${SAMSARA_IO_SOURCE_DIR}
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

IF(MSVC)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup ")
ENDIF(MSVC)

IF(MSVC)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ENTRY:mainCRTStartup ")
ENDIF(MSVC)

IF(UNIX)
	LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH}
		 ${SAMSARA_SOURCE_DIR}/thirdParty/linux_x86_64/lib/)
ENDIF(UNIX)

ADD_EXECUTABLE(tentacle WIN32 MACOSX_BUNDLE
                        Tentacle.cpp
                        main.cpp
			            Tentacle.h)

IF(UNIX)
  IF(APPLE)
    TARGET_LINK_LIBRARIES(tentacle math newmat Horde3D Horde3DUtils common core io ${OPENGL_LIBRARIES} ${HORDE3D_LIBRARIES} ${GLFW_LIBRARIES} ${X11_LIBRARIES} ${LOG4CPLUS_LIBRARIES} pthread 
    "-dylib_file /System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
  ELSE(APPLE)
    LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH} ${SAMSARA_SOURCE_DIR}/thirdParty/linux_x86_64/lib/)
    TARGET_LINK_LIBRARIES(tentacle math common core io newmat Horde3D Horde3DUtils ${OPENGL_LIBRARIES} ${X11_LIBRARIES} glfw Xrandr pthread ${LOG4CPLUS_LIBRARIES} )
  ENDIF(APPLE)
ENDIF(UNIX)
IF(WIN32)
  LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH} ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/)
  TARGET_LINK_LIBRARIES(tentacle math newmat common core io Horde3D Horde3DUtils glfw ${LOG4CPLUS_LIBRARIES} )
ENDIF(WIN32)


INSTALL(TARGETS tentacle DESTINATION bin CONFIGURATIONS Release)
