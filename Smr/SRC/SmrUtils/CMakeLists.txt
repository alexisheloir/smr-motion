PROJECT(SAMSARA_UTILS)

INCLUDE_DIRECTORIES(
      ${SAMSARA_NEWMAT_SOURCE_DIR}
      ${LIBXML2_INCLUDE_DIR}
      ${SAMSARA_ICONV_INCLUDE_DIR}
	    ${SAMSARA_COMMON_SOURCE_DIR}
	    ${SAMSARA_CORE_SOURCE_DIR}
	    ${SAMSARA_ACTUATOR_SOURCE_DIR}
	    ${SAMSARA_MATH_SOURCE_DIR}
        ${GLUT_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIR}
        ${HORDE3D_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
        ${LOG4CPLUS_INCLUDE_DIR})

ADD_LIBRARY(utils STATIC 
			SmrUtils.cpp
			HordeCamera.cpp
			HordeWindow.cpp
			HordeApplication.cpp
			HordeResource.cpp
			HordeNode.cpp
			HordeBones.cpp

			SmrUtils.h
			HordeCamera.h
			HordeWindow.h
			HordeApplication.h
			HordeResource.h
			HordeNode.h
			HordeBones.h
)


LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

TARGET_LINK_LIBRARIES(utils common newmat glfw Horde3D Horde3DUtils ${OPENGL_LIBRARIES})

SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")

