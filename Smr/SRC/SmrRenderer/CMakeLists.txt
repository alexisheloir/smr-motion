PROJECT(SAMSARA_RENDERER)

INCLUDE_DIRECTORIES(	
      ${OPENGL_INCLUDE_DIR}
			${GLEW_INCLUDE_DIR}
			${SAMSARA_NEWMAT_SOURCE_DIR}
			${SAMSARA_RENDERER_SOURCE_DIR}
			${SAMSARA_CORE_SOURCE_DIR} 
			${SAMSARA_MATH_SOURCE_DIR} 
			${SAMSARA_COMMON_SOURCE_DIR}
			${LOG4CPLUS_INCLUDE_DIR})

ADD_LIBRARY(renderer STATIC
			SmrSimpleRenderer.cpp
			SmrBasicRenderer.cpp
			Shaders.cpp
	 
			SmrSimpleRenderer.h
			SmrBasicRenderer.h
			SmrRenderer.h
			Shaders.h
)


SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")
