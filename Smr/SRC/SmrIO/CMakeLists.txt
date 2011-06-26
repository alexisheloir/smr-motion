PROJECT(SAMSARA_IO)

INCLUDE_DIRECTORIES(
      ${SAMSARA_NEWMAT_SOURCE_DIR}
      ${LIBXML2_INCLUDE_DIR}
      ${SAMSARA_ICONV_INCLUDE_DIR}
	    ${SAMSARA_COMMON_SOURCE_DIR}
	    ${SAMSARA_CORE_SOURCE_DIR}
	    ${SAMSARA_ACTUATOR_SOURCE_DIR}
	    ${SAMSARA_MATH_SOURCE_DIR}
	    ${LOG4CPLUS_INCLUDE_DIR})

ADD_LIBRARY(io STATIC 
			SmrLoaderBvh.cpp
			SmrExporterBvh.cpp
			SmrExporterAcclaim.cpp
			SmrLoaderAcclaim.cpp
			SmrLoaderVsk.cpp
			SmrLoader.h
			SmrExporter.h
			SmrIO.h
)


LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

TARGET_LINK_LIBRARIES(io common newmat)

SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")

