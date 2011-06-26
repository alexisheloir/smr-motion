PROJECT(SAMSARA_SKELETON_MANIPULATION)

INCLUDE_DIRECTORIES(	${SAMSARA_CORE_SOURCE_DIR}
                      ${SAMSARA_ARGH_SOURCE_DIR}
			                ${GLEW_INCLUDE_DIR}
			                ${SAMSARA_MATH_SOURCE_DIR}
			                ${SAMSARA_COMMON_SOURCE_DIR}
			                ${SAMSARA_IO_SOURCE_DIR}
			                ${SAMSARA_NEWMAT_SOURCE_DIR}
	                    ${LOG4CPLUS_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

IF(WIN32)
	LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH}
		 ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/)
ENDIF(WIN32)

ADD_EXECUTABLE(skeletonmanipulation skeletonmanipulation.cpp)

TARGET_LINK_LIBRARIES(skeletonmanipulation argh newmat common core math io ${LIBXML2_LIBRARIES}  ${LOG4CPLUS_LIBRARIES} )
IF(APPLE)
 TARGET_LINK_LIBRARIES(skeletonmanipulation ${ZLIB_LIBRARIES} ${ICONV_LIBRARIES})
ENDIF(APPLE)
