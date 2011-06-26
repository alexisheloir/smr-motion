PROJECT(SAMSARA_PYHON_WRAPPER)

INCLUDE_DIRECTORIES(    ${SAMSARA_PYHON_WRAPPER}
			${SAMSARA_CORE_SOURCE_DIR}
			${SAMSARA_MATH_SOURCE_DIR}
			${SAMSARA_COMMON_SOURCE_DIR}
			${SAMSARA_NEWMAT_SOURCE_DIR}
			${BOOST_INCLUDE_PATH}
		   )

SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")

LINK_DIRECTORIES(       ${LIBRARY_OUTPUT_PATH}
                        ${BOOST_FS_MT_D_LIB_PATH}
                )

ADD_LIBRARY( SMRPy SHARED
             smrWrapper.cpp
           )

TARGET_LINK_LIBRARIES(  SMRPy
                        newmat
                        common
                        core
                        math
						io
                     )

