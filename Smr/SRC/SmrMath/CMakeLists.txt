PROJECT(SAMSARA_MATH)

INCLUDE_DIRECTORIES(
        ${SAMSARA_NEWMAT_SOURCE_DIR}
		    ${SAMSARA_MATH_SOURCE_DIR}
		    ${SAMSARA_COMMON_SOURCE_DIR}
		    ${SAMSARA_CORE_SOURCE_DIR}
		    ${LOG4CPLUS_INCLUDE_DIR}
		   )

ADD_LIBRARY(math STATIC
			SmrPCA.cpp
			SmrDTW.cpp
			SmrRandom.cpp
			SmrVector3.h
			SmrTimeSerie.h
			SmrRandom.h
			SmrPCA.h
			SmrDTW.h
			SmrMath.h
			SmrQuaternion.h
			SmrParticleFilter.h)


SOURCE_GROUP(Inline REGULAR_EXPRESSION "[.]inl$")
SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")
