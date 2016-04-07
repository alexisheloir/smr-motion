PROJECT(SAMSARA_CORE)

INCLUDE_DIRECTORIES(
      ${SAMSARA_NEWMAT_SOURCE_DIR}
      ${SAMSARA_LIBXML2_INCLUDE_DIR}
      ${SAMSARA_ICONV_INCLUDE_DIR}
	    ${SAMSARA_COMMON_SOURCE_DIR}
	    ${SAMSARA_CORE_SOURCE_DIR}
	    ${SAMSARA_ACTUATOR_SOURCE_DIR}
	    ${SAMSARA_MATH_SOURCE_DIR}
	    ${SAMSARA_IO_SOURCE_DIR}	    
	    ${LOG4CPLUS_INCLUDE_DIR})

ADD_LIBRARY(core STATIC 
			SmrBlender.cpp
			SmrJoint.cpp
			SmrKinematicJoint.cpp
			SmrBallAndSocket.cpp
			SmrDof.cpp
			SmrKinematicChain.cpp
			SmrMotion.cpp
			SmrMotionPlayer.cpp
			SmrSimpleJointActuator.cpp
			SmrIKConstraint.cpp
			SmrIKSolver.cpp
			SmrGSMMSolver.cpp
			SmrInvJacSolver.cpp
			SmrAnalyticalIKSolver.cpp
			SmrSwivel.cpp

			SmrBlender.h
			SmrJoint.h
			SmrKinematicJoint.h
			SmrKinematicChain.h
			SmrDof.h
			SmrBallAndSocket.h
			SmrSkeleton.h
			SmrMotion.h
			SmrCore.h
			SmrActuator.h
			SmrMotionPlayer.h
			SmrSimpleJointActuator.h
			SmrIKSolver.h
			SmrIKConstraint.h
			SmrGSMMSolver.h
			SmrSkeletonTemplate.h
			SmrInvJacSolver.h
			SmrSkeletonTemplate.incl
			SmrAnalyticalIKSolver.h
			SmrSwivel.h
)

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

TARGET_LINK_LIBRARIES(core common newmat)

SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")

