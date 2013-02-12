# This is the root Cmake file of SAMSARA PROJECT
# author : Alexis Heloir/Nicolas Courty
# Date of birth : December, 6th, 2005
#

cmake_minimum_required(VERSION 2.6)
#ENABLE_TESTING()
PROJECT(SAMSARA)

SET(CMAKE_BUILD_TYPE Release)

SET(CMAKE_VERBOSE_MAKEFILE true)

SET ( SAMSARA_ICONV_INCLUDE_DIR
    "${SAMSARA_SOURCE_DIR}/thirdParty/Universal/iconv/include"
    CACHE PATH
    "Path to the iconv include directory."
)

SET ( SAMSARA_DATA_PATH
    "${SAMSARA_SOURCE_DIR}/../Data"
    CACHE PATH
    "Path to the data directory."
)

#set(LOG4CPLUS_BUILD_TESTING FALSE)
#set(UNICODE FALSE)

#SET(LOG4CPLUS_INCLUDE ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/log4cplus-1.0.3/include)
#SET(LOG4CPLUS_LIBRARIES log4cplus) #U for Unicode, S for static
#ADD_DEFINITIONS (-DLOG4CPLUS_STATIC)
MESSAGE("LOG4CPLUS_LIBRARIES is now: ${LOG4CPLUS_LIBRARIES}")
MESSAGE("CMAKE_BUILD_TYPE is now: ${CMAKE_BUILD_TYPE}")

#MESSAGE("log4cplus is ${log4cplus}")


SET(LOG4CPLUS_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/log4cplus-1.1.1-rc3/include)
IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  SET(LOG4CPLUS_LIBRARIES log4cplusD) #U for Unicode, S for Static, D for Debug
ELSE(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  SET(LOG4CPLUS_LIBRARIES log4cplus) #U for Unicode, S for Static, D for Debug
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")

SET(HORDE3D_ROOT_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/Horde3D_SDK_1.0.0_Beta5 )
SET(HORDE3D_INCLUDE_DIR ${HORDE3D_ROOT_DIR}/Horde3D/Bindings/C++)

INCLUDE(FindOpenGL)

#SET(GLUT_Xmu_LIBRARY /usr/lib/libglut.a) 


IF(UNIX)

  INCLUDE(FindGLUT)

  IF(APPLE)
    SET(LIBRARY_OUTPUT_PATH ${SAMSARA_SOURCE_DIR}/../macBuild/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${SAMSARA_SOURCE_DIR}/../macBuild/bin)
    SET(ARCHIVE_OUTPUT_DIRECTORY ${SAMSARA_SOURCE_DIR}/../macBuild/build)
  
    INCLUDE(FindX11)
 
    SET(GLUT_LIBRARIES "-framework glut" objc)
  
    SET(HORDE3D_LIBRARIES Horde3D) #"-framework Horde3D")
   
    SET(GLFW_INCLUDE_DIR ${HORDE3D_ROOT_DIR}/Horde3D/Dependencies/Include)
    SET(GLFW_LIBRARIES "-framework Carbon" libglfw.a )

    #SET ( LIBXML2_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/mac/include/libxml2)
    #SET ( LIBXML2_LIBRARIES  ${SAMSARA_SOURCE_DIR}/thirdParty/mac/lib/libxml2.a)
    INCLUDE(FindLibXml2)
    INCLUDE(FindZlib)


    SET(ICONV_LIBRARIES /usr/lib/libiconv.dylib)
    #SET(ZLIB_LIBRARIES ${SAMSARA_SOURCE_DIR}/thirdParty/mac/lib/libz.a)

  ELSE(APPLE)
    
    SET(GLFW_INCLUDE_DIR ${HORDE3D_ROOT_DIR}/Horde3D/Dependencies/Include)
    SET(LIBRARY_OUTPUT_PATH ${SAMSARA_SOURCE_DIR}/../linuxBuild/lib/Release)
    SET(EXECUTABLE_OUTPUT_PATH ${SAMSARA_SOURCE_DIR}/../linuxBuild/bin/Release)
    SET(ARCHIVE_OUTPUT_DIRECTORY ${SAMSARA_SOURCE_DIR}/../linuxBuild/build)
    
    SET(GLUT_Xi_LIBRARY /usr/lib/libglut.a) 
    SET(GLUT_Xmu_LIBRARY /usr/lib/libglut.a) 
    
    INCLUDE(FindLibXml2)
  ENDIF(APPLE)


  SET(GLEW_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/glew/include)
  SET(GLEW_LIBRARIES glew)


  # In Linux the includes are generally in the system paths. I.e/usr/include/boost
  # In Linux the libs are generally in the system paths. I.e. /usr/lib ->
  # If you build and install boost yourself, the headers are in:/usr/local/include/boost-1_33/
  # and the libraries are in: /usr/local/lib/
  #  "Path to the Boost multi-threaded thread debug libraries.")
  # SET ( BOOST_INCLUDE_PATH
  #      /usr/local/include/boost-1_33/
  #      CACHE PATH
  #      "Path to the Boost include files."
  #)

ENDIF(UNIX)

IF(WIN32)

    #SET(CMAKE_MFC_FLAG 1)
    
    #SET(CMAKE_CXX_FLAGS_RELEASE "/MT /O2 /ob2 /W3"  )
    
    SET(GTEST_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/gtest-1.5.0/include)
	add_definitions (-D_VARIADIC_MAX=10)
    SET(GLFW_INCLUDE_DIR ${HORDE3D_ROOT_DIR}/Horde3D/Dependencies/Include)
    SET(LIBRARY_OUTPUT_PATH 
        "${SAMSARA_SOURCE_DIR}/../win32Build/lib"
        CACHE PATH
	"Where the binaries and .dll files go")
    SET(EXECUTABLE_OUTPUT_PATH 
        "${SAMSARA_SOURCE_DIR}/../win32Build/bin"
	CACHE  PATH
	"Where the binaries and .dll files go")
    SET(ARCHIVE_OUTPUT_DIRECTORY ${SAMSARA_SOURCE_DIR}/../win32Build/build)
    
    INCLUDE(FindQt)
    SET(GUI "Win32")
    SET(GLUT_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/win32/include)
    SET(GLEW_INCLUDE_DIR ${SAMSARA_SOURCE_DIR}/thirdParty/Universal/glew/include)
    SET(GLUT_LIBRARIES ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/glut32.lib)
    SET(GLEW_LIBRARIES ${SAMSARA_SOURCE_DIR}/thirdParty/win32/lib/glew32.lib)
    SET(GLFW_LIBRARIES glfw.lib)
    SET(HORDE3D_LIBRARIES Horde3D_vc8.lib Horde3DUtils_vc8.lib)

    SET ( LIBXML2_INCLUDE_DIR
        "${SAMSARA_SOURCE_DIR}/thirdParty/Universal/libxml2/include"
        CACHE PATH
        "Path to the libxml2 include directory."
	)

    SET ( LIBXML2_LIBRARIES
	  libxml2
	)

ENDIF(WIN32)

SUBDIRS(thirdParty SRC Examples tests Tutorials)


# Somehow Horde 3D is putting its libraries in the wrong directory: lib.
# We copy them from the 'lib' dir to the 'bin' dir.
IF(WIN32)

	# Dummy target to add the custom commands
	add_custom_target(Horde3DCopyFix ALL
		DEPENDS Horde3D Horde3DUtils
	)

	message("Horde3D libraries will be copied in the right location: ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
	
	add_custom_command (
		TARGET Horde3DCopyFix
		POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Horde3D.dll ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}
	)

	add_custom_command (
		TARGET Horde3DCopyFix
		POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Horde3DUtils.dll ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}
	)
	
ENDIF(WIN32)
