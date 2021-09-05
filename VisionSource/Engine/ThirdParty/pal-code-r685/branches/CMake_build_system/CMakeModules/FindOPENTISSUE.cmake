MESSAGE( FATAL_ERROR "Find module not implemented" )

# Locate OpenTissue
# This module defines
# OPENTISSUE_LIBRARY, OPENTISSUE_LIBRARY_DEBUG
#OPENTISSUE_FOUND, if false, do not try to link to OpenTissue
# OPENTISSUE_INCLUDE_DIR, where to find the headers

#FIND_PACKAGE(Boost

#SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
SET(Boost_ADDITIONAL_VERSIONS "1.37.0")
#FIND_PACKAGE(Boost 1.37.0 COMPONENTS numeric )
FIND_PACKAGE(Boost 1.37.0)


SET(OPENTISSUE_HEADERS Cooking Foundation Character Physics PhysXLoader)		# Extensions
SET(OPENTISSUE_INCLUDE_DIR_ERROR "NO")

FOREACH(CUR_DIR ${OPENTISSUE_HEADERS})
	FIND_PATH(OPENTISSUE_${CUR_DIR}_INCLUDE_DIR
		NAMES "${CUR_DIR}.h" "Nx${CUR_DIR}.h"
		HINTS
		$ENV{OPENTISSUE_DIR}
		$ENV{OPENTISSUE_PATH}
		${ADDITIONAL_SEARCH_PATHS}
		PATH_SUFFIXES include "SDKs/${CUR_DIR}/include"
		PATHS
			~/Library/Frameworks
			/Library/Frameworks
			/usr/local
			/usr
			/sw # Fink
			/opt/local # DarwinPorts
			/opt/csw # Blastwave
			/opt
	)

	# Combine all dirs to one variable
	IF(OPENTISSUE_${CUR_DIR}_INCLUDE_DIR AND NOT OPENTISSUE_INCLUDE_DIR_ERROR)
		SET(OPENTISSUE_INCLUDE_DIR "${OPENTISSUE_INCLUDE_DIR};${OPENTISSUE_${CUR_DIR}_INCLUDE_DIR}")
	ELSE()
		SET(OPENTISSUE_INCLUDE_DIR "OPENTISSUE_INCLUDE_DIR-NOTFOUND")
		SET(OPENTISSUE_INCLUDE_DIR_ERROR "YES")
	ENDIF()
ENDFOREACH()



FIND_LIBRARY(OPENTISSUE_LIBRARY
	NAMES box2d
	HINTS
	$ENV{OPENTISSUE_DIR}
	$ENV{OPENTISSUE_PATH}
	${ADDITIONAL_SEARCH_PATHS}
	PATH_SUFFIXES lib64 lib lib/release Library
	PATHS
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local
		/usr
		/sw
		/opt/local
		/opt/csw
		/opt
)

FIND_LIBRARY(OPENTISSUE_LIBRARY_DEBUG 
	NAMES box2dd box2d_d
	HINTS
	$ENV{OPENTISSUE_DIR}
	$ENV{OPENTISSUE_PATH}
	${ADDITIONAL_SEARCH_PATHS}
	PATH_SUFFIXES lib64 lib lib/debug Library
	PATHS
		~/Library/Frameworks
		/Library/Frameworks
		/usr/local
		/usr
		/sw
		/opt/local
		/opt/csw
		/opt
)

SET(OPENTISSUE_FOUND "NO")
IF(OPENTISSUE_LIBRARY AND OPENTISSUE_INCLUDE_DIR)
  SET(OPENTISSUE_FOUND "YES")
ENDIF()

