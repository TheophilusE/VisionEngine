# Locate PhysX
# This module defines XXX_FOUND, XXX_INCLUDE_DIRS and XXX_LIBRARIES standard variables

#SET(NOVODEX_HEADERS Cooking Foundation Character Physics PhysXLoader)		# Extensions
SET(NOVODEX_INCLUDE_DIRS)
INCLUDE(FindPackageTargetLibraries)

#FOREACH(CUR_DIR ${NOVODEX_HEADERS})
#	FIND_PATH(NOVODEX_${CUR_DIR}_INCLUDE_DIR
#		NAMES "${CUR_DIR}.h" "Px${CUR_DIR}.h"
#		HINTS
#		${ADDITIONAL_SEARCH_PATHS}
#		PATH_SUFFIXES include Include 
#	)
#
#ENDFOREACH()

FIND_PATH(NOVODEX_INCLUDE_DIR
   NAMES "PxPhysics.h"
   HINTS
   ${ADDITIONAL_SEARCH_PATHS}
   PATH_SUFFIXES include Include 
)

SET(NOVODEX_INCLUDE_DIRS ${NOVODEX_INCLUDE_DIR})

OPTION(BUILD_PHYSX_AGAINST_CHECKED_LIBS "Build against the CHECKED libraries in physx" OFF)
OPTION(BUILD_PHYSX_AGAINST_PROFILE_LIBS "Build against the PROFILE libraries in physx" OFF)

if (bitness EQUAL 64)
   set(PX_BUILD_64 ON)
   set(PX_BUILD_32 OFF)
else()
   set(PX_BUILD_64 OFF)
   set(PX_BUILD_32 ON)
endif()

if (APPLE)
  list(FIND CMAKE_OSX_ARCHITECTURES i386 PX_IDX)
  if (PX_IDX GREATER -1)
    set(PX_BUILD_32 ON)
  endif()
  list(FIND CMAKE_OSX_ARCHITECTURES x86_64 PX_IDX)
  if (PX_IDX GREATER -1)
    set(PX_BUILD_64 ON)
  endif()
endif()

SET(NOVODEX_LIBS 
   LowLevel
   LowLevelCloth
   PhysX3
   PhysX3CharacterKinematic
   PhysX3Common
   PhysX3Cooking
   PhysX3Extensions
   PhysX3Vehicle
   PhysXProfileSDK
   PhysXVisualDebuggerSDK
   PvdRuntime
   PxTask
   SceneQuery
   SimulationController)

macro(find_PX_LIB CUR_LIB)
   STRING(TOLOWER "${CUR_LIB}" CUR_LIB_LOWER)
   
   if (PX_BUILD_64)
      FIND_LIBRARY(NOVODEX_LIBRARY_64_${CUR_LIB}
         NAMES ${CUR_LIB} ${CUR_LIB_LOWER}
         HINTS
            ${ADDITIONAL_SEARCH_PATHS}
         PATH_SUFFIXES Lib/osx64 Lib/win64
      )
   endif()

   if (PX_BUILD_32)
      FIND_LIBRARY(NOVODEX_LIBRARY_32_${CUR_LIB}
         NAMES ${CUR_LIB} ${CUR_LIB_LOWER}
         HINTS
            ${ADDITIONAL_SEARCH_PATHS}
         PATH_SUFFIXES Lib/osx32 Lib/win32
      )
   endif()
endmacro()

FOREACH(CUR_LIB ${NOVODEX_LIBS})
   if (BUILD_PHYSX_AGAINST_CHECKED_LIBS)
      find_PX_LIB(${CUR_LIB}CHECKED)
   elif(BUILD_PHYSX_AGAINST_PROFILE_LIBS)
      find_PX_LIB(${CUR_LIB}PROFILE)
   endif()
   
   if(NOT NOVODEX_LIBRARY_64_${CUR_LIB})
      find_PX_LIB(${CUR_LIB})
   endif()

	# Combine all libs to one variable
	IF(PX_BUILD_64 AND NOVODEX_LIBRARY_64_${CUR_LIB})
		FIND_PACKAGE_ADD_TARGET_LIBRARIES(NOVODEX "${NOVODEX_LIBRARY_64_${CUR_LIB}}" "")
	ENDIF()
   # Combine all libs to one variable
   IF(PX_BUILD_32 AND NOVODEX_LIBRARY_32_${CUR_LIB})
      FIND_PACKAGE_ADD_TARGET_LIBRARIES(NOVODEX "${NOVODEX_LIBRARY_32_${CUR_LIB}}" "")
   ENDIF()
ENDFOREACH()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(NOVODEX DEFAULT_MSG NOVODEX_LIBRARIES NOVODEX_INCLUDE_DIRS)

IF(NOVODEX_FOUND)
	# NOVODEX_LIBRARIES has been set before
	# NOVODEX_INCLUDE_DIRS has been set before
ELSE()
	SET(NOVODEX_LIBRARIES)
	SET(NOVODEX_INCLUDE_DIRS)
ENDIF()
