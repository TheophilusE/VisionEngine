cmake_minimum_required (VERSION 3.6)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Project metadata
project(VisionSource)

set(CMAKE_CXX_STANDARD 17)

set(VISION_INCLUDES
    ${VISION_INCLUDES}
    ${CMAKE_CURRENT_LIST_DIR}
CACHE INTERNAL "")

add_subdirectory(Editor)
add_subdirectory(Engine)
add_subdirectory(Shaders)
add_subdirectory(ThirdParty)
