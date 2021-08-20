
cmake_minimum_required (VERSION 3.16)
cmake_policy(SET CMP0091 NEW)

project(VisionEngine CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE
    Application/ApplicationBase.cpp
    Application/VisionApp.cpp

    Input/InputListener.cpp

    ../../DiligentTools/ThirdParty/imgui/imgui_demo.cpp
)

set(INCLUDE
    Application/ApplicationBase.h
    Application/VisionApp.h
    
    Input/InputListener.h

    ${VINCLUDES}
)

add_subdirectory(ThirdParty/entt-3.8.1)
add_subdirectory(ThirdParty/Gainput)
add_subdirectory(ThirdParty/spdlog-1.9.1)
#add_subdirectory(ThirdParty/eigen-3.4-rc1)

set(SHADERS
)

set(ASSETS
)

add_sample_app("VisionEngine" "VisionSource" "${SOURCE}" "${INCLUDE}" "${SHADERS}" "${ASSETS}")

target_include_directories(VisionEngine
PRIVATE
    ../../DiligentFX/Shaders/PostProcess/ToneMapping/public/
    ../../DiligentFX/Shaders/Common/public/
    ThirdParty/gainput/lib/include/
    ThirdParty/entt-3.8.1
    ThirdParty/spdlog-1.9.1
)

target_link_libraries(VisionEngine
PRIVATE
    Diligent-GraphicsAccessories
    Diligent-TextureLoader
    DiligentFX
    Diligent-AssetLoader
    Diligent-GraphicsTools
    EnTT
    spdlog
    gainputstatic
)