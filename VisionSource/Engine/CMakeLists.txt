
cmake_minimum_required (VERSION 3.16)
cmake_policy(SET CMP0091 NEW)

project(VisionEngine CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 17)

# Typically you don't care so much for a third party library's tests to be
# run from your own project's code.
set(JSON_BuildTests OFF CACHE INTERNAL "")

set(SOURCE
    Application/ApplicationBase.cpp
    Application/VisionApp.cpp

    Core/OS/OS.cpp
    Core/OS/Timer.cpp

    Core/Event/Event.cpp
    Core/Event/EventManager.cpp

    Input/InputListener.cpp
    Input/InputManager.cpp
    Input/InputSystem.cpp

    FrameWork/System.cpp
    FrameWork/Scene.cpp
    FrameWork/ECS.cpp
    FrameWork/Components.cpp

    ../../DiligentTools/ThirdParty/imgui/imgui_demo.cpp
)

set(INCLUDE
    Application/ApplicationBase.h
    Application/VisionApp.h
    
    Core/OS/Defines.h
    Core/OS/OS.h
    Core/OS/Timer.h

    Core/Event/Event.h
    Core/Event/EventManager.h

    Input/InputListener.h
    Input/InputManager.h
    Input/InputSystem.h

    FrameWork/System.h
    FrameWork/Scene.h
    FrameWork/ECS.h
    FrameWork/Components.h
)

add_subdirectory(ThirdParty/entt-3.8.1)
add_subdirectory(ThirdParty/Gainput)
add_subdirectory(ThirdParty/spdlog-1.9.1)
#add_subdirectory(ThirdParty/eigen-3.4-rc1)
add_subdirectory(ThirdParty/json)

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
    nlohmann_json::nlohmann_json
)