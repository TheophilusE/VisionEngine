
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

    Render/Renderer.cpp

    Core/Math/VMath.cpp

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

    Render/Renderer.h

    Core/Math/VMath.h
)

add_subdirectory(ThirdParty/entt-3.8.1)
add_subdirectory(ThirdParty/Gainput)
add_subdirectory(ThirdParty/spdlog-1.9.1)
add_subdirectory(ThirdParty/eigen-3.4.0)
add_subdirectory(ThirdParty/json)

set(SHADERS
    assets/shaders/BoundBox.vsh
    assets/shaders/BoundBox.psh
    assets/shaders/env_map.vsh
    assets/shaders/env_map.psh
    assets/shaders/MeshVS.vsh
    assets/shaders/MeshPS.psh
    ../../DiligentFX/Shaders/Common/public/Shadows.fxh
    assets/shaders/HostSharedTerrainStructs.fxh
	../../DiligentFX/Shaders/PostProcess/EpipolarLightScattering/public/EpipolarLightScatteringStructures.fxh
    ../../DiligentFX/Shaders/PostProcess/EpipolarLightScattering/public/EpipolarLightScatteringFunctions.fxh
)

set(EXTERNAL_SHADERS
    ../../DiligentFX/Shaders/PostProcess/ToneMapping/public/ToneMappingStructures.fxh
    ../../DiligentFX/Shaders/PostProcess/ToneMapping/public/ToneMapping.fxh
    ../../DiligentFX/Shaders/Common/public/BasicStructures.fxh
    ../../DiligentFX/Shaders/Common/public/ShaderUtilities.fxh
)

set(ALL_SHADERS ${SHADERS} ${EXTERNAL_SHADERS})

set(TEXTURES
    assets/textures/sky.dds
)

set(MODELS
)

set(ASSETS
    ${MODELS}
    ${TEXTURES}
)
add_sample_app("VisionEngine" "VisionSource" "${SOURCE}" "${INCLUDE}" "${ALL_SHADERS}" "${ASSETS}")

target_include_directories(VisionEngine
PRIVATE
    ../../DiligentFX/Shaders/PostProcess/ToneMapping/public/
    ../../DiligentFX/Shaders/Common/public/
    ThirdParty/gainput/lib/include/
    ThirdParty/entt-3.8.1
    ThirdParty/spdlog-1.9.1
    ThirdParty/eigen-3.4.0/
    ThirdParty/bullet3-3.17/src/
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
    Bullet
)

foreach(FILE ${EXTERNAL_SHADERS})
    # Copy external shaders
    add_custom_command(TARGET VisionEngine PRE_BUILD 
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/${FILE}" "${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders"
    )
endforeach(FILE)

add_custom_command(TARGET VisionEngine PRE_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../../DiligentFX/Shaders/Common/public/BasicStructures.fxh" "${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/../../DiligentFX/Shaders/Common/public/Shadows.fxh" "${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders"
)

set_source_files_properties(${ALL_SHADERS} PROPERTIES
    VS_DEPLOYMENT_LOCATION "shaders"
    MACOSX_PACKAGE_LOCATION Resources/shaders
)

set_source_files_properties(${TEXTURES} PROPERTIES
    VS_DEPLOYMENT_LOCATION "textures"
    MACOSX_PACKAGE_LOCATION "Resources/textures"
)

source_group("assets" FILES ${ASSETS})
source_group("assets\\shaders" FILES ${SHADERS})
source_group("assets\\shaders\\external" FILES ${EXTERNAL_SHADERS})
source_group("assets\\textures" FILES ${TEXTURES})