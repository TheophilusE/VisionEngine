cmake_minimum_required (VERSION 3.8)

project(VisionEngine CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 17)

set(ENGINE_HEADERS
    Application/Application.h
    Application/App.h

    Core/OS/Defines.h
    Core/OS/OS.h
    Core/OS/Timer.h

    Input/InputListener.h
    Input/InputManager.h

    Core/Event/Event.h
    Core/Event/EventManager.h
    
    FrameWork/System.h
)

set(ENGINE_SOURCES
    Application/Application.cpp
    Application/App.cpp
    
    Core/OS/Defines.cpp
    Core/OS/OS.cpp
    Core/OS/Timer.cpp

    Input/InputListener.cpp
    Input/InputManager.cpp

    Core/Event/Event.cpp
    Core/Event/EventManager.cpp

    FrameWork/System.cpp
)

if(PLATFORM_MACOS)
    add_executable(VisionEngine MACOSX_BUNDLE ${ENGINE_HEADERS} ${ENGINE_SOURCES})
else()
    add_executable(VisionEngine ${ENGINE_HEADERS} ${ENGINE_SOURCES})
endif()

set_target_properties(VisionEngine PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_compile_features(VisionEngine PUBLIC cxx_std_17)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ENGINE_HEADERS} ${ENGINE_SOURCES})
target_include_directories(VisionEngine PRIVATE
     "../../DiligentCore"
     #"../ThirdParty/eigen-3.4-rc1"
     ../ThirdParty/entt-3.8.1
     ../ThirdParty/spdlog-1.9.1
     ../ThirdParty/Gainput/include/
     ../ThirdParty/Gainput/source/
     )
if(METAL_SUPPORTED)
    target_include_directories(VisionEngine PRIVATE "../../DiligentCorePro")
endif()

target_compile_definitions(VisionEngine PRIVATE UNICODE)

set_target_properties(VisionEngine PROPERTIES
    FOLDER VisionSource
)

target_compile_definitions(VisionEngine PRIVATE ENGINE_DLL=1)
target_link_libraries(VisionEngine
PRIVATE
    Diligent-Common
    Diligent-GraphicsTools
    glfw
    Diligent-TextureLoader
    Diligent-Imgui
    Diligent-GraphicsAccessories
    Diligent-TextureLoader
    DiligentFX
    Diligent-AssetLoader
    EnTT
    spdlog
    Gainput
)

if(D3D11_SUPPORTED)
    target_link_libraries(VisionEngine PRIVATE Diligent-GraphicsEngineD3D11-shared)
endif()
if(D3D12_SUPPORTED)
    target_link_libraries(VisionEngine PRIVATE Diligent-GraphicsEngineD3D12-shared)
endif()
if(GL_SUPPORTED)
    target_link_libraries(VisionEngine PRIVATE Diligent-GraphicsEngineOpenGL-shared)
endif()
if(VULKAN_SUPPORTED)
    target_link_libraries(VisionEngine PRIVATE Diligent-GraphicsEngineVk-shared)
endif()
if(METAL_SUPPORTED)
    target_link_libraries(VisionEngine PRIVATE Diligent-GraphicsEngineMetal-shared)
endif()
if(PLATFORM_WIN32)
    copy_required_dlls(VisionEngine)
endif()

if(PLATFORM_MACOS AND VULKAN_LIB_PATH)
    # Configure rpath so that executables can find vulkan library
    set_target_properties(VisionEngine PROPERTIES
        BUILD_RPATH "${VULKAN_LIB_PATH}"
    )
endif()
