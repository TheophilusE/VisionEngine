/*
 *  Copyright (c) 2021 Theophilus Eriata
*/


#ifndef ENGINE_DLL
#    define ENGINE_DLL 1
#endif

#ifndef D3D11_SUPPORTED
#    define D3D11_SUPPORTED 0
#endif

#ifndef D3D12_SUPPORTED
#    define D3D12_SUPPORTED 0
#endif

#ifndef GL_SUPPORTED
#    define GL_SUPPORTED 0
#endif

#ifndef VULKAN_SUPPORTED
#    define VULKAN_SUPPORTED 0
#endif

#ifndef METAL_SUPPORTED
#    define METAL_SUPPORTED 0
#endif

#if PLATFORM_WIN32
#    define GLFW_EXPOSE_NATIVE_WIN32 1
#endif

#if PLATFORM_LINUX
#    define GLFW_EXPOSE_NATIVE_X11 1
#endif

#if PLATFORM_MACOS
#    define GLFW_EXPOSE_NATIVE_COCOA 1
#endif

#if D3D11_SUPPORTED
#    include "Graphics/GraphicsEngineD3D11/interface/EngineFactoryD3D11.h"
#endif
#if D3D12_SUPPORTED
#    include "Graphics/GraphicsEngineD3D12/interface/EngineFactoryD3D12.h"
#endif
#if GL_SUPPORTED
#    include "Graphics/GraphicsEngineOpenGL/interface/EngineFactoryOpenGL.h"
#endif
#if VULKAN_SUPPORTED
#    include "Graphics/GraphicsEngineVulkan/interface/EngineFactoryVk.h"
#endif
#if METAL_SUPPORTED
#    include "Graphics/GraphicsEngineMetal/interface/EngineFactoryMtl.h"
#endif

#if PLATFORM_WIN32
#    undef GetObject
#    undef CreateWindow
#endif

#include "Application.h"
#include "GLFW/glfw3native.h"

#if PLATFORM_MACOS
extern void* GetNSWindowView(GLFWwindow* wnd);
#endif

namespace Vision
{

Application::Application()
{
}

Application::~Application()
{
    if (m_pImmediateContext)
        m_pImmediateContext->Flush();

    m_pSwapChain        = nullptr;
    m_pImmediateContext = nullptr;
    m_pDevice           = nullptr;

    if (m_Window)
    {
        glfwDestroyWindow(m_Window);
        glfwTerminate();
    }
}

bool Application::CreateWindow(const char* Title, int Width, int Height, int GlfwApiHint)
{
    if (glfwInit() != GLFW_TRUE)
        return false;

    glfwWindowHint(GLFW_CLIENT_API, GlfwApiHint);
    if (GlfwApiHint == GLFW_OPENGL_API)
    {
        // We need compute shaders, so request OpenGL 4.2 at least
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    }

    m_Window = glfwCreateWindow(Width, Height, Title, nullptr, nullptr);
    if (m_Window == nullptr)
    {
        LOG_ERROR_MESSAGE("Failed to create GLFW window");
        return false;
    }

    glfwSetWindowUserPointer(m_Window, this);
    glfwSetFramebufferSizeCallback(m_Window, &GLFW_ResizeCallback);
    glfwSetKeyCallback(m_Window, &GLFW_KeyCallback);
    glfwSetMouseButtonCallback(m_Window, &GLFW_MouseButtonCallback);
    glfwSetCursorPosCallback(m_Window, &GLFW_CursorPosCallback);
    glfwSetScrollCallback(m_Window, &GLFW_MouseWheelCallback);

    glfwSwapInterval(m_bVSync ? 1 : 0);

    glfwSetWindowSizeLimits(m_Window, 320, 240, GLFW_DONT_CARE, GLFW_DONT_CARE);
    return true;
}

bool Application::InitEngine(RENDER_DEVICE_TYPE DevType)
{
#if PLATFORM_WIN32
    Win32NativeWindow Window{glfwGetWin32Window(m_Window)};
#endif
#if PLATFORM_LINUX
    LinuxNativeWindow Window;
    Window.WindowId = glfwGetX11Window(m_Window);
    Window.pDisplay = glfwGetX11Display();
    if (DevType == RENDER_DEVICE_TYPE_GL)
        glfwMakeContextCurrent(m_Window);
#endif
#if PLATFORM_MACOS
    MacOSNativeWindow Window;
    if (DevType == RENDER_DEVICE_TYPE_GL)
        glfwMakeContextCurrent(m_Window);
    else
        Window.pNSView = GetNSWindowView(m_Window);
#endif

    SwapChainDesc SCDesc;
    switch (DevType)
    {
#if D3D11_SUPPORTED
        case RENDER_DEVICE_TYPE_D3D11:
        {
#    if ENGINE_DLL
            // Load the dll and import GetEngineFactoryD3D11() function
            auto* GetEngineFactoryD3D11 = LoadGraphicsEngineD3D11();
#    endif
            auto* pFactoryD3D11 = GetEngineFactoryD3D11();

            EngineD3D11CreateInfo EngineCI;
            pFactoryD3D11->CreateDeviceAndContextsD3D11(EngineCI, &m_pDevice, &m_pImmediateContext);
            pFactoryD3D11->CreateSwapChainD3D11(m_pDevice, m_pImmediateContext, SCDesc, FullScreenModeDesc{}, Window, &m_pSwapChain);
        }
        break;
#endif // D3D11_SUPPORTED


#if D3D12_SUPPORTED
        case RENDER_DEVICE_TYPE_D3D12:
        {
#    if ENGINE_DLL
            // Load the dll and import GetEngineFactoryD3D12() function
            auto* GetEngineFactoryD3D12 = LoadGraphicsEngineD3D12();
#    endif
            auto* pFactoryD3D12 = GetEngineFactoryD3D12();

            EngineD3D12CreateInfo EngineCI;
            pFactoryD3D12->CreateDeviceAndContextsD3D12(EngineCI, &m_pDevice, &m_pImmediateContext);
            pFactoryD3D12->CreateSwapChainD3D12(m_pDevice, m_pImmediateContext, SCDesc, FullScreenModeDesc{}, Window, &m_pSwapChain);
        }
        break;
#endif // D3D12_SUPPORTED


#if GL_SUPPORTED
        case RENDER_DEVICE_TYPE_GL:
        {
#    if EXPLICITLY_LOAD_ENGINE_GL_DLL
            // Load the dll and import GetEngineFactoryOpenGL() function
            auto GetEngineFactoryOpenGL = LoadGraphicsEngineOpenGL();
#    endif
            auto* pFactoryOpenGL = GetEngineFactoryOpenGL();

            EngineGLCreateInfo EngineCI;
            EngineCI.Window = Window;
            pFactoryOpenGL->CreateDeviceAndSwapChainGL(EngineCI, &m_pDevice, &m_pImmediateContext, SCDesc, &m_pSwapChain);
        }
        break;
#endif // GL_SUPPORTED


#if VULKAN_SUPPORTED
        case RENDER_DEVICE_TYPE_VULKAN:
        {
#    if EXPLICITLY_LOAD_ENGINE_VK_DLL
            // Load the dll and import GetEngineFactoryVk() function
            auto* GetEngineFactoryVk = LoadGraphicsEngineVk();
#    endif
            auto* pFactoryVk = GetEngineFactoryVk();

            EngineVkCreateInfo EngineCI;
            pFactoryVk->CreateDeviceAndContextsVk(EngineCI, &m_pDevice, &m_pImmediateContext);
            pFactoryVk->CreateSwapChainVk(m_pDevice, m_pImmediateContext, SCDesc, Window, &m_pSwapChain);
        }
        break;
#endif // VULKAN_SUPPORTED

#if METAL_SUPPORTED
        case RENDER_DEVICE_TYPE_METAL:
        {
            auto* pFactoryMtl = GetEngineFactoryMtl();

            EngineMtlCreateInfo EngineCI;
            pFactoryMtl->CreateDeviceAndContextsMtl(EngineCI, &m_pDevice, &m_pImmediateContext);
            pFactoryMtl->CreateSwapChainMtl(m_pDevice, m_pImmediateContext, SCDesc, Window, &m_pSwapChain);
        }
        break;
#endif // METAL_SUPPORTED

        default:
            std::cerr << "Unknown/unsupported device type";
            return false;
            break;
    }

    if (m_pDevice == nullptr || m_pImmediateContext == nullptr || m_pSwapChain == nullptr)
        return false;

    // Initialize Logger
    OS::Initialize();

    return true;
}

void Application::GLFW_ResizeCallback(GLFWwindow* wnd, int w, int h)
{
    auto* pSelf = static_cast<Application*>(glfwGetWindowUserPointer(wnd));
    if (pSelf->m_pSwapChain != nullptr)
        pSelf->m_pSwapChain->Resize(static_cast<Uint32>(w), static_cast<Uint32>(h));
}

void Application::GLFW_KeyCallback(GLFWwindow* wnd, int key, int, int state, int)
{
    auto* pSelf = static_cast<Application*>(glfwGetWindowUserPointer(wnd));
    pSelf->OnKeyEvent(static_cast<Key>(key), static_cast<KeyState>(state));
}

void Application::GLFW_MouseButtonCallback(GLFWwindow* wnd, int button, int state, int)
{
    auto* pSelf = static_cast<Application*>(glfwGetWindowUserPointer(wnd));
    pSelf->OnKeyEvent(static_cast<Key>(button), static_cast<KeyState>(state));
}

void Application::GLFW_CursorPosCallback(GLFWwindow* wnd, double xpos, double ypos)
{
    float xscale = 1;
    float yscale = 1;
    glfwGetWindowContentScale(wnd, &xscale, &yscale);
    auto* pSelf = static_cast<Application*>(glfwGetWindowUserPointer(wnd));
    pSelf->MouseEvent(float2(static_cast<float>(xpos * xscale), static_cast<float>(ypos * yscale)));
}

void Application::GLFW_MouseWheelCallback(GLFWwindow* wnd, double dx, double dy)
{
}

float2 Application::GetMousePosition()
{
    auto*  window = static_cast<GLFWwindow*>(m_Window);
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    return float2((float)xpos, (float)ypos);
}

float Application::GetMouseX()
{
    return GetMousePosition().x;
}

float Application::GetMouseY()
{
    return GetMousePosition().y;
}

void Application::Start()
{
}

void Application::Run()
{
    Start();

    m_LastUpdate = TClock::now();
    for (;;)
    {
        if (glfwWindowShouldClose(m_Window))
            return;

        glfwPollEvents();

        for (auto KeyIter = m_ActiveKeys.begin(); KeyIter != m_ActiveKeys.end();)
        {
            KeyEvent(KeyIter->key, KeyIter->state);

            // GLFW does not send 'Repeat' state again, we have to keep these keys until the 'Release' is received.
            switch (KeyIter->state)
            {
                // clang-format off
                case KeyState::Release: KeyIter = m_ActiveKeys.erase(KeyIter); break;
                case KeyState::Press:   KeyIter->state = KeyState::Repeat;     break;
                case KeyState::Repeat:  ++KeyIter;                             break;
                // clang-format on
                default:
                    break;
            }
        }

        const auto time = TClock::now();
        const auto dt   = std::chrono::duration_cast<TSeconds>(time - m_LastUpdate).count();
        m_LastUpdate    = time;

        PreUpdate();

        FixedUpdate(1.f / 60.f);

        Update(dt);

        PostUpdate();

        int w, h;
        glfwGetWindowSize(m_Window, &w, &h);

        // Skip rendering if window is minimized or too small
        if (w > 0 && h > 0)
        {
            if (!m_pSwapChain)
                return;

            auto* pCtx = GetContext();
            auto* pRTV = m_pSwapChain->GetCurrentBackBufferRTV();
            auto* pDSV = m_pSwapChain->GetDepthBufferDSV();
            pCtx->SetRenderTargets(1, &pRTV, pDSV, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

            Render();

            // Restore default render target in case the sample has changed it
            pCtx->SetRenderTargets(1, &pRTV, pDSV, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

            m_pSwapChain->Present();
        }
    }
}

bool Application::GetKeyEvent(Key key, KeyState state)
{
    for (auto& active : m_ActiveKeys)
    {
        if (active.key == key && active.state == state)
        {
            return true;
        }
    }

    return false;
}

bool Application::GetKeyEvent(Key key)
{
    for (auto& active : m_ActiveKeys)
    {
        if (active.key == key)
        {
            if (active.state == KeyState::Press || active.state == KeyState::Repeat)
            {
                return true;
            }
        }
    }

    return false;
}


void Application::OnKeyEvent(Key key, KeyState newState)
{
    for (auto& active : m_ActiveKeys)
    {
        if (active.key == key)
        {
            if (newState == KeyState::Release)
                active.state = newState;

            return;
        }
    }

    m_ActiveKeys.push_back({key, newState});
}


void Application::Quit()
{
    VERIFY_EXPR(m_Window != nullptr);
    glfwSetWindowShouldClose(m_Window, GLFW_TRUE);
}

bool Application::ProcessCommandLine(const char* CmdLine, RENDER_DEVICE_TYPE& DevType)
{
    // Choose Specified Render Device
    if (DevType != RENDER_DEVICE_TYPE_UNDEFINED)
    {
        return true;
    }

#if PLATFORM_LINUX || PLATFORM_MACOS
#    define _stricmp strcasecmp
#endif

    const auto* Key = "-mode ";
    const auto* pos = strstr(CmdLine, Key);
    if (pos != nullptr)
    {
        pos += strlen(Key);
        if (_stricmp(pos, "D3D11") == 0)
        {
#if D3D11_SUPPORTED
            DevType = RENDER_DEVICE_TYPE_D3D11;
#else
            std::cerr << "Direct3D11 is not supported. Please select another device type";
            return false;
#endif
        }
        else if (_stricmp(pos, "D3D12") == 0)
        {
#if D3D12_SUPPORTED
            DevType = RENDER_DEVICE_TYPE_D3D12;
#else
            std::cerr << "Direct3D12 is not supported. Please select another device type";
            return false;
#endif
        }
        else if (_stricmp(pos, "GL") == 0)
        {
#if GL_SUPPORTED
            DevType = RENDER_DEVICE_TYPE_GL;
#else
            std::cerr << "OpenGL is not supported. Please select another device type";
            return false;
#endif
        }
        else if (_stricmp(pos, "VK") == 0)
        {
#if VULKAN_SUPPORTED
            DevType = RENDER_DEVICE_TYPE_VULKAN;
#else
            std::cerr << "Vulkan is not supported. Please select another device type";
            return false;
#endif
        }
        else if (_stricmp(pos, "MTL") == 0)
        {
#if METAL_SUPPORTED
            DevType = RENDER_DEVICE_TYPE_METAL;
#else
            std::cerr << "Metal is not supported. Please select another device type";
            return false;
#endif
        }
        else
        {
            std::cerr << "Unknown device type. Only the following types are supported: D3D11, D3D12, GL, VK";
            return false;
        }
    }
    else
    {
#if METAL_SUPPORTED
        DevType = RENDER_DEVICE_TYPE_METAL;
#elif VULKAN_SUPPORTED
        DevType = RENDER_DEVICE_TYPE_VULKAN;
#elif D3D12_SUPPORTED
        DevType = RENDER_DEVICE_TYPE_D3D12;
#elif D3D11_SUPPORTED
        DevType = RENDER_DEVICE_TYPE_D3D11;
#elif GL_SUPPORTED
        DevType = RENDER_DEVICE_TYPE_GL;
#endif
    }
    return true;
}

int ApplicationMain(const char* cmdLine)
{
    std::unique_ptr<Application> Samp{CreateGLFWApp()};

    RENDER_DEVICE_TYPE DevType = RENDER_DEVICE_TYPE_D3D12;
    if (!Samp->ProcessCommandLine(cmdLine, DevType))
        return -1;


    String Title = Samp->GetSampleName() == "" ? "Vision Engine - Copyright (c) 2021 Theophilus Eriata" : Samp->GetSampleName();
    switch (DevType)
    {
        case RENDER_DEVICE_TYPE_D3D11: Title.append(" (D3D11"); break;
        case RENDER_DEVICE_TYPE_D3D12: Title.append(" (D3D12"); break;
        case RENDER_DEVICE_TYPE_GL: Title.append(" (GL"); break;
        case RENDER_DEVICE_TYPE_VULKAN: Title.append(" (VK"); break;
        case RENDER_DEVICE_TYPE_METAL: Title.append(" (Metal"); break;
        default:
            UNEXPECTED("Unexpected device type");
    }
    //Title.append(", API ");
    //Title.append(std::to_string(DILIGENT_API_VERSION));
    Title.push_back(')');

    int APIHint = GLFW_NO_API;
#if !PLATFORM_WIN32
    if (DevType == RENDER_DEVICE_TYPE_GL)
    {
        // On platforms other than Windows Diligent Engine
        // attaches to existing OpenGL context
        APIHint = GLFW_OPENGL_API;
    }
#endif

    if (!Samp->CreateWindow(Title.c_str(), 1044, 602, APIHint))
        return -1;

    if (!Samp->InitEngine(DevType))
        return -1;

    if (!Samp->Initialize())
        return -1;

    Samp->Run();

    return 0;
}
} // namespace Vision


int main(int argc, const char** argv)
{
    return Vision::ApplicationMain(argc >= 2 ? argv[1] : "");
}
