
#include "ApplicationBase.h"

#include "windows.h"

namespace Vision
{

extern SampleBase* CreateSample()
{
    return new ApplicationBase();
}

ApplicationBase::ApplicationBase()
{}

ApplicationBase::~ApplicationBase()
{}

void ApplicationBase::ModifyEngineInitInfo(const ModifyEngineInitInfoAttribs& Attribs)
{
    SampleBase::ModifyEngineInitInfo(Attribs);

    Attribs.EngineCI.Features.ComputeShaders = DEVICE_FEATURE_STATE_ENABLED;
    Attribs.EngineCI.Features.DepthClamp     = DEVICE_FEATURE_STATE_OPTIONAL;

#if D3D12_SUPPORTED
    if (Attribs.DeviceType == RENDER_DEVICE_TYPE_D3D12)
    {
        auto& D3D12CI                           = static_cast<EngineD3D12CreateInfo&>(Attribs.EngineCI);
        D3D12CI.GPUDescriptorHeapSize[1]        = 1024; // Sampler descriptors
        D3D12CI.GPUDescriptorHeapDynamicSize[1] = 1024;
    }
#endif
}

void ApplicationBase::Initialize(const SampleInitInfo& InitInfo)
{
    SampleBase::Initialize(InitInfo);

    // Initialize Logger
    OS::Initialize();

    int Width = 1280;
    int Height = 1024;

    InputSystem::GetSingleton()->initialize(static_cast<unsigned int>(Width), static_cast<unsigned int>(Height));
}

void ApplicationBase::HandleWin32Proc(const void* MsgData)
{
    struct WindowMessageData
    {
        HWND   hWnd;
        UINT   message;
        WPARAM wParam;
        LPARAM lParam;
    };
    const WindowMessageData& WndMsg = *reinterpret_cast<const WindowMessageData*>(MsgData);

    auto hWnd   = WndMsg.hWnd;
    auto uMsg   = WndMsg.message;
    auto wParam = WndMsg.wParam;
    auto lParam = WndMsg.lParam;

    InputManager::GetSingleton()->forwardMessage({hWnd, uMsg, wParam, lParam });
}

void ApplicationBase::PreUpdate()
{
}

void ApplicationBase::FixedUpdate()
{
}

void ApplicationBase::Update(double CurrTime, double ElapsedTime)
{
    SampleBase::Update(CurrTime, ElapsedTime);
    InputSystem::GetSingleton()->Update(static_cast<float>(ElapsedTime));
}

void ApplicationBase::PostUpdate()
{
    EventManager::GetSingleton()->dispatchDeferred();
}

void ApplicationBase::Render()
{
}

void ApplicationBase::PostRender()
{
}

void ApplicationBase::WindowResize(Uint32 Width, Uint32 Height)
{
    // Flush is required because Intel driver does not release resources until
    // command buffer is flushed. When window is resized, WindowResize() is called for
    // every intermediate window size, and light scattering object creates resources
    // for the new size. This resources are then released by the light scattering object, but
    // not by Intel driver, which results in memory exhaustion.
    m_pImmediateContext->Flush();

    EventManager::GetSingleton()->call(VisionEvents::WindowResized, float2{static_cast<float>(Width), static_cast<float>(Height)});
}

} // namespace Vision