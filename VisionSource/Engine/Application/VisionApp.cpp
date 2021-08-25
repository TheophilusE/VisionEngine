
#include "VisionApp.h"

#include "imgui.h"

namespace Diligent
{
SampleBase* CreateSample()
{
    return new Vision::VisionApp();
}
} // namespace Diligent


namespace Vision
{

VisionApp::~VisionApp()
{
}

void VisionApp::Initialize(const SampleInitInfo& InitInfo)
{
    ApplicationBase::Initialize(InitInfo);

    InputDescription jump;
    jump.device     = Device::Keyboard;
    jump.button     = 32;
    jump.inputEvent = "Jump";

    m_InputScheme.bools.push_back(jump);

    m_InputMap.insert(std::pair<String, InputScheme>("DefaultInputScheme", m_InputScheme));

    InputSystem::GetSingleton()->loadSchemes(m_InputMap);
    InputSystem::GetSingleton()->pushScheme("DefaultInputScheme");

    InputManager::GetSingleton()->setEnabled(true);

    auto BackBufferFmt  = m_pSwapChain->GetDesc().ColorBufferFormat;
    auto DepthBufferFmt = m_pSwapChain->GetDesc().DepthBufferFormat;

    GLTF_PBR_Renderer::CreateInfo RendererCI;
    RendererCI.RTVFmt          = BackBufferFmt;
    RendererCI.DSVFmt          = DepthBufferFmt;
    RendererCI.AllowDebugView  = true;
    RendererCI.UseIBL          = true;
    RendererCI.FrontCCW        = true;
    RendererCI.UseTextureAtlas = false; // Use Resource Cache
    m_Renderer.reset(new Renderer(m_pDevice, m_pImmediateContext, RendererCI));

    m_Renderer->pEngineFactory = m_pEngineFactory;
    m_Renderer->pDevice        = m_pDevice;
    m_Renderer->pContext       = m_pImmediateContext;
    m_Renderer->pSwapChain     = m_pSwapChain;
    m_Renderer->Initialize();

    m_Camera = m_Scene->CreateEntity("Camera Component");
    m_DirectionalLight = m_Scene->CreateEntity("Directional Light Component");

    m_DirectionalLight.AddComponent<DirectionalLightComponent>();
    m_Camera.AddComponent<CameraComponent>();
}

void VisionApp::UpdateUI()
{
    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }
}

void VisionApp::PreUpdate() { ApplicationBase::PreUpdate(); }
void VisionApp::FixedUpdate() { ApplicationBase::FixedUpdate(); }
void VisionApp::PostUpdate() { ApplicationBase::PostUpdate(); }

void VisionApp::Update(double CurrTime, double ElapsedTime)
{
    ApplicationBase::Update(CurrTime, ElapsedTime);
    UpdateUI();

    //InputManager::GetSingleton()->getMouse()->GetState().

    if (InputManager::GetSingleton()->isPressed("Jump"))
    {
        //VISION_INFO("Jump Pressed!");
    }
}

void VisionApp::Render()
{
    m_Renderer->Render();
}

void VisionApp::PostRender()
{
}

void VisionApp::WindowResize(Uint32 Width, Uint32 Height)
{
    // Call atmo resize here

    ApplicationBase::WindowResize(Width, Height);
}
} // namespace Vision