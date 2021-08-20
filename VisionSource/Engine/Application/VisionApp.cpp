
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

    
    if (InputManager::GetSingleton()->isPressed("Jump"))
    {
        //VISION_INFO("Jump Pressed!");
    }
    
}

void VisionApp::Render()
{
    auto* pRTV = m_pSwapChain->GetCurrentBackBufferRTV();
    m_pImmediateContext->ClearRenderTarget(pRTV, &m_ClearColor.x, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
}

void VisionApp::PostRender()
{
}

void VisionApp::WindowResize(Uint32 Width, Uint32 Height)
{
    // Call atmo resize here

    // Flush is required because Intel driver does not release resources until
    // command buffer is flushed. When window is resized, WindowResize() is called for
    // every intermediate window size, and light scattering object creates resources
    // for the new size. This resources are then released by the light scattering object, but
    // not by Intel driver, which results in memory exhaustion.
    m_pImmediateContext->Flush();
}
} // namespace Vision