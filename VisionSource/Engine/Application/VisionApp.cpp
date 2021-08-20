
#include "VisionApp.h"

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
}

void VisionApp::UpdateUI()
{
}

void VisionApp::PreUpdate() { ApplicationBase::PreUpdate(); }
void VisionApp::FixedUpdate() { ApplicationBase::FixedUpdate(); }
void VisionApp::PostUpdate() { ApplicationBase::PostUpdate(); }

void VisionApp::Update(double CurrTime, double ElapsedTime)
{
    ApplicationBase::Update(CurrTime, ElapsedTime);
    UpdateUI();
}

void VisionApp::Render()
{
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