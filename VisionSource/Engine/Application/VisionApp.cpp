
#include "VisionApp.h"

#include "imgui.h"
#include "imGuIZMO.h"
#include "ImGuiUtils.hpp"

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

    m_Renderer.Initialize(m_pEngineFactory, m_pDevice, m_pImmediateContext, m_pSwapChain);

    m_Camera           = m_Scene->CreateEntity("Camera Component");
    m_DirectionalLight = m_Scene->CreateEntity("Directional Light Component");
    m_Helmet           = m_Scene->CreateEntity("Helmet");

    m_DirectionalLight.AddComponent<DirectionalLightComponent>();
    m_Camera.AddComponent<CameraComponent>();
    auto& m = m_Helmet.AddComponent<MeshComponent>();
    m.LoadModel("models/DamagedHelmet/DamagedHelmet.gltf", m_Renderer);
    //auto& t = m_Helmet.GetComponent<TransformComponent>();
    //t.Rotation = t.Rotation.RotationFromAxisAngle(float3{0.f, 0.f, 1.f}, 120.f) * t.Rotation;
}

void VisionApp::UpdateUI()
{
    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    {
        auto& light = m_DirectionalLight.GetComponent<DirectionalLightComponent>();
        auto& m     = m_Helmet.GetComponent<MeshComponent>();
        auto& t     = m_Helmet.GetComponent<TransformComponent>();
        auto& c     = m_Camera.GetComponent<CameraComponent>();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
#ifdef PLATFORM_WIN32
            if (ImGui::Button("Load model"))
            {
                FileDialogAttribs OpenDialogAttribs{FILE_DIALOG_TYPE_OPEN};
                OpenDialogAttribs.Title  = "Select GLTF file";
                OpenDialogAttribs.Filter = "glTF files\0*.gltf;*.glb\0";
                auto FileName            = FileSystem::FileDialog(OpenDialogAttribs);
                if (!FileName.empty())
                {
                    m.LoadModel(FileName.c_str(), m_Renderer);
                }
            }
#endif
            {
                ImGui::gizmo3D("Model Rotation", t.Rotation, ImGui::GetTextLineHeight() * 10);
                ImGui::SameLine();
                ImGui::gizmo3D("Light direction", light.m_LightDirection, ImGui::GetTextLineHeight() * 10);
            }

            {
                if (ImGui::TreeNode("Lighting"))
                {
                    ImGui::ColorEdit3("Light Color", &light.m_LightColor.r);
                    // clang-format off
                    ImGui::SliderFloat("Light Intensity", &light.m_LightIntensity, 0.f, 100.f);
                    //ImGui::SliderFloat("Occlusion strength", &m_RenderParams.OcclusionStrength, 0.f,  1.f);
                    //ImGui::SliderFloat("Emission scale",     &m_RenderParams.EmissionScale,     0.f,  1.f);
                    //ImGui::SliderFloat("IBL scale",          &m_RenderParams.IBLScale,          0.f,  1.f);
                    // clang-format on
                    ImGui::TreePop();
                }
            }

            {
                if (ImGui::Button("Reset view"))
                {
                    c.m_Camera.SetPos(float3{});
                    c.m_Camera.SetRotation(0.f, 0.f);
                    t.Translation = float3{};
                    t.Rotation    = Quaternion::RotationFromAxisAngle(float3{0.f, 1.0f, 0.0f}, -PI_F / 2.f);
                }
            }

            {
                if (!m.m_Model->Animations.empty())
                {
                    ImGui::SetNextTreeNodeOpen(true, ImGuiCond_FirstUseEver);
                    if (ImGui::TreeNode("Model Animation"))
                    {
                        ImGui::Checkbox("Play Animation", &m.m_PlayAnimation);
                        Vector<const char*> Animations(m.m_Model->Animations.size());
                        for (size_t i = 0; i < m.m_Model->Animations.size(); ++i)
                            Animations[i] = m.m_Model->Animations[i].Name.c_str();
                        ImGui::Combo("Current Animation", reinterpret_cast<int*>(&m.m_AnimationIndex), Animations.data(), static_cast<int>(Animations.size()));
                        ImGui::TreePop();
                    }
                }
            }
        }
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

    m_Scene->Update(m_InputController, m_Renderer.GetRenderDevice(), m_Renderer.GetSwapChain(), static_cast<float>(ElapsedTime));

    auto& t = m_Helmet.GetComponent<TransformComponent>();
    if (InputManager::IsPressed("Jump"))
    {
        t.Translation.y += 0.001f;

        VISION_CORE_INFO(t.Translation.y);
    }

    //t.Rotation = t.Rotation.RotationFromAxisAngle(float3{0.f, 1.f, 0.f}, 0.01f) * t.Rotation.RotationFromAxisAngle(float3{1.f, 0.f, 0.f}, 0.01f) * t.Rotation.RotationFromAxisAngle(float3{0.f, 0.f, 1.f}, 0.01f) * t.Rotation;
}

void VisionApp::Render()
{
    m_Renderer.Render();
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