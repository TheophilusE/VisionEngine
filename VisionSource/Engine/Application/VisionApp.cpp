
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

            if (ImGui::TreeNode("Environment Settings"))
            {
                {
                    if (ImGui::TreeNode("Environment Lighting"))
                    {
                        ImGui::ColorEdit3("Light Color", &light.m_LightColor.r);
                        // clang-format off
                        ImGui::SliderFloat("Light Intensity",    &light.m_LightIntensity,                        0.f, 100.f);
                        ImGui::SliderFloat("Occlusion strength", &m_Renderer.GetRenderParams().OcclusionStrength, 0.f,  1.f);
                        ImGui::SliderFloat("Emission scale",     &m_Renderer.GetRenderParams().EmissionScale,     0.f,  1.f);
                        ImGui::SliderFloat("IBL scale",          &m_Renderer.GetRenderParams().IBLScale,          0.f,  1.f);
                        ImGui::SliderFloat("Environment Map MIP Level", &m_Renderer.GetRenderSettings().m_EnvMapMipLevel, 0.0f, 7.0f);
                        // clang-format on
                        ImGui::TreePop();
                    }

                    if (ImGui::TreeNode("Tone mapping"))
                    {
                        // clang-format off
                        ImGui::SliderFloat("Average log lum",    &m_Renderer.GetRenderParams().AverageLogLum,     0.01f, 10.0f);
                        ImGui::SliderFloat("Middle gray",        &m_Renderer.GetRenderParams().MiddleGray,        0.01f,  1.0f);
                        ImGui::SliderFloat("White point",        &m_Renderer.GetRenderParams().WhitePoint,        0.1f,  20.0f);
                        // clang-format on
                        ImGui::TreePop();
                    }

                    {
                        std::array<const char*, static_cast<size_t>(Renderer::BackgroundMode::NumModes)> BackgroundModes;
                        BackgroundModes[static_cast<size_t>(Renderer::BackgroundMode::None)]              = "None";
                        BackgroundModes[static_cast<size_t>(Renderer::BackgroundMode::EnvironmentMap)]    = "Environment Map";
                        BackgroundModes[static_cast<size_t>(Renderer::BackgroundMode::Irradiance)]        = "Irradiance";
                        BackgroundModes[static_cast<size_t>(Renderer::BackgroundMode::PrefilteredEnvMap)] = "Prefiltered Environment Map";
                        if (ImGui::Combo("Background mode", reinterpret_cast<int*>(&m_Renderer.m_BackgroundMode), BackgroundModes.data(), static_cast<int>(BackgroundModes.size())))
                        {
                            m_Renderer.CreateEnvMapSRB();
                        }
                    }

                    {
                        Array<const char*, static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::NumDebugViews)> DebugViews;

                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::None)]            = "None";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::BaseColor)]       = "Base Color";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Transparency)]    = "Transparency";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::NormalMap)]       = "Normal Map";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Occlusion)]       = "Occlusion";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Emissive)]        = "Emissive";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Metallic)]        = "Metallic";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Roughness)]       = "Roughness";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::DiffuseColor)]    = "Diffuse color";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::SpecularColor)]   = "Specular color (R0)";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::Reflectance90)]   = "Reflectance90";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::MeshNormal)]      = "Mesh normal";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::PerturbedNormal)] = "Perturbed normal";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::NdotV)]           = "n*v";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::DiffuseIBL)]      = "Diffuse IBL";
                        DebugViews[static_cast<size_t>(GLTF_PBR_Renderer::RenderInfo::DebugViewType::SpecularIBL)]     = "Specular IBL";
                        ImGui::Combo("Debug view", reinterpret_cast<int*>(&m_Renderer.GetRenderParams().DebugView), DebugViews.data(), static_cast<int>(DebugViews.size()));
                    }
                }
                ImGui::TreePop();
            }

            {
                if (ImGui::TreeNode("Post Processing"))
                {
                    ImGui::Checkbox("Enable MSAA", &m_Renderer.GetRenderSettings().m_UseMSAA);

                    std::array<std::pair<Uint8, const char*>, 4> ComboItems;

                    Uint32 NumItems = 0;

                    ComboItems[NumItems++] = std::make_pair(Uint8{1}, "1");
                    if (m_Renderer.GetRenderSettings().m_SupportedSampleCounts & 0x02)
                        ComboItems[NumItems++] = std::make_pair(Uint8{2}, "2");
                    if (m_Renderer.GetRenderSettings().m_SupportedSampleCounts & 0x04)
                        ComboItems[NumItems++] = std::make_pair(Uint8{4}, "4");
                    if (m_Renderer.GetRenderSettings().m_SupportedSampleCounts & 0x08)
                        ComboItems[NumItems++] = std::make_pair(Uint8{8}, "8");
                    if (ImGui::Combo("MSAA Sample count", &m_Renderer.GetRenderSettings().m_SampleCount, ComboItems.data(), NumItems))
                    {
                        m_Renderer.CreateMSAARenderTarget();
                    }

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
    m_Renderer.CreateMSAARenderTarget();
}
} // namespace Vision