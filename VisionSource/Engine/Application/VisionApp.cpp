
#include "VisionApp.h"

#include "imgui.h"
#include "imGuIZMO.h"
#include "ImGuiUtils.hpp"
#include <math.h>

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

static void DrawVectorControl(const String& Label, float3& fVector, float resetValue = 0.0f, float ColumnWidth = 100.0f);
static void DrawVectorControl(const String& Label, float4& fVector, float resetValue = 0.0f, float ColumnWidth = 100.0f);

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

    auto& light = m_DirectionalLight.AddComponent<DirectionalLightComponent>();
    m_Camera.AddComponent<CameraComponent>();
    m_Helmet.AddComponent<MeshComponent>();

    light.m_LightAttribs.ShadowAttribs.iNumCascades     = 4;
    light.m_LightAttribs.ShadowAttribs.fFixedDepthBias  = 0.0025f;
    light.m_LightAttribs.ShadowAttribs.iFixedFilterSize = 5;
    light.m_LightAttribs.ShadowAttribs.fFilterWorldSize = 0.1f;

    light.m_LightAttribs.f4Direction    = float3(-0.522699475f, -0.481321275f, -0.703671455f);
    light.m_LightAttribs.f4Intensity    = float4(1, 0.8f, 0.5f, 1);
    light.m_LightAttribs.f4AmbientLight = float4(0.125f, 0.125f, 0.125f, 1);

    /*
    VISION_CORE_INFO("CPS");
    m_Renderer.CreatePipelineStates();
    VISION_CORE_INFO("CPSEnd");
    VISION_CORE_INFO("CS");
    m_Renderer.CreateShadowMap();
    VISION_CORE_INFO("CSEnd");
    */
    //auto& m = m_Helmet.GetComponent<MeshComponent>();
    //m.LoadModel("models/DamagedHelmet/DamagedHelmet.gltf", m_Renderer);
    //auto& t = m_Helmet.GetComponent<TransformComponent>();
    //t.Rotation = t.Rotation.RotationFromAxisAngle(float3{0.f, 0.f, 1.f}, 120.f) * t.Rotation;

    //VISION_CORE_INFO(PhysicsEngineTestBullet2());
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
        auto& light  = m_DirectionalLight.GetComponent<DirectionalLightComponent>();
        auto& m      = m_Helmet.GetComponent<MeshComponent>();
        auto& t      = m_Helmet.GetComponent<TransformComponent>();
        auto& camera = m_Camera.GetComponent<CameraComponent>();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        if (ImGui::Begin("Settings", nullptr)) // ImGuiWindowFlags_AlwaysAutoResize
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
                    //m_Renderer.CreatePipelineStates();
                    //m_Renderer.CreateShadowMap();
                }
            }
#endif
            {
                ImGui::gizmo3D("Model Rotation", t.Rotation, ImGui::GetTextLineHeight() * 10);
                ImGui::SameLine();
                ImGui::gizmo3D("Light direction", light.m_LightDirection, ImGui::GetTextLineHeight() * 10);
            }

            {
                if (ImGui::TreeNode("Transform"))
                {
                    DrawVectorControl("Translation", t.Translation);

                    float4 Rotation = t.Rotation.q;

                    DrawVectorControl("Rotation", Rotation);

                    DrawVectorControl("Scale", t.Scale, 1.0f);

                    if (ImGui::Button("Update Rotation"))
                    {
                        t.Rotation = Quaternion::RotationFromAxisAngle(float3{1.f, 0.f, 0.f}, Rotation.x) *
                            Quaternion::RotationFromAxisAngle(float3{0.f, 1.f, 0.f}, Rotation.y) *
                            Quaternion::RotationFromAxisAngle(float3{0.f, 0.f, 1.f}, Rotation.z) *
                            t.Rotation *
                            Quaternion::RotationFromAxisAngle(float3{0.75f, 0.0f, 0.75f}, PI_F);
                    }

                    ImGui::TreePop();
                }
            }

            {
                if (ImGui::Button("Reset view"))
                {
                    camera.m_Camera.SetPos(float3{});
                    camera.m_Camera.SetRotation(0.f, 0.f);
                    t.Translation = float3{};
                    t.Rotation    = Quaternion::RotationFromAxisAngle(float3{0.f, 1.0f, 0.0f}, -PI_F / 2.f);
                }

                if (ImGui::TreeNode("Camera Settings"))
                {
                    /*
                        Link - https://dopeguides.com/field-of-view-calculator-camera/#:~:text=%20Field%20of%20View%20%3D%202%20%28Tan%20%28Angle,of%20view%20has%20never%20been%20easier%20than%20this.
                        Link - https://www.studiobinder.com/blog/camera-sensor-size/#:~:text=Sensor%20size%20chart%201%20Full%20Frame%2036mm%20by,8mm%20sensor%20with%20a%202.7x%20crop%20factor.%20
                        Field angle of view = 2 x arctan ((sensor dimension eg: width/(2x focal length)) * (180/Π)
                        Field of View = 2 (Tan (Angle of view/2) X linear distance to the object being captured)

                        Camera Dimensions: 36mm by 24mm.
                        Camera Focal Length: 10mm to 500mm.

                        Lenses
                        - 36 by 24
                        - 27.9 by 18.6
                        - 23.6 by 15.67
                        - 22.2 by 14.8
                        - 18.7 by 14.0
                        - 17.3 by 13
                        - 12.8 by 9.6
                        - 10.67 by 8
                        - 8.8 by 6.6
                        - 7.6 by 5.7
                        - 6.17 by 4.55
                        - 4.54 by 3.42
                    */
                    float   nearClip        = camera.m_Camera.GetProjAttribs().NearClipPlane;
                    float   farClip         = camera.m_Camera.GetProjAttribs().FarClipPlane;
                    float2& sensorDimension = camera.sensorDimension; // mm
                    float&  focalLength     = camera.focalLength;     // mm
                    float&  linearDistance  = camera.linearDistance;
                    float   angleOfView     = atan(sensorDimension.x / (2.f * focalLength)) * atan(sensorDimension.y / (2.f * focalLength)) * (180.f / PI_F);
                    float   fieldOfView     = 2.f * (angleOfView / 2.f) * linearDistance;
                    Float32 fov             = fieldOfView;

                    ImGui::SliderFloat("Sensor Dimension Width", &sensorDimension.x, 0.00454f, 0.0364f);
                    ImGui::SliderFloat("Sensor Dimension Height", &sensorDimension.y, 0.00342f, 0.024f);
                    ImGui::SliderFloat("Camera Focal Length", &focalLength, 0.018f, 0.5f);
                    ImGui::SliderFloat("Camera Linear Distance", &linearDistance, 0.001f, 10.f);
                    ImGui::SliderFloat("Camera Near Clip", &nearClip, 0.01f, 10000.f);
                    ImGui::SliderFloat("Camera Far Clip", &farClip, 1.f, 10000.f);
                    ImGui::SliderFloat("Camera Speed", &camera.speed, 0.f, 1000.f);
                    ImGui::SliderFloat("Camera Speed Scale", &camera.speedScale, 0.f, 1000000.f);
                    ImGui::SliderFloat("Camera Rotation Scale", &camera.rotationScale, 0.001f, 1.f);

                    camera.m_Camera.SetProjAttribs(nearClip, farClip, camera.m_Camera.GetProjAttribs().AspectRatio, fov,
                                                   m_pSwapChain->GetDesc().PreTransform, m_pDevice->GetDeviceInfo().IsGLDevice());
                    camera.m_Camera.SetMoveSpeed(camera.speed);
                    camera.m_Camera.SetSpeedUpScales(camera.speedScale, camera.superSpeedScale);
                    camera.m_Camera.SetRotationSpeed(camera.rotationScale);

                    ImGui::TreePop();
                }
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
                        // clang-format on

                        if (m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_MODE_REINHARD_MOD ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_MODE_UNCHARTED2 ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_LOGARITHMIC ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_ADAPTIVE_LOG)
                        {
                            ImGui::SliderFloat("White Point", &m_Renderer.GetRenderSettings().fWhitePoint, 0.01f, 20.0f);
                        }

                        ImGui::Checkbox("Auto Exposure", &m_Renderer.GetRenderSettings().bAutoExposure);
                        if (m_Renderer.GetRenderSettings().bAutoExposure)
                            ImGui::Checkbox("Light Adaptation", &m_Renderer.GetRenderSettings().bLightAdaptation);

                        {
                            Array<const char*, 7> ToneMappingMode;
                            ToneMappingMode[TONE_MAPPING_MODE_EXP]          = "Exp";
                            ToneMappingMode[TONE_MAPPING_MODE_REINHARD]     = "Reinhard";
                            ToneMappingMode[TONE_MAPPING_MODE_REINHARD_MOD] = "Reinhard Mod";
                            ToneMappingMode[TONE_MAPPING_MODE_UNCHARTED2]   = "Uncharted 2";
                            ToneMappingMode[TONE_MAPPING_FILMIC_ALU]        = "Filmic ALU";
                            ToneMappingMode[TONE_MAPPING_LOGARITHMIC]       = "Logarithmic";
                            ToneMappingMode[TONE_MAPPING_ADAPTIVE_LOG]      = "Adaptive log";
                            ImGui::Combo("Tone Mapping Mode", &m_Renderer.GetRenderSettings().iToneMappingMode, ToneMappingMode.data(), static_cast<int>(ToneMappingMode.size()));
                        }

                        if (m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_MODE_EXP ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_MODE_REINHARD ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_MODE_REINHARD_MOD ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_LOGARITHMIC ||
                            m_Renderer.GetRenderSettings().iToneMappingMode == TONE_MAPPING_ADAPTIVE_LOG)
                        {
                            ImGui::SliderFloat("Luminance Saturation", &m_Renderer.GetRenderSettings().fLuminanceSaturation, 0.01f, 2.f);
                        }

                        ImGui::TreePop();
                    }

                    {
                        Array<const char*, static_cast<size_t>(Renderer::BackgroundMode::NumModes)> BackgroundModes;
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
                if (m.m_Model != nullptr)
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

    //auto& light  = m_DirectionalLight.GetComponent<DirectionalLightComponent>();
    //auto& camera = m_Camera.GetComponent<CameraComponent>();

    // Update Shadow Map
    /*
    {
        ShadowMapManager::DistributeCascadeInfo DistrInfo;
        DistrInfo.pCameraView   = &camera.m_Camera.GetViewMatrix();
        DistrInfo.pCameraProj   = &camera.m_Camera.GetProjMatrix();
        float3 f3LightDirection = float3(light.m_LightAttribs.f4Direction.x, light.m_LightAttribs.f4Direction.y, light.m_LightAttribs.f4Direction.z);
        DistrInfo.pLightDir     = &f3LightDirection;

        DistrInfo.fPartitioningFactor = m_Renderer.GetShadowSettings().PartitioningFactor;
        DistrInfo.SnapCascades        = m_Renderer.GetShadowSettings().SnapCascades;
        DistrInfo.EqualizeExtents     = m_Renderer.GetShadowSettings().EqualizeExtents;
        DistrInfo.StabilizeExtents    = m_Renderer.GetShadowSettings().StabilizeExtents;

        m_Renderer.GetShadowMapManager().DistributeCascades(DistrInfo, light.m_LightAttribs.ShadowAttribs);
    }
    */

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

static void DrawVectorControl(const String& Label, float3& fVector, float resetValue, float ColumnWidth)
{
    ImGui::PushID(Label.c_str());

    ImGui::Columns(2);
    ImGui::SetColumnWidth(0, ColumnWidth);
    ImGui::Text(Label.c_str());
    ImGui::NextColumn();

    ImGui::PushMultiItemsWidths(3, ImGui::CalcItemWidth());
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2{0, 0});

    float  lineHeight = GImGui->Font->FontSize + GImGui->Style.FramePadding.y * 2.0f;
    ImVec2 buttonSize = {lineHeight + 3.0f, lineHeight};

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.8f, 0.1f, 0.15f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.9f, 0.2f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.8f, 0.1f, 0.15f, 1.0f});
    if (ImGui::Button("X", buttonSize))
        fVector.x = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##X", &fVector.x, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.2f, 0.7f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.3f, 0.8f, 0.3f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.2f, 0.7f, 0.2f, 1.0f});
    if (ImGui::Button("Y", buttonSize))
        fVector.y = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##Y", &fVector.y, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.1f, 0.25f, 0.8f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.2f, 0.35f, 0.9f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.1f, 0.25f, 0.8f, 1.0f});
    if (ImGui::Button("Z", buttonSize))
        fVector.z = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##Z", &fVector.z, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();

    ImGui::PopStyleVar();

    ImGui::Columns(1);

    ImGui::PopID();
}

static void DrawVectorControl(const String& Label, float4& fVector, float resetValue, float ColumnWidth)
{
    ImGui::PushID(Label.c_str());

    ImGui::Columns(2);
    ImGui::SetColumnWidth(0, ColumnWidth);
    ImGui::Text(Label.c_str());
    ImGui::NextColumn();

    ImGui::PushMultiItemsWidths(3, ImGui::CalcItemWidth());
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2{0, 0});

    float  lineHeight = GImGui->Font->FontSize + GImGui->Style.FramePadding.y * 2.0f;
    ImVec2 buttonSize = {lineHeight + 3.0f, lineHeight};

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.8f, 0.1f, 0.15f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.9f, 0.2f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.8f, 0.1f, 0.15f, 1.0f});
    if (ImGui::Button("X", buttonSize))
        fVector.x = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##X", &fVector.x, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.2f, 0.7f, 0.2f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.3f, 0.8f, 0.3f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.2f, 0.7f, 0.2f, 1.0f});
    if (ImGui::Button("Y", buttonSize))
        fVector.y = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##Y", &fVector.y, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.1f, 0.25f, 0.8f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.2f, 0.35f, 0.9f, 1.0f});
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.1f, 0.25f, 0.8f, 1.0f});
    if (ImGui::Button("Z", buttonSize))
        fVector.z = resetValue;
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::DragFloat("##Z", &fVector.z, 0.1f, 0.0f, 0.0f, "%.2f");
    ImGui::PopItemWidth();

    ImGui::PopStyleVar();

    ImGui::Columns(1);

    ImGui::PopID();
}

void VisionApp::WindowResize(Uint32 Width, Uint32 Height)
{
    // Call atmo resize here

    ApplicationBase::WindowResize(Width, Height);
    m_Renderer.CreateMSAARenderTarget();

    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry  = scenes[i]->GetSceneRegistry();
        auto  viewCamera = pRegistry.view<CameraComponent>();

        // Update Camera
        for (auto entity : viewCamera)
        {
            auto& camera = viewCamera.get<CameraComponent>(entity);

            float AspectRatio = static_cast<float>(Width) / static_cast<float>(Height);
            camera.m_Camera.SetProjAttribs(camera.m_Camera.GetProjAttribs().NearClipPlane, camera.m_Camera.GetProjAttribs().FarClipPlane, AspectRatio, PI_F / 4.f,
                                           m_pSwapChain->GetDesc().PreTransform, m_pDevice->GetDeviceInfo().IsGLDevice());
        }
    }
}
} // namespace Vision