
#include "Renderer.h"

#include "MapHelper.hpp"
#include "GraphicsUtilities.h"
#include "TextureUtilities.h"
#include "CommonlyUsedStates.h"
#include "ShaderMacroHelper.hpp"
#include "FileSystem.hpp"

#include "../Core/OS/OS.h"

namespace Vision
{
#include "Shaders/Common/public/BasicStructures.fxh"
#include "Shaders/PostProcess/ToneMapping/public/ToneMappingStructures.fxh"

namespace
{
struct EnvMapRenderAttribs
{
    ToneMappingAttribs TMAttribs;

    float AverageLogLum;
    float MipLevel;
    float Unusued1;
    float Unusued2;
};
} // namespace

void Renderer::Initialize(IEngineFactory* pEF, IRenderDevice* pD, IDeviceContext* pC, ISwapChain* pS)
{
    pEngineFactory = pEF;
    pDevice        = pD;
    pContext       = pC;
    pSwapChain     = pS;

    if (!(pEngineFactory || pDevice || pContext || pSwapChain))
    {
        VISION_CORE_ERROR("Renderpath Initialization Failed");
        VISION_CORE_WARN("Failed To Initialize Renderer");
        LOG_FATAL_ERROR_AND_THROW("Renderer Initialization Returned Null");
    }

    RefCntAutoPtr<ITexture> EnvironmentMap;
    CreateTextureFromFile("textures/sky.dds", TextureLoadInfo{"Environment map"}, pDevice, &EnvironmentMap);
    m_EnvironmentMapSRV = EnvironmentMap->GetDefaultView(TEXTURE_VIEW_SHADER_RESOURCE);

    auto BackBufferFmt  = pSwapChain->GetDesc().ColorBufferFormat;
    auto DepthBufferFmt = pSwapChain->GetDesc().DepthBufferFormat;

    GLTF_PBR_Renderer::CreateInfo RendererCI;
    RendererCI.RTVFmt          = BackBufferFmt;
    RendererCI.DSVFmt          = DepthBufferFmt;
    RendererCI.AllowDebugView  = true;
    RendererCI.UseIBL          = true;
    RendererCI.FrontCCW        = true;
    RendererCI.UseTextureAtlas = m_bUseResourceCache;
    m_GLTFRenderer.reset(new GLTF_PBR_Renderer(pDevice, pContext, RendererCI));

    CreateUniformBuffer(pDevice, sizeof(CameraAttribs), "Camera attribs buffer", &m_CameraAttribsCB);
    CreateUniformBuffer(pDevice, sizeof(LightAttribs), "Light attribs buffer", &m_LightAttribsCB);
    CreateUniformBuffer(pDevice, sizeof(EnvMapRenderAttribs), "Env map render attribs buffer", &m_EnvMapRenderAttribsCB);

    // clang-format off
    StateTransitionDesc Barriers [] =
    {
        {m_CameraAttribsCB,        RESOURCE_STATE_UNKNOWN, RESOURCE_STATE_CONSTANT_BUFFER, STATE_TRANSITION_FLAG_UPDATE_STATE},
        {m_LightAttribsCB,         RESOURCE_STATE_UNKNOWN, RESOURCE_STATE_CONSTANT_BUFFER, STATE_TRANSITION_FLAG_UPDATE_STATE},
        {m_EnvMapRenderAttribsCB,  RESOURCE_STATE_UNKNOWN, RESOURCE_STATE_CONSTANT_BUFFER, STATE_TRANSITION_FLAG_UPDATE_STATE},
        {EnvironmentMap,           RESOURCE_STATE_UNKNOWN, RESOURCE_STATE_SHADER_RESOURCE, STATE_TRANSITION_FLAG_UPDATE_STATE}
    };
    // clang-format on
    pContext->TransitionResourceStates(_countof(Barriers), Barriers);

    m_GLTFRenderer->PrecomputeCubemaps(pDevice, pContext, m_EnvironmentMapSRV);

    CreateEnvMapPSO();

    // Set scene settings 
    m_BackgroundMode = BackgroundMode::EnvironmentMap;
}

void Renderer::CreateEnvMapPSO()
{
    ShaderCreateInfo                               ShaderCI;
    RefCntAutoPtr<IShaderSourceInputStreamFactory> pShaderSourceFactory;
    pEngineFactory->CreateDefaultShaderSourceStreamFactory("shaders", &pShaderSourceFactory);
    ShaderCI.pShaderSourceStreamFactory = pShaderSourceFactory;
    ShaderCI.SourceLanguage             = SHADER_SOURCE_LANGUAGE_HLSL;
    ShaderCI.UseCombinedTextureSamplers = true;

    ShaderMacroHelper Macros;
    Macros.AddShaderMacro("TONE_MAPPING_MODE", "TONE_MAPPING_MODE_UNCHARTED2");
    ShaderCI.Macros = Macros;

    ShaderCI.Desc.ShaderType = SHADER_TYPE_VERTEX;
    ShaderCI.Desc.Name       = "Environment map VS";
    ShaderCI.EntryPoint      = "main";
    ShaderCI.FilePath        = "env_map.vsh";
    RefCntAutoPtr<IShader> pVS;
    pDevice->CreateShader(ShaderCI, &pVS);

    ShaderCI.Desc.Name       = "Environment map PS";
    ShaderCI.EntryPoint      = "main";
    ShaderCI.FilePath        = "env_map.psh";
    ShaderCI.Desc.ShaderType = SHADER_TYPE_PIXEL;
    RefCntAutoPtr<IShader> pPS;
    pDevice->CreateShader(ShaderCI, &pPS);

    GraphicsPipelineStateCreateInfo PSOCreateInfo;
    PipelineStateDesc&              PSODesc          = PSOCreateInfo.PSODesc;
    GraphicsPipelineDesc&           GraphicsPipeline = PSOCreateInfo.GraphicsPipeline;

    PSODesc.ResourceLayout.DefaultVariableType = SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE;

    // clang-format off
    ImmutableSamplerDesc ImmutableSamplers[] =
    {
        {SHADER_TYPE_PIXEL, "EnvMap", Sam_LinearClamp}
    };
    // clang-format on
    PSODesc.ResourceLayout.ImmutableSamplers    = ImmutableSamplers;
    PSODesc.ResourceLayout.NumImmutableSamplers = _countof(ImmutableSamplers);

    // clang-format off
    ShaderResourceVariableDesc Vars[] = 
    {
        {SHADER_TYPE_PIXEL, "cbCameraAttribs",       SHADER_RESOURCE_VARIABLE_TYPE_STATIC},
        {SHADER_TYPE_PIXEL, "cbEnvMapRenderAttribs", SHADER_RESOURCE_VARIABLE_TYPE_STATIC}
    };
    // clang-format on
    PSODesc.ResourceLayout.Variables    = Vars;
    PSODesc.ResourceLayout.NumVariables = _countof(Vars);

    PSODesc.Name      = "EnvMap PSO";
    PSOCreateInfo.pVS = pVS;
    PSOCreateInfo.pPS = pPS;

    GraphicsPipeline.RTVFormats[0]              = pSwapChain->GetDesc().ColorBufferFormat;
    GraphicsPipeline.NumRenderTargets           = 1;
    GraphicsPipeline.DSVFormat                  = pSwapChain->GetDesc().DepthBufferFormat;
    GraphicsPipeline.PrimitiveTopology          = PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_LESS_EQUAL;

    pDevice->CreateGraphicsPipelineState(PSOCreateInfo, &m_EnvMapPSO);
    m_EnvMapPSO->GetStaticVariableByName(SHADER_TYPE_PIXEL, "cbCameraAttribs")->Set(m_CameraAttribsCB);
    m_EnvMapPSO->GetStaticVariableByName(SHADER_TYPE_PIXEL, "cbEnvMapRenderAttribs")->Set(m_EnvMapRenderAttribsCB);
    CreateEnvMapSRB();
}

void Renderer::CreateEnvMapSRB()
{
    if (m_BackgroundMode != BackgroundMode::None)
    {
        m_EnvMapSRB.Release();
        m_EnvMapPSO->CreateShaderResourceBinding(&m_EnvMapSRB, true);
        ITextureView* pEnvMapSRV = nullptr;
        switch (m_BackgroundMode)
        {
            case BackgroundMode::EnvironmentMap:
                pEnvMapSRV = m_EnvironmentMapSRV;
                break;

            case BackgroundMode::Irradiance:
                pEnvMapSRV = m_GLTFRenderer->GetIrradianceCubeSRV();
                break;

            case BackgroundMode::PrefilteredEnvMap:
                pEnvMapSRV = m_GLTFRenderer->GetPrefilteredEnvMapSRV();
                break;

            default:
                UNEXPECTED("Unexpected background mode");
        }
        m_EnvMapSRB->GetVariableByName(SHADER_TYPE_PIXEL, "EnvMap")->Set(pEnvMapSRV);
    }
}

void Renderer::Render()
{
    auto* pRTV = pSwapChain->GetCurrentBackBufferRTV();
    auto* pDSV = pSwapChain->GetDepthBufferDSV();
    // Clear the back buffer
    const float ClearColor[] = {0.032f, 0.032f, 0.032f, 1.0f};
    pContext->ClearRenderTarget(pRTV, ClearColor, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    pContext->ClearDepthStencil(pDSV, CLEAR_DEPTH_FLAG, 1.f, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry            = scenes[i]->GetSceneRegistry();
        auto  viewDirectionalLight = pRegistry.view<DirectionalLightComponent>();
        auto  viewCamera           = pRegistry.view<CameraComponent>();

        // Render Camera
        for (auto entity : viewCamera)
        {
            auto& camera = viewCamera.get<CameraComponent>(entity);
            if (camera.Active)
            {
                MapHelper<CameraAttribs> CamAttribs(pContext, m_CameraAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                CamAttribs->mProjT        = camera.m_Camera.GetProjMatrix().Transpose();
                CamAttribs->mViewProjT    = camera.m_Camera.GetViewMatrix().Transpose();
                CamAttribs->mViewProjInvT = camera.m_Camera.GetViewMatrix().Inverse().Transpose();
                CamAttribs->f4Position    = float4(camera.m_Camera.GetPos(), 1);
            }
        }

        // Render Directional Light
        for (auto entity : viewDirectionalLight)
        {
            // Ex. auto &vel = view.get<pos, velocity>(entity);
            auto& dirLight = viewDirectionalLight.get<DirectionalLightComponent>(entity);
            {
                MapHelper<LightAttribs> lightAttribs(pContext, m_LightAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                lightAttribs->f4Direction = dirLight.m_LightDirection;
                lightAttribs->f4Intensity = dirLight.GetIntensity();
            }
        }
    }

    if (m_BackgroundMode != BackgroundMode::None)
    {
        {
            MapHelper<EnvMapRenderAttribs> EnvMapAttribs(pContext, m_EnvMapRenderAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
            EnvMapAttribs->TMAttribs.iToneMappingMode     = TONE_MAPPING_MODE_UNCHARTED2;
            EnvMapAttribs->TMAttribs.bAutoExposure        = 0;
            EnvMapAttribs->TMAttribs.fMiddleGray          = m_RenderParams.MiddleGray;
            EnvMapAttribs->TMAttribs.bLightAdaptation     = 0;
            EnvMapAttribs->TMAttribs.fWhitePoint          = m_RenderParams.WhitePoint;
            EnvMapAttribs->TMAttribs.fLuminanceSaturation = 1.0;
            EnvMapAttribs->AverageLogLum                  = m_RenderParams.AverageLogLum;
            EnvMapAttribs->MipLevel                       = m_RenderSettings.m_EnvMapMipLevel;
        }
        pContext->SetPipelineState(m_EnvMapPSO);
        pContext->CommitShaderResources(m_EnvMapSRB, RESOURCE_STATE_TRANSITION_MODE_VERIFY);
        DrawAttribs drawAttribs(3, DRAW_FLAG_VERIFY_ALL);
        pContext->Draw(drawAttribs);
    }
}
} // namespace Vision