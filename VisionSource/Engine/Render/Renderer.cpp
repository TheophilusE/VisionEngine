
#include "Renderer.h"

#include "MapHelper.hpp"
#include "GraphicsUtilities.h"
#include "TextureUtilities.h"
#include "CommonlyUsedStates.h"
#include "ShaderMacroHelper.hpp"
#include "FileSystem.hpp"

#include "../Core/OS/OS.h"
#include "../FrameWork/Components.h"

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

    const auto& ColorFmtInfo                 = pDevice->GetTextureFormatInfoExt(pSwapChain->GetDesc().ColorBufferFormat);
    const auto& DepthFmtInfo                 = pDevice->GetTextureFormatInfoExt(DepthBufferFormat);
    m_RenderSettings.m_SupportedSampleCounts = ColorFmtInfo.SampleCounts & DepthFmtInfo.SampleCounts;
    if (m_RenderSettings.m_SupportedSampleCounts & 0x04)
    {
        m_RenderSettings.m_SampleCount = 4;
    }
    else if (m_RenderSettings.m_SupportedSampleCounts & 0x02)
    {
        m_RenderSettings.m_SampleCount = 2;
    }
    else
    {
        VISION_CORE_WARN(ColorFmtInfo.Name, " + ", DepthFmtInfo.Name, " pair does not allow multisampling on this device");
        m_RenderSettings.m_SampleCount = 1;
    }
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

void Renderer::CreateMSAARenderTarget()
{
    if (m_RenderSettings.m_SampleCount == 1)
        return;

    const auto& SCDesc = pSwapChain->GetDesc();
    // Create window-size multi-sampled offscreen render target
    TextureDesc ColorDesc;
    ColorDesc.Name           = "Multisampled render target";
    ColorDesc.Type           = RESOURCE_DIM_TEX_2D;
    ColorDesc.BindFlags      = BIND_RENDER_TARGET;
    ColorDesc.Width          = SCDesc.Width;
    ColorDesc.Height         = SCDesc.Height;
    ColorDesc.MipLevels      = 1;
    ColorDesc.Format         = SCDesc.ColorBufferFormat;
    bool NeedsSRGBConversion = pDevice->GetDeviceInfo().IsD3DDevice() && (ColorDesc.Format == TEX_FORMAT_RGBA8_UNORM_SRGB || ColorDesc.Format == TEX_FORMAT_BGRA8_UNORM_SRGB);
    if (NeedsSRGBConversion)
    {
        // Internally Direct3D swap chain images are not SRGB, and ResolveSubresource
        // requires source and destination formats to match exactly or be typeless.
        // So we will have to create a typeless texture and use SRGB render target view with it.
        ColorDesc.Format = ColorDesc.Format == TEX_FORMAT_RGBA8_UNORM_SRGB ? TEX_FORMAT_RGBA8_TYPELESS : TEX_FORMAT_BGRA8_TYPELESS;
    }

    // Set the desired number of samples
    ColorDesc.SampleCount = m_RenderSettings.m_SampleCount;
    // Define optimal clear value
    ColorDesc.ClearValue.Format   = SCDesc.ColorBufferFormat;
    ColorDesc.ClearValue.Color[0] = 0.125f;
    ColorDesc.ClearValue.Color[1] = 0.125f;
    ColorDesc.ClearValue.Color[2] = 0.125f;
    ColorDesc.ClearValue.Color[3] = 1.f;
    RefCntAutoPtr<ITexture> pColor;
    pDevice->CreateTexture(ColorDesc, nullptr, &pColor);

    // Store the render target view
    m_pMSColorRTV.Release();
    if (NeedsSRGBConversion)
    {
        TextureViewDesc RTVDesc;
        RTVDesc.ViewType = TEXTURE_VIEW_RENDER_TARGET;
        RTVDesc.Format   = SCDesc.ColorBufferFormat;
        pColor->CreateView(RTVDesc, &m_pMSColorRTV);
    }
    else
    {
        m_pMSColorRTV = pColor->GetDefaultView(TEXTURE_VIEW_RENDER_TARGET);
    }


    // Create window-size multi-sampled depth buffer
    TextureDesc DepthDesc = ColorDesc;
    DepthDesc.Name        = "Multisampled depth buffer";
    DepthDesc.Format      = DepthBufferFormat;
    DepthDesc.BindFlags   = BIND_DEPTH_STENCIL;
    // Define optimal clear value
    DepthDesc.ClearValue.Format               = DepthDesc.Format;
    DepthDesc.ClearValue.DepthStencil.Depth   = 1;
    DepthDesc.ClearValue.DepthStencil.Stencil = 0;

    RefCntAutoPtr<ITexture> pDepth;
    pDevice->CreateTexture(DepthDesc, nullptr, &pDepth);
    // Store the depth-stencil view
    m_pMSDepthDSV = pDepth->GetDefaultView(TEXTURE_VIEW_DEPTH_STENCIL);
}

void Renderer::CreatePipelineStates()
{
    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry = scenes[i]->GetSceneRegistry();
        auto  viewLight = pRegistry.view<DirectionalLightComponent>();

        for (auto entity : viewLight)
        {
            auto& light = viewLight.get<DirectionalLightComponent>(entity);
            {
                ShaderCreateInfo                               ShaderCI;
                RefCntAutoPtr<IShaderSourceInputStreamFactory> pShaderSourceFactory;
                pEngineFactory->CreateDefaultShaderSourceStreamFactory("shaders", &pShaderSourceFactory);
                ShaderCI.pShaderSourceStreamFactory = pShaderSourceFactory;
                ShaderCI.SourceLanguage             = SHADER_SOURCE_LANGUAGE_HLSL;
                ShaderCI.UseCombinedTextureSamplers = true;

                ShaderMacroHelper Macros;
                // clang-format off
                Macros.AddShaderMacro( "SHADOW_MODE",            m_ShadowSettings.iShadowMode);
                Macros.AddShaderMacro( "SHADOW_FILTER_SIZE",     light.m_LightAttribs.ShadowAttribs.iFixedFilterSize);
                Macros.AddShaderMacro( "FILTER_ACROSS_CASCADES", m_ShadowSettings.FilterAcrossCascades);
                Macros.AddShaderMacro( "BEST_CASCADE_SEARCH",    m_ShadowSettings.SearchBestCascade );
                // clang-format on
                ShaderCI.Macros = Macros;

                ShaderCI.Desc.ShaderType = SHADER_TYPE_VERTEX;
                ShaderCI.Desc.Name       = "Mesh VS";
                ShaderCI.EntryPoint      = "MeshVS";
                ShaderCI.FilePath        = "MeshVS.vsh";
                RefCntAutoPtr<IShader> pVS;
                pDevice->CreateShader(ShaderCI, &pVS);

                ShaderCI.Desc.Name       = "Mesh PS";
                ShaderCI.EntryPoint      = "MeshPS";
                ShaderCI.FilePath        = "MeshPS.psh";
                ShaderCI.Desc.ShaderType = SHADER_TYPE_PIXEL;
                RefCntAutoPtr<IShader> pPS;
                pDevice->CreateShader(ShaderCI, &pPS);

                Macros.AddShaderMacro("SHADOW_PASS", true);
                ShaderCI.Desc.ShaderType = SHADER_TYPE_VERTEX;
                ShaderCI.Desc.Name       = "Mesh VS";
                ShaderCI.EntryPoint      = "MeshVS";
                ShaderCI.FilePath        = "MeshVS.vsh";
                ShaderCI.Macros          = Macros;
                RefCntAutoPtr<IShader> pShadowVS;
                pDevice->CreateShader(ShaderCI, &pShadowVS);
            }
        }
    }
}

void Renderer::InitializeResourceBindings()
{

}

void Renderer::CreateShadowMap()
{
    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry            = scenes[i]->GetSceneRegistry();
        auto  viewDirectionalLight = pRegistry.view<DirectionalLightComponent>();

        // Render Directional Light
        for (auto entity : viewDirectionalLight)
        {
            auto& light = viewDirectionalLight.get<DirectionalLightComponent>(entity);
            {
                if (m_ShadowSettings.Resolution >= 2048)
                    light.m_LightAttribs.ShadowAttribs.fFixedDepthBias = 0.0025f;
                else if (m_ShadowSettings.Resolution >= 1024)
                    light.m_LightAttribs.ShadowAttribs.fFixedDepthBias = 0.005f;
                else
                    light.m_LightAttribs.ShadowAttribs.fFixedDepthBias = 0.0075f;

                ShadowMapManager::InitInfo SMMgrInitInfo;
                SMMgrInitInfo.Format               = m_ShadowSettings.ShadowMapFormat;
                SMMgrInitInfo.Resolution           = m_ShadowSettings.Resolution;
                SMMgrInitInfo.NumCascades          = static_cast<Uint32>(light.m_LightAttribs.ShadowAttribs.iNumCascades);
                SMMgrInitInfo.ShadowMode           = m_ShadowSettings.iShadowMode;
                SMMgrInitInfo.Is32BitFilterableFmt = m_ShadowSettings.Is32BitFilterableFmt;

                if (!m_pComparisonSampler)
                {
                    SamplerDesc ComparsionSampler;
                    ComparsionSampler.ComparisonFunc = COMPARISON_FUNC_LESS;
                    // Note: anisotropic filtering requires SampleGrad to fix artifacts at
                    // cascade boundaries
                    ComparsionSampler.MinFilter = FILTER_TYPE_COMPARISON_LINEAR;
                    ComparsionSampler.MagFilter = FILTER_TYPE_COMPARISON_LINEAR;
                    ComparsionSampler.MipFilter = FILTER_TYPE_COMPARISON_LINEAR;
                    pDevice->CreateSampler(ComparsionSampler, &m_pComparisonSampler);
                }
                SMMgrInitInfo.pComparisonSampler = m_pComparisonSampler;

                if (!m_pFilterableShadowMapSampler)
                {
                    SamplerDesc SamplerDesc;
                    SamplerDesc.MinFilter     = FILTER_TYPE_ANISOTROPIC;
                    SamplerDesc.MagFilter     = FILTER_TYPE_ANISOTROPIC;
                    SamplerDesc.MipFilter     = FILTER_TYPE_ANISOTROPIC;
                    SamplerDesc.MaxAnisotropy = light.m_LightAttribs.ShadowAttribs.iMaxAnisotropy;
                    pDevice->CreateSampler(SamplerDesc, &m_pFilterableShadowMapSampler);
                }

                SMMgrInitInfo.pFilterableShadowMapSampler = m_pFilterableShadowMapSampler;
                m_ShadowMapMgr.Initialize(pDevice, SMMgrInitInfo);
                InitializeResourceBindings();
            }
        }
    }
}

void Renderer::RenderShadowMap()
{
    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry    = scenes[i]->GetSceneRegistry();
        auto  viewCamera   = pRegistry.view<CameraComponent>();
        auto  viewDirLight = pRegistry.view<DirectionalLightComponent>();

        for (auto entityLight : viewDirLight)
        {
            auto& light = viewDirLight.get<DirectionalLightComponent>(entityLight);

            auto iNumShadowCascades = light.m_LightAttribs.ShadowAttribs.iNumCascades;
            for (int iCascade = 0; iCascade < iNumShadowCascades; ++iCascade)
            {
                const auto    CascadeProjMatr           = m_ShadowMapMgr.GetCascadeTranform(iCascade).Proj; // Problem
                auto          WorldToLightViewSpaceMatr = light.m_LightAttribs.ShadowAttribs.mWorldToLightViewT.Transpose();
                auto          WorldToLightProjSpaceMatr = WorldToLightViewSpaceMatr * CascadeProjMatr;
                CameraAttribs ShadowCameraAttribs       = {};

                ShadowCameraAttribs.mViewT     = light.m_LightAttribs.ShadowAttribs.mWorldToLightViewT;
                ShadowCameraAttribs.mProjT     = CascadeProjMatr.Transpose();
                ShadowCameraAttribs.mViewProjT = WorldToLightProjSpaceMatr.Transpose();

                ShadowCameraAttribs.f4ViewportSize.x = static_cast<float>(m_ShadowSettings.Resolution);
                ShadowCameraAttribs.f4ViewportSize.y = static_cast<float>(m_ShadowSettings.Resolution);
                ShadowCameraAttribs.f4ViewportSize.z = 1.f / ShadowCameraAttribs.f4ViewportSize.x;
                ShadowCameraAttribs.f4ViewportSize.w = 1.f / ShadowCameraAttribs.f4ViewportSize.y;

                {
                    MapHelper<CameraAttribs> CameraData(pContext, m_CameraAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                    *CameraData = ShadowCameraAttribs;
                }

                auto* pCascadeDSV = m_ShadowMapMgr.GetCascadeDSV(iCascade);
                pContext->SetRenderTargets(0, nullptr, pCascadeDSV, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
                pContext->ClearDepthStencil(pCascadeDSV, CLEAR_DEPTH_FLAG, 1.f, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

                ViewFrustumExt Frutstum;
                ExtractViewFrustumPlanesFromMatrix(WorldToLightProjSpaceMatr, Frutstum, pDevice->GetDeviceInfo().IsGLDevice());
            }

            if (m_ShadowSettings.iShadowMode > SHADOW_MODE_PCF)
                m_ShadowMapMgr.ConvertToFilterable(pContext, light.m_LightAttribs.ShadowAttribs);
        }
    }
}

void Renderer::Render()
{
    RenderShadowMap();

    ITextureView* pRTV = nullptr;
    ITextureView* pDSV = nullptr;
    if (m_RenderSettings.m_SampleCount > 1 && m_RenderSettings.m_UseMSAA)
    {
        // Set off-screen multi-sampled render target and depth-stencil buffer
        pRTV = m_pMSColorRTV;
        pDSV = m_pMSDepthDSV;
    }
    else
    {
        // Render directly to the current swap chain back buffer.
        pRTV = pSwapChain->GetCurrentBackBufferRTV();
        pDSV = pSwapChain->GetDepthBufferDSV();
    }

    // Clear the back buffer
    const float ClearColor[] = {0.032f, 0.032f, 0.032f, 1.0f};
    pContext->SetRenderTargets(1, &pRTV, pDSV, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    pContext->ClearRenderTarget(pRTV, ClearColor, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    pContext->ClearDepthStencil(pDSV, CLEAR_DEPTH_FLAG, 1.0f, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

    // View Frustrum
    ViewFrustumExt Frutstum;

    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry            = scenes[i]->GetSceneRegistry();
        auto  viewDirectionalLight = pRegistry.view<DirectionalLightComponent>();
        auto  viewCamera           = pRegistry.view<CameraComponent>();
        auto  viewModel            = pRegistry.view<MeshComponent>();

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

        // Render Camera
        for (auto entity : viewCamera)
        {
            auto& camera = viewCamera.get<CameraComponent>(entity);
            if (camera.Active)
            {
                // Get pretransform matrix that rotates the scene according the surface orientation
                auto SrfPreTransform = GetSurfacePretransformMatrix(float3{0, 0, 1});

                const auto  CameraView     = camera.m_Camera.GetViewMatrix() * SrfPreTransform;
                const auto& CameraWorld    = camera.m_Camera.GetWorldMatrix();
                float3      CameraWorldPos = float3::MakeVector(CameraWorld[3]);
                const auto& Proj           = camera.m_Camera.GetProjMatrix();

                auto CameraViewProj = CameraView * Proj;

                {
                    MapHelper<CameraAttribs> CamAttribs(pContext, m_CameraAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                    CamAttribs->mProjT        = Proj.Transpose();
                    CamAttribs->mViewProjT    = CameraViewProj.Transpose();
                    CamAttribs->mViewProjInvT = CameraViewProj.Inverse().Transpose();
                    CamAttribs->f4Position    = float4(CameraWorldPos, 1);
                }

                ExtractViewFrustumPlanesFromMatrix(CameraViewProj, Frutstum, pDevice->GetDeviceInfo().IsGLDevice());
            }
        }

        // Render Models
        for (auto entity : viewModel)
        {
            auto& model = viewModel.get<MeshComponent>(entity);

            // Reset settings from the renderer
            model.m_RenderParams.AlphaModes        = m_RenderParams.AlphaModes;
            model.m_RenderParams.AverageLogLum     = m_RenderParams.AverageLogLum;
            model.m_RenderParams.DebugView         = m_RenderParams.DebugView;
            model.m_RenderParams.EmissionScale     = m_RenderParams.EmissionScale;
            model.m_RenderParams.IBLScale          = m_RenderParams.IBLScale;
            model.m_RenderParams.MiddleGray        = m_RenderParams.MiddleGray;
            model.m_RenderParams.OcclusionStrength = m_RenderParams.OcclusionStrength;
            model.m_RenderParams.WhitePoint        = m_RenderParams.WhitePoint;

            // Do check to make sure that the model pointer is valid
            if (model.m_Model != nullptr)
            {
                if (model.m_bUseResourceCache)
                {
                    m_GLTFRenderer->Begin(pDevice, pContext, model.m_CacheUseInfo, model.m_CacheBindings, m_CameraAttribsCB, m_LightAttribsCB);
                    m_GLTFRenderer->Render(pContext, *model.m_Model, model.m_RenderParams, nullptr, &model.m_CacheBindings);
                }
                else
                {
                    m_GLTFRenderer->Begin(pContext);
                    m_GLTFRenderer->Render(pContext, *model.m_Model, model.m_RenderParams, &model.m_ModelResourceBindings);
                }
            }
        }
    }

    if (m_BackgroundMode != BackgroundMode::None)
    {
        {
            MapHelper<EnvMapRenderAttribs> EnvMapAttribs(pContext, m_EnvMapRenderAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
            EnvMapAttribs->TMAttribs.iToneMappingMode     = TONE_MAPPING_MODE_UNCHARTED2;
            EnvMapAttribs->TMAttribs.bAutoExposure        = m_RenderSettings.bAutoExposure;
            EnvMapAttribs->TMAttribs.fMiddleGray          = m_RenderParams.MiddleGray;
            EnvMapAttribs->TMAttribs.bLightAdaptation     = m_RenderSettings.bLightAdaptation;
            EnvMapAttribs->TMAttribs.fWhitePoint          = m_RenderParams.WhitePoint;
            EnvMapAttribs->TMAttribs.fLuminanceSaturation = m_RenderSettings.fLuminanceSaturation;
            EnvMapAttribs->AverageLogLum                  = m_RenderParams.AverageLogLum;
            EnvMapAttribs->MipLevel                       = m_RenderSettings.m_EnvMapMipLevel;
        }
        pContext->SetPipelineState(m_EnvMapPSO);
        pContext->CommitShaderResources(m_EnvMapSRB, RESOURCE_STATE_TRANSITION_MODE_VERIFY);
        DrawAttribs drawAttribs(3, DRAW_FLAG_VERIFY_ALL);
        pContext->Draw(drawAttribs);
    }

    if (m_RenderSettings.m_SampleCount > 1 && m_RenderSettings.m_UseMSAA)
    {
        // Resolve multi-sampled render target into the current swap chain back buffer.
        auto pCurrentBackBuffer = pSwapChain->GetCurrentBackBufferRTV()->GetTexture();

        ResolveTextureSubresourceAttribs ResolveAttribs;
        ResolveAttribs.SrcTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        ResolveAttribs.DstTextureTransitionMode = RESOURCE_STATE_TRANSITION_MODE_TRANSITION;
        pContext->ResolveTextureSubresource(m_pMSColorRTV->GetTexture(), pCurrentBackBuffer, ResolveAttribs);
    }
}

GLTF_PBR_Renderer::ModelResourceBindings Renderer::CreateResourceBindings(GLTF::Model& GLTFModel,
                                                                          IBuffer*     pCameraAttribs,
                                                                          IBuffer*     pLightAttribs)
{
    return m_GLTFRenderer->CreateResourceBindings(GLTFModel, pCameraAttribs, pLightAttribs);
}

float4x4 Renderer::GetAdjustedProjectionMatrix(float FOV, float NearPlane, float FarPlane) const
{
    const auto& SCDesc = pSwapChain->GetDesc();

    float AspectRatio = static_cast<float>(SCDesc.Width) / static_cast<float>(SCDesc.Height);
    float XScale, YScale;
    if (SCDesc.PreTransform == SURFACE_TRANSFORM_ROTATE_90 ||
        SCDesc.PreTransform == SURFACE_TRANSFORM_ROTATE_270 ||
        SCDesc.PreTransform == SURFACE_TRANSFORM_HORIZONTAL_MIRROR_ROTATE_90 ||
        SCDesc.PreTransform == SURFACE_TRANSFORM_HORIZONTAL_MIRROR_ROTATE_270)
    {
        // When the screen is rotated, vertical FOV becomes horizontal FOV
        XScale = 1.f / std::tan(FOV / 2.f);
        // Aspect ratio is inversed
        YScale = XScale * AspectRatio;
    }
    else
    {
        YScale = 1.f / std::tan(FOV / 2.f);
        XScale = YScale / AspectRatio;
    }

    float4x4 Proj;
    Proj._11 = XScale;
    Proj._22 = YScale;
    Proj.SetNearFarClipPlanes(NearPlane, FarPlane, pDevice->GetDeviceInfo().IsGLDevice());
    return Proj;
}

float4x4 Renderer::GetSurfacePretransformMatrix(const float3& f3CameraViewAxis) const
{
    const auto& SCDesc = pSwapChain->GetDesc();
    switch (SCDesc.PreTransform)
    {
        case SURFACE_TRANSFORM_ROTATE_90:
            // The image content is rotated 90 degrees clockwise.
            return float4x4::RotationArbitrary(f3CameraViewAxis, -PI_F / 2.f);

        case SURFACE_TRANSFORM_ROTATE_180:
            // The image content is rotated 180 degrees clockwise.
            return float4x4::RotationArbitrary(f3CameraViewAxis, -PI_F);

        case SURFACE_TRANSFORM_ROTATE_270:
            // The image content is rotated 270 degrees clockwise.
            return float4x4::RotationArbitrary(f3CameraViewAxis, -PI_F * 3.f / 2.f);

        case SURFACE_TRANSFORM_OPTIMAL:
            UNEXPECTED("SURFACE_TRANSFORM_OPTIMAL is only valid as parameter during swap chain initialization.");
            return float4x4::Identity();

        case SURFACE_TRANSFORM_HORIZONTAL_MIRROR:
        case SURFACE_TRANSFORM_HORIZONTAL_MIRROR_ROTATE_90:
        case SURFACE_TRANSFORM_HORIZONTAL_MIRROR_ROTATE_180:
        case SURFACE_TRANSFORM_HORIZONTAL_MIRROR_ROTATE_270:
            UNEXPECTED("Mirror transforms are not supported");
            return float4x4::Identity();

        default:
            return float4x4::Identity();
    }
}

} // namespace Vision