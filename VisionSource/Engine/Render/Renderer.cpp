
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

void Renderer::Render()
{
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

    bool m_bIsGLDevice = pDevice->GetDeviceInfo().IsGLDevice();
    // Clear the back buffer
    const float ClearColor[] = {0.032f, 0.032f, 0.032f, 1.0f};
    pContext->SetRenderTargets(1, &pRTV, pDSV, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    pContext->ClearRenderTarget(pRTV, ClearColor, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);
    pContext->ClearDepthStencil(pDSV, CLEAR_DEPTH_FLAG, 1.0f, 0, RESOURCE_STATE_TRANSITION_MODE_TRANSITION);

    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry            = scenes[i]->GetSceneRegistry();
        auto  viewDirectionalLight = pRegistry.view<DirectionalLightComponent>();
        auto  viewCamera           = pRegistry.view<CameraComponent>();
        auto  viewModel            = pRegistry.view<MeshComponent>();

        // Render Camera
        for (auto entity : viewCamera)
        {
            auto& camera = viewCamera.get<CameraComponent>(entity);
            if (camera.Active)
            {
                //MapHelper<CameraAttribs> CamAttribs(pContext, m_CameraAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                //CamAttribs->mProjT        = camera.m_Camera.GetProjMatrix().Transpose();
                //CamAttribs->mViewProjT    = camera.m_Camera.GetViewMatrix().Transpose();
                //CamAttribs->mViewProjInvT = camera.m_Camera.GetViewMatrix().Inverse().Transpose();
                //CamAttribs->f4Position    = float4(camera.m_Camera.GetPos(), 1);

                float4x4 mViewProj = camera.m_Camera.GetViewMatrix() * camera.m_Camera.GetProjMatrix();

                CameraAttribs CameraAttribsB;
                CameraAttribsB.mViewT        = camera.m_Camera.GetViewMatrix().Transpose();
                CameraAttribsB.mProjT        = camera.m_Camera.GetProjMatrix().Transpose();
                CameraAttribsB.mViewProjT    = mViewProj.Transpose();
                CameraAttribsB.mViewProjInvT = mViewProj.Inverse().Transpose();
                float fNearPlane = 0.f, fFarPlane = 0.f;
                camera.m_Camera.GetProjMatrix().GetNearFarClipPlanes(fNearPlane, fFarPlane, m_bIsGLDevice);
                CameraAttribsB.fNearPlaneZ      = fNearPlane;
                CameraAttribsB.fFarPlaneZ       = fFarPlane * 0.999999f;
                CameraAttribsB.f4Position       = camera.m_Camera.GetPos();
                CameraAttribsB.f4ViewportSize.x = static_cast<float>(pSwapChain->GetDesc().Width);
                CameraAttribsB.f4ViewportSize.y = static_cast<float>(pSwapChain->GetDesc().Height);
                CameraAttribsB.f4ViewportSize.z = 1.f / CameraAttribsB.f4ViewportSize.x;
                CameraAttribsB.f4ViewportSize.w = 1.f / CameraAttribsB.f4ViewportSize.y;

                {
                    //MapHelper<CameraAttribs> CamAttribsCBData(pContext, camera.m_pcbCameraAttribs, MAP_WRITE, MAP_FLAG_DISCARD);
                    //*CamAttribsCBData = CameraAttribsB;
                } {
                    MapHelper<CameraAttribs> CamAttribs(pContext, m_CameraAttribsCB, MAP_WRITE, MAP_FLAG_DISCARD);
                    *CamAttribs = CameraAttribsB;
                }
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

} // namespace Vision