
#pragma once

#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"
#include "AdvancedMath.hpp"
#include "ShadowMapManager.hpp"

#include "../FrameWork/Scene.h"

namespace Vision
{
using namespace Diligent;

struct RenderSettings
{
    float m_EnvMapMipLevel = 1.0f;

    // MSAA
    bool   m_UseMSAA               = false;
    Uint8  m_SampleCount           = 4;
    Uint32 m_SupportedSampleCounts = 0;
};

struct ShadowSettings
{
    bool           SnapCascades            = true;
    bool           StabilizeExtents        = true;
    bool           EqualizeExtents         = true;
    bool           SearchBestCascade       = true;
    bool           FilterAcrossCascades    = true;
    int            Resolution              = 2048;
    float          PartitioningFactor      = 0.95f;
    int            m_iNumShadowCascades    = 6;
    int            m_bBestCascadeSearch    = 1;
    int            m_FixedShadowFilterSize = 5;
    TEXTURE_FORMAT ShadowMapFormat         = TEX_FORMAT_D16_UNORM;
    TEXTURE_FORMAT DstRTVFormat            = TEX_FORMAT_R11G11B10_FLOAT;
    int            iShadowMode             = SHADOW_MODE_PCF;

    bool Is32BitFilterableFmt = true;
};

class Renderer
{
public:
    void                                     Initialize(IEngineFactory* pEF, IRenderDevice* pD, IDeviceContext* pC, ISwapChain* pS);
    void                                     CreateEnvMapPSO();
    void                                     CreateEnvMapSRB();
    void                                     Render();
    GLTF_PBR_Renderer::ModelResourceBindings CreateResourceBindings(GLTF::Model& GLTFModel,
                                                                    IBuffer*     pCameraAttribs,
                                                                    IBuffer*     pLightAttribs);
    void                                     CreateMSAARenderTarget();
    void                                     CreateShadowMap();
    void                                     RenderShadowMap();
    void                                     CreatePipelineStates();
    void                                     InitializeResourceBindings();

    IEngineFactory*                GetEngineFactory() { return pEngineFactory; }
    IRenderDevice*                 GetRenderDevice() { return pDevice; }
    IDeviceContext*                GetDeviceContext() { return pContext; }
    ISwapChain*                    GetSwapChain() { return pSwapChain; }
    RefCntAutoPtr<IBuffer>         GetCamAttribs() { return m_CameraAttribsCB; }
    RefCntAutoPtr<IBuffer>         GetLightAttribs() { return m_LightAttribsCB; }
    GLTF_PBR_Renderer::RenderInfo& GetRenderParams() { return m_RenderParams; }
    RenderSettings&                GetRenderSettings() { return m_RenderSettings; }
    ShadowSettings&                GetShadowSettings() { return m_ShadowSettings; }
    ShadowMapManager&              GetShadowMapManager() { return m_ShadowMapMgr; }

    enum class BackgroundMode : int
    {
        None,
        EnvironmentMap,
        Irradiance,
        PrefilteredEnvMap,
        NumModes
    } m_BackgroundMode = BackgroundMode::PrefilteredEnvMap;

protected:
    GLTF_PBR_Renderer::RenderInfo         m_RenderParams;
    Ptr<GLTF_PBR_Renderer>                m_GLTFRenderer;
    RefCntAutoPtr<IPipelineState>         m_EnvMapPSO;
    RefCntAutoPtr<IShaderResourceBinding> m_EnvMapSRB;
    RefCntAutoPtr<ITextureView>           m_EnvironmentMapSRV;
    RefCntAutoPtr<IBuffer>                m_EnvMapRenderAttribsCB;

    RefCntAutoPtr<IBuffer> m_CameraAttribsCB;
    RefCntAutoPtr<IBuffer> m_LightAttribsCB;
    RenderSettings         m_RenderSettings;
    ShadowSettings         m_ShadowSettings;

    bool                                 m_bUseResourceCache = false;
    RefCntAutoPtr<GLTF::ResourceManager> m_pResourceMgr;
    GLTF::ResourceCacheUseInfo           m_CacheUseInfo;

    // Offscreen multi-sampled render target and depth-stencil - MSAA
    RefCntAutoPtr<ITextureView>     m_pMSColorRTV;
    RefCntAutoPtr<ITextureView>     m_pMSDepthDSV;
    static constexpr TEXTURE_FORMAT DepthBufferFormat = TEX_FORMAT_D32_FLOAT;

    // ShadowMap
    ShadowMapManager        m_ShadowMapMgr;
    RefCntAutoPtr<ISampler> m_pComparisonSampler;
    RefCntAutoPtr<ISampler> m_pFilterableShadowMapSampler;

protected:
    // Returns projection matrix adjusted to the current screen orientation
    float4x4 GetAdjustedProjectionMatrix(float FOV, float NearPlane, float FarPlane) const;

    // Returns pretransform matrix that matches the current screen rotation
    float4x4 GetSurfacePretransformMatrix(const float3& f3CameraViewAxis) const;

private:
    IEngineFactory* pEngineFactory;
    IRenderDevice*  pDevice;
    IDeviceContext* pContext;
    ISwapChain*     pSwapChain;
};
} // namespace Vision