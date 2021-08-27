
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

    IEngineFactory*                GetEngineFactory() { return pEngineFactory; }
    IRenderDevice*                 GetRenderDevice() { return pDevice; }
    IDeviceContext*                GetDeviceContext() { return pContext; }
    ISwapChain*                    GetSwapChain() { return pSwapChain; }
    RefCntAutoPtr<IBuffer>         GetCamAttribs() { return m_CameraAttribsCB; }
    RefCntAutoPtr<IBuffer>         GetLightAttribs() { return m_LightAttribsCB; }
    GLTF_PBR_Renderer::RenderInfo& GetRenderParams() { return m_RenderParams; }
    RenderSettings&                GetRenderSettings() { return m_RenderSettings; }

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

    bool                                 m_bUseResourceCache = false;
    RefCntAutoPtr<GLTF::ResourceManager> m_pResourceMgr;
    GLTF::ResourceCacheUseInfo           m_CacheUseInfo;

    // Offscreen multi-sampled render target and depth-stencil - MSAA
    RefCntAutoPtr<ITextureView>     m_pMSColorRTV;
    RefCntAutoPtr<ITextureView>     m_pMSDepthDSV;
    static constexpr TEXTURE_FORMAT DepthBufferFormat = TEX_FORMAT_D32_FLOAT;

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