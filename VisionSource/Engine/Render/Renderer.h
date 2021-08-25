
#pragma once

#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"
#include "BasicMath.hpp"

#include "../FrameWork/Scene.h"

namespace Vision
{
using namespace Diligent;

struct RenderSettings
{
    float m_EnvMapMipLevel = 1.0f;
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

    IEngineFactory*        GetEngineFactory() { return pEngineFactory; }
    IRenderDevice*         GetRenderDevice() { return pDevice; }
    IDeviceContext*        GetDeviceContext() { return pContext; }
    ISwapChain*            GetSwapChain() { return pSwapChain; }
    RefCntAutoPtr<IBuffer> GetCamAttribs() { return m_CameraAttribsCB; }
    RefCntAutoPtr<IBuffer> GetLightAttribs() { return m_LightAttribsCB; }

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

private:
    IEngineFactory* pEngineFactory;
    IRenderDevice*  pDevice;
    IDeviceContext* pContext;
    ISwapChain*     pSwapChain;
};
} // namespace Vision