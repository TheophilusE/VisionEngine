
#pragma once

#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"
#include "BasicMath.hpp"

#include "../FrameWork/Scene.h"
#include "../FrameWork/Components.h"

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
    void Initialize(IEngineFactory* pEF, IRenderDevice* pD, IDeviceContext* pC, ISwapChain* pS);
    void CreateEnvMapPSO();
    void CreateEnvMapSRB();
    void Render();

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
    std::unique_ptr<GLTF_PBR_Renderer>    m_GLTFRenderer;
    RefCntAutoPtr<IPipelineState>         m_EnvMapPSO;
    RefCntAutoPtr<IShaderResourceBinding> m_EnvMapSRB;
    RefCntAutoPtr<ITextureView>           m_EnvironmentMapSRV;
    RefCntAutoPtr<IBuffer>                m_EnvMapRenderAttribsCB;

    RefCntAutoPtr<IBuffer> m_CameraAttribsCB;
    RefCntAutoPtr<IBuffer> m_LightAttribsCB;
    RenderSettings m_RenderSettings;

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