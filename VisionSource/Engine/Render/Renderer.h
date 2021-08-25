
#pragma once

#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"

#include "../Core/OS/OS.h"
#include "../FrameWork/ECS.h"

namespace Vision
{
using namespace Diligent;

struct RenderSettings
{
    float m_EnvMapMipLevel = 1.f;
};

class Renderer : public GLTF_PBR_Renderer
{
public:
    Renderer();
    Renderer(IRenderDevice*    pDev,
             IDeviceContext*   pCtx,
             const CreateInfo& CI);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    virtual ~Renderer()                  = default;

    void Initialize();
    void Render();

    // Clears the scene and the associated renderer resources
    void ClearWorld(Scene& scene);

public:
    IEngineFactory* pEngineFactory;
    IDeviceContext* pContext;
    IRenderDevice*  pDevice;
    ISwapChain*     pSwapChain;

private:
    void CreateEnvMapPSO();
    void CreateEnvMapSRB();

private:
    enum class BackgroundMode : int
    {
        None,
        EnvironmentMap,
        Irradiance,
        PrefilteredEnvMap,
        NumModes
    } m_BackgroundMode = BackgroundMode::PrefilteredEnvMap;

    RenderInfo     m_RenderParams;
    RenderSettings m_RenderSettings;

    Ptr<GLTF_PBR_Renderer>                m_GLTFRenderer;
    RefCntAutoPtr<IBuffer>                m_CameraAttribsCB;
    RefCntAutoPtr<IBuffer>                m_LightAttribsCB;
    RefCntAutoPtr<IPipelineState>         m_EnvMapPSO;
    RefCntAutoPtr<IShaderResourceBinding> m_EnvMapSRB;
    RefCntAutoPtr<ITextureView>           m_EnvironmentMapSRV;
    RefCntAutoPtr<IBuffer>                m_EnvMapRenderAttribsCB;
};


} // namespace Vision