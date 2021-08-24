
#pragma once

#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"

#include "../Core/OS/OS.h"
#include "../FrameWork/ECS.h"

using namespace Diligent;

namespace Vision
{

enum class BackgroundMode : int
{
    None,
    EnvironmentMap,
    Irradiance,
    PrefilteredEnvMap,
    NumModes
};

struct RenderSettings
{
    float m_EnvMapMipLevel = 1.f;
};

class Renderer : public GLTF_PBR_Renderer
{
public:
    Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    virtual ~Renderer()                  = default;

    void Initialize();

    // Clears the scene and the associated renderer resources
    void ClearWorld(Scene& scene);

private:
    void CreateEnvMapPSO();
    void CreateEnvMapSRB();

private:
    BackgroundMode m_BackgroundMode = BackgroundMode::PrefilteredEnvMap;
    RenderInfo     m_RenderParams;
    RenderSettings m_RenderSettings;

    RefCntAutoPtr<IPipelineState>         m_EnvMapPSO;
    RefCntAutoPtr<IShaderResourceBinding> m_EnvMapSRB;
    RefCntAutoPtr<ITextureView>           m_EnvironmentMapSRV;
    RefCntAutoPtr<IBuffer>                m_EnvMapRenderAttribsCB;
};


} // namespace Vision