
#pragma once
#include "../Core/OS/OS.h"

#include <vector>

#include "FirstPersonCamera.hpp"

#include "FileSystem.hpp"
#include "GraphicsUtilities.h"
#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"
#include "MapHelper.hpp"
#include "EpipolarLightScattering.hpp"
#include "Shaders/Common/public/BasicStructures.fxh"
#include "Shaders/PostProcess/ToneMapping/public/ToneMappingStructures.fxh"
#include "TextureUtilities.h"
#include "CommonlyUsedStates.h"
#include "ShaderMacroHelper.hpp"
#include "../Render/Renderer.h"

namespace Vision
{
using namespace Diligent;

struct TagComponent
{
    String Tag;

    TagComponent()                    = default;
    TagComponent(const TagComponent&) = default;
    TagComponent(const String& tag) :
        Tag(tag) {}

    inline void operator=(const String& str) { Tag = str; }
    inline void operator=(String&& str) { Tag = std::move(str); }
    inline bool operator==(const std::string& str) const { return Tag.compare(str) == 0; }
};

struct TransformComponent
{
    float3     Translation = {0.0f, 0.0f, 0.0f};
    Quaternion Rotation    = Quaternion::RotationFromAxisAngle(float3{0.f, 1.0f, 0.0f}, -PI_F / 2.f);
    float3     Scale       = {1.0f, 1.0f, 1.0f};

    TransformComponent()                          = default;
    TransformComponent(const TransformComponent&) = default;
    TransformComponent(const Diligent::float3& translation) :
        Translation(translation) {}

    float4x4 GetTransform() const
    {
        return float4x4::Translation(Translation) * Rotation.ToMatrix() * float4x4::Scale(Scale);
    }
};

struct BoundingBoxComponent
{
    BoundingBoxComponent()                            = default;
    BoundingBoxComponent(const BoundingBoxComponent&) = default;

    enum class BoundBoxMode : int
    {
        None = 0,
        Local,
        Global
    };
    BoundBoxMode                          m_BoundBoxMode = BoundBoxMode::None;
    RefCntAutoPtr<IPipelineState>         m_BoundBoxPSO;
    RefCntAutoPtr<IShaderResourceBinding> m_BoundBoxSRB;

    void CreateBoundBoxPSO(TEXTURE_FORMAT RTVFmt, TEXTURE_FORMAT DSVFmt, Renderer& renderer);
};

struct DirectionalLightComponent
{
    DirectionalLightComponent()                                 = default;
    DirectionalLightComponent(const DirectionalLightComponent&) = default;

    float3 m_LightDirection = {-0.554699242f, -0.0599640049f, -0.829887390f};
    float4 m_LightColor     = float4(1, 1, 1, 1);
    float  m_LightIntensity = 3.f;

    RefCntAutoPtr<IBuffer> m_LightAttribsCB;

    float4 GetIntensity()
    {
        return m_LightColor * m_LightIntensity;
    }
};

struct CameraComponent
{
    CameraComponent()                       = default;
    CameraComponent(const CameraComponent&) = default;

    bool              Active          = true;
    float2            sensorDimension = {0.036f, 0.024f}; // mm
    float             focalLength     = 0.04327f;       // mm
    float             linearDistance  = 0.001f;
    FirstPersonCamera m_Camera;
};

struct MeshComponent
{
    MeshComponent() = default;
    //MeshComponent(const MeshComponent&) = default;

    void LoadModel(const char* Path, Renderer& renderer);
    void CreateGLTFResourceCache(Renderer& renderer);

    Ptr<GLTF::Model>                         m_Model;
    GLTF_PBR_Renderer::RenderInfo            m_RenderParams;
    GLTF_PBR_Renderer::ModelResourceBindings m_ModelResourceBindings;
    GLTF_PBR_Renderer::ResourceCacheBindings m_CacheBindings;

    bool                                 m_PlayAnimation  = false;
    int                                  m_AnimationIndex = 0;
    Vector<float>                        m_AnimationTimers;
    bool                                 m_bUseResourceCache = false;
    RefCntAutoPtr<GLTF::ResourceManager> m_pResourceMgr;
    GLTF::ResourceCacheUseInfo           m_CacheUseInfo;

    Uint32                           m_CameraId = 0;
    std::vector<const GLTF::Camera*> m_Cameras;
};

} // namespace Vision