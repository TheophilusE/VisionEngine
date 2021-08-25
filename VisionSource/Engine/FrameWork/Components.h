
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
#include "ShadowMapManager.hpp"
#include "Shaders/Common/public/BasicStructures.fxh"
#include "Shaders/PostProcess/ToneMapping/public/ToneMappingStructures.fxh"
#include "TextureUtilities.h"
#include "CommonlyUsedStates.h"
#include "ShaderMacroHelper.hpp"

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
    Diligent::float3 Translation = {0.0f, 0.0f, 0.0f};
    Diligent::float4 Rotation    = {0.0f, 0.0f, 0.0f, 1.0f};
    Diligent::float3 Scale       = {1.0f, 1.0f, 1.0f};

    TransformComponent()                          = default;
    TransformComponent(const TransformComponent&) = default;
    TransformComponent(const Diligent::float3& translation) :
        Translation(translation) {}

    Diligent::float4x4 GetTransform() const
    {
        return Diligent::float4x4::Translation(Translation) * Diligent::Quaternion::MakeQuaternion(Rotation).ToMatrix() * Diligent::float4x4::Scale(Scale);
    }
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

    bool                   Active = true;
    FirstPersonCamera      m_Camera;
    RefCntAutoPtr<IBuffer> m_pcbCameraAttribs;
};

struct MeshComponent
{
    MeshComponent()                     = default;
    MeshComponent(const MeshComponent&) = default;
};

} // namespace Vision