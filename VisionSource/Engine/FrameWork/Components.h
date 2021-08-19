
#pragma once
#include "../Core/OS/OS.h"

namespace Vision
{
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

}