
#include "VMath.h"

namespace Vision
{
    static Vision::Vector2f ConvertToVector2f(Diligent::float2& vector)
    {
        return Vector2f(vector.x, vector.y);
    }

    static Diligent::float2 ConvertToFloat2(Vision::Vector2f& vector)
    {
        return Diligent::float2{vector.x(), vector.y()};
    }

    static Vision::Vector2d ConvertToVector2d(Diligent::double2& vector)
    {
        return Vector2d(vector.x, vector.y);
    }

    static Diligent::double2 ConvertToDouble2(Vision::Vector2d& vector)
    {
        return Diligent::double2{vector.x(), vector.y()};
    }

    static Vision::Vector3f ConvertToVector3f(Diligent::float3& vector)
    {
        return Vector3f(vector.x, vector.y, vector.z);
    }

    static Diligent::float3 ConvertToFloat3(Vision::Vector3f& vector)
    {
        return Diligent::float3{vector.x(), vector.y(), vector.z()};
    }

    static Vision::Vector3d ConvertToVector3d(Diligent::double3& vector)
    {
        return Vector3d(vector.x, vector.y, vector.z);
    }

    static Diligent::double3 ConvertToDouble3(Vision::Vector3d& vector)
    {
        return Diligent::double3{vector.x(), vector.y(), vector.z()};
    }

    static Vision::Vector4f ConvertToVector4f(Diligent::float4& vector)
    {
        return Vector4f(vector.x, vector.y, vector.z, vector.w);
    }

    static Diligent::float4 ConvertToFloat4(Vision::Vector4f& vector)
    {
        return Diligent::float4{vector.x(), vector.y(), vector.z(), vector.w()};
    }

    static Vision::Vector4d ConvertToVector4d(Diligent::double4& vector)
    {
        return Vector4d(vector.x, vector.y, vector.z, vector.w);
    }

    static Diligent::double4 ConvertToDouble4(Vision::Vector4d& vector)
    {
        return Diligent::double4{vector.x(), vector.y(), vector.z(), vector.w()};
    }
}