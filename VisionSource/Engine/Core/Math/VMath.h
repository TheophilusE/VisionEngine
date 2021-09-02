
#pragma once

#include <Eigen/Dense>
#include "AdvancedMath.hpp"

namespace Vision
{
    using Vector2f = Eigen::Vector2f;
    using Vector2d = Eigen::Vector2d;
    using Vector3f = Eigen::Vector3f;
    using Vector3d = Eigen::Vector3d;
    using Vector4f = Eigen::Vector4f;
    using Vector4d = Eigen::Vector4d;

    using Matrix2f = Eigen::Matrix2f;
    using Matrix2d = Eigen::Matrix2d;
    using Matrix3f = Eigen::Matrix3f;
    using Matrix3d = Eigen::Matrix3d;
    using Matrix4f = Eigen::Matrix4f;
    using Matrix4d = Eigen::Matrix4d;
    
    // Convert Diligent Float2 To Vector2f
    static Vision::Vector2f ConvertToVector2f(Diligent::float2& vector);
    // Convert Vector2f to Diligent Float2
    static Diligent::float2 ConvertToFloat2(Vision::Vector2f& vector);

    // Convert Diligent Double2 To Vector2d
    static Vision::Vector2d ConvertToVector2d(Diligent::double2& vector);
    // Convert Vector2f To Diligent Double2
    static Diligent::double2 ConvertToDouble2(Vision::Vector2d& vector);

    // Convert Diligent Float3 To Vector3f
    static Vision::Vector3f ConvertToVector3f(Diligent::float3& vector);
    // Convert Vector2f To Diligent Float3
    static Diligent::float3 ConvertToFloat3(Vision::Vector3f& vector);

    // Convert Diligent Double3 To Vector3d
    static Vision::Vector3d ConvertToVector3d(Diligent::double3& vector);
    // Convert Vector3d To Diligent Double3
    static Diligent::double3 ConvertToDouble3(Vision::Vector3d& vector);

    // Convert Diligent Float4 To Vector4f
    static Vision::Vector4f ConvertToVector4f(Diligent::float4& vector);
    // Convert Vector4f To Diligent Float4
    static Diligent::float4 ConvertToFloat4(Vision::Vector4f& vector);

    // Convert Diligent Double4 To Vector4d
    static Vision::Vector4d ConvertToVector4d(Diligent::double4& vector);
    // Convert Vector4d To Diligent Double4
    static Diligent::double4 ConvertToDouble4(Vision::Vector4d& vector);
}