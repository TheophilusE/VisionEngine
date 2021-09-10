
#pragma once
#pragma warning(disable : 4127)

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "AdvancedMath.hpp"

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"

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

using Quaternionf = Eigen::Quaternionf;
using Quaterniond = Eigen::Quaterniond;

using AlignedBox1i = Eigen::AlignedBox1i;
using AlignedBox1f = Eigen::AlignedBox1f;
using AlignedBox1d = Eigen::AlignedBox1d;
using AlignedBox2i = Eigen::AlignedBox2i;
using AlignedBox2f = Eigen::AlignedBox2f;
using AlignedBox2d = Eigen::AlignedBox2d;
using AlignedBox3f = Eigen::AlignedBox3f;
using AlignedBox3d = Eigen::AlignedBox3d;
using AlignedBox4f = Eigen::AlignedBox4f;
using AlignedBox4d = Eigen::AlignedBox4d;

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

// Convert Diligent Matrix To Matrix
static Vision::Matrix2f ConvertToMatrix2f(Diligent::float2x2& vector);
// Convert Matrix2f to Diligent float2x2
static Diligent::float2x2 ConvertToFloat2x2(Vision::Matrix2f& vector);

// Convert Diligent Double2 To Matrix2d
static Vision::Matrix2d ConvertToMatrix2d(Diligent::double2x2& vector);
// Convert Matrix2d To Diligent Double2x2
static Diligent::double2x2 ConvertToDouble2x2(Vision::Matrix2d& vector);

// Convert Diligent Float3x3 To Matrix3f
static Vision::Matrix3f ConvertToMatrix3f(Diligent::float3x3& vector);
// Convert Matrix3f To Diligent Float3x3
static Diligent::float3x3 ConvertToFloat3x3(Vision::Matrix3f& vector);

// Convert Diligent Double3x3 To Matrix3d
static Vision::Matrix3d ConvertToMatrix3d(Diligent::double3x3& vector);
// Convert Matrix3d To Diligent Double3x3
static Diligent::double3x3 ConvertToDouble3x3(Vision::Matrix3d& vector);

// Convert Diligent Float4x4 To Matrix4f
static Vision::Matrix4f ConvertToMatrix4f(Diligent::float4x4& vector);
// Convert Matrix4f To Diligent Float4x4
static Diligent::float4x4 ConvertToFloat4x4(Vision::Matrix4f& vector);

// Convert Diligent Double4x4 To Matrix4d
static Vision::Matrix4d ConvertToMatrix4d(Diligent::double4x4& vector);
// Convert Matrix4d To Diligent Double4x4
static Diligent::double4x4 ConvertToDouble4x4(Vision::Matrix4d& vector);

// Convert Diligent Quaternion To Quaternionf
static Vision::Quaternionf ConvertToQuaternionf(Diligent::Quaternion& vector);
// Convert Quaternionf To Diligent Quaternion
static Diligent::Quaternion ConvertToQuaternion(Vision::Quaternionf& vector);

// Convert Diligent Quaternion To Quaterniond
static Vision::Quaterniond ConvertToQuaterniond(Diligent::Quaternion& vector);
// Convert Quaterniond To Diligent Quaternion
static Diligent::Quaternion ConvertToQuaternion(Vision::Quaterniond& vector);

// Convert Matrix4f to Bullet Transform Matrix
static btTransform Matrix4fTobtTransform(Matrix4f const& matrix);

// Convert Bullet Transform Matrix To Matrix4f
static Matrix4f BtTransformToMatrix4f(btTransform const& transform);

// Convert Vector3f To Bullet Vector3
static btVector3 Vector3fTobtVector3(Vector3f const& vector);

// Convert Bullet Vector3 To Vector3f
static Vector3f BtVector3ToVector3f(btVector3 const& vector);

} // namespace Vision