
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

static Vision::Matrix2f ConvertToMatrix2f(Diligent::float2x2& vector)
{
    /*
        return Matrix2f {
            {vector.m00, vector.m01},
            {vector.m10, vector.m11}
        };
        */
    /*
       Matrix2f result;
       result(0, 0) = vector.m00;
       result(0, 1) = vector.m01;
       result(1, 0) = vector.m10;
       result(1, 1) = vector.m11;
       */

    Matrix2f result;
    result << vector.m00, vector.m01,
        vector.m10, vector.m11;

    return result;
}

static Diligent::float2x2 ConvertToFloat2x2(Vision::Matrix2f& vector)
{
    return Diligent::float2x2{
        vector(0, 0), vector(0, 1),
        vector(1, 0), vector(1, 1)};
}

static Vision::Matrix2d ConvertToMatrix2d(Diligent::double2x2& vector)
{
    Vision::Matrix2d result;

    result << vector.m00, vector.m01,
        vector.m10, vector.m11;

    return result;
}

static Diligent::double2x2 ConvertToDouble2x2(Vision::Matrix2d& vector)
{
    return Diligent::double2x2{
        vector(0, 0), vector(0, 1),
        vector(1, 0), vector(1, 1)};
}

static Vision::Matrix3f ConvertToMatrix3f(Diligent::float3x3& vector)
{
    Matrix3f result;

    result << vector.m00, vector.m01, vector.m02,
        vector.m10, vector.m11, vector.m12,
        vector.m20, vector.m21, vector.m22;
}

static Diligent::float3x3 ConvertToFloat3x3(Vision::Matrix3f& vector)
{
    return Diligent::float3x3{
        vector(0, 0),
        vector(0, 1),
        vector(0, 2),
        vector(1, 0),
        vector(1, 1),
        vector(1, 2),
        vector(2, 0),
        vector(2, 1),
        vector(2, 2),
    };
}

static Vision::Matrix3d ConvertToMatrix3d(Diligent::double3x3& vector)
{
    Matrix3d result;

    result << vector.m00, vector.m01, vector.m02,
        vector.m10, vector.m11, vector.m12,
        vector.m20, vector.m21, vector.m22;

    return result;
}

static Diligent::double3x3 ConvertToDouble3x3(Vision::Matrix3d& vector)
{
    return Diligent::double3x3{
        vector(0, 0),
        vector(0, 1),
        vector(0, 2),
        vector(1, 0),
        vector(1, 1),
        vector(1, 2),
        vector(2, 0),
        vector(2, 1),
        vector(2, 2),
    };
}

static Vision::Matrix4f ConvertToMatrix4f(Diligent::float4x4& vector)
{
    Matrix4f result;

    result << vector.m00, vector.m01, vector.m02, vector.m03,
        vector.m10, vector.m11, vector.m12, vector.m13,
        vector.m20, vector.m21, vector.m22, vector.m23,
        vector.m30, vector.m31, vector.m32, vector.m33;

    return result;
}

static Diligent::float4x4 ConvertToFloat4x4(Vision::Matrix4f& vector)
{
    return Diligent::float4x4{
        vector(0, 0), vector(0, 1), vector(0, 2), vector(0, 3),
        vector(1, 0), vector(1, 1), vector(1, 2), vector(1, 3),
        vector(2, 0), vector(2, 1), vector(2, 2), vector(2, 3),
        vector(3, 0), vector(3, 1), vector(3, 2), vector(3, 3)};
}

static Vision::Matrix4d ConvertToMatrix4d(Diligent::double4x4& vector)
{
    Matrix4d result;

    result << vector.m00, vector.m01, vector.m02, vector.m03,
        vector.m10, vector.m11, vector.m12, vector.m13,
        vector.m20, vector.m21, vector.m22, vector.m23,
        vector.m30, vector.m31, vector.m32, vector.m33;

    return result;
}

static Diligent::double4x4 ConvertToDouble4x4(Vision::Matrix4d& vector)
{
    return Diligent::double4x4{
        vector(0, 0), vector(0, 1), vector(0, 2), vector(0, 3),
        vector(1, 0), vector(1, 1), vector(1, 2), vector(1, 3),
        vector(2, 0), vector(2, 1), vector(2, 2), vector(2, 3),
        vector(3, 0), vector(3, 1), vector(3, 2), vector(3, 3)};
}

static Vision::Quaternionf ConvertToQuaternionf(Diligent::Quaternion& vector)
{
    return Quaternionf(vector.q.w, vector.q.x, vector.q.y, vector.q.z);
}

static Diligent::Quaternion ConvertToQuaternion(Vision::Quaternionf& vector)
{
    return Diligent::Quaternion{vector.x(), vector.y(), vector.z(), vector.w()};
}

static Vision::Quaterniond ConvertToQuaterniond(Diligent::Quaternion& vector)
{
    return Quaterniond(vector.q.w, vector.q.x, vector.q.y, vector.q.z);
}

static Diligent::Quaternion ConvertToQuaternion(Vision::Quaterniond& vector)
{
    return Diligent::Quaternion{static_cast<float>(vector.x()), static_cast<float>(vector.y()), static_cast<float>(vector.z()), static_cast<float>(vector.w())};
}

static btTransform Matrix4fTobtTransform(Matrix4f const& matrix)
{
    // convert from Mat4x4 to btTransform
    btMatrix3x3 bulletRotation;
    btVector3   bulletPosition;

    // copy rotation matrix
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            bulletRotation[row][column] = matrix(column, row);
            // note the reversed indexing (row/column vs. column/row)
            // this is because Mat4x4s are row-major matrices and
            // btMatrix3x3 are column-major.  This reversed indexing
            // implicitly transposes (flips along the diagonal)
            // the matrix when it is copied.
        }
    }

    // copy position
    for (int column = 0; column < 3; ++column)
    {
        bulletPosition[column] = matrix(3, column);
    }

    return btTransform(bulletRotation, bulletPosition);
}

Matrix4f BtTransformToMatrix4f(btTransform const& transform)
{
    Matrix4f returnValue = Matrix4f::Identity();

    // convert from btTransform to Mat4x4
    btMatrix3x3 const& bulletRotation = transform.getBasis();
    btVector3 const&   bulletPosition = transform.getOrigin();

    // copy rotation matrix
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            returnValue(row, column) = bulletRotation[column][row];
            // note the reversed indexing (row/column vs. column/row)
            // this is because Mat4x4s are row-major matrices and
            // btMatrix3x3 are column-major.  This reversed indexing
            // implicitly transposes (flips along the diagonal)
            // the matrix when it is copied.
        }
    }

    // copy position
    for (int column = 0; column < 3; ++column)
    {
        returnValue(3, column) = bulletPosition[column];
    }

    return returnValue;
}

static btVector3 Vector3fTobtVector3(Vector3f const& vector)
{
    return btVector3(vector.x(), vector.y(), vector.z());
}

static Vector3f BtVector3ToVector3f(btVector3 const& vector)
{
    return Vector3f(vector.x(), vector.y(), vector.z());
}

} // namespace Vision