
#pragma once
#include "../Core/OS/OS.h"

#include "../Core/Math/VMath.h"
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

#include "btBulletCollisionCommon.h"

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
    LightAttribs           m_LightAttribs;

    float4 GetIntensity()
    {
        return m_LightColor * m_LightIntensity;
    }
};

struct CameraComponent
{
    CameraComponent()                       = default;
    CameraComponent(const CameraComponent&) = default;

    bool   Active          = true;
    float2 sensorDimension = {0.036f, 0.024f}; // mm
    float  focalLength     = 0.04327f;         // mm
    float  linearDistance  = 0.1f;

    float speed           = 1.f;
    float speedScale      = 5.f;
    float superSpeedScale = 10.f;
    float rotationScale   = 0.01f;

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

    Uint32                      m_CameraId = 0;
    Vector<const GLTF::Camera*> m_Cameras;

    // Shadows
    Vector<Uint32>                                m_PSOIndex;
    Vector<RefCntAutoPtr<IPipelineState>>         m_RenderMeshShadowPSO;
    Vector<RefCntAutoPtr<IShaderResourceBinding>> m_ShadowSRBs;
};

enum class CollisionMask : unsigned int
{
    None          = 0,
    Player        = 1 << 0,
    Enemy         = 1 << 1,
    Architecture  = 1 << 2,
    TriggerVolume = 1 << 3,
    Other         = 1 << 4,
    All           = Player | Enemy | Architecture | TriggerVolume | Other
};

enum PhysicsMaterial;

struct RigidBodyComponent
{
    RigidBodyComponent() = default;
    //RigidBodyComponent(const RigidBodyComponent&) = default;

public:
    RigidBodyComponent(int collisionGroup, int collisionMask);
    virtual ~RigidBodyComponent() = default;

    RigidBodyComponent(
        const PhysicsMaterial&                   material,
        float                        volume,
        const Vector3f&              offset,
        const Vector3f&              gravity,
        const Vector3f&              angularFactor,
        int                          collisionGroup,
        int                          collisionMask,
        bool                         isMoveable,
        bool                         isKinematic,
        bool                         generatesHitEvents,
        bool                         canSleep,
        bool                         isCCD,
        const Ref<btCollisionShape>& collisionShape);


    //virtual void handleHit(Hit* h);

    void RemovePhysicsBody();
    void Draw();
    void DisplayCollisionLayers(unsigned int& collision);

    void ApplyForce(const Vector3f& force);
    void ApplyTorque(const Vector3f& torque);

    Vector3f GetAngularFactor() const { return m_AngularFactor; }
    void     SetAngularFactor(const Vector3f& factors);
    void     SetAxisLock(bool enabled);

    Vector3f GetOffset() const { return m_Offset; };
    void     SetOffset(const Vector3f& offset);

    Vector3f GetGravity() const { return m_Gravity; };
    void     SetGravity(const Vector3f& gravity);

    PhysicsMaterial GetMaterialID() const;

    Vector3f GetVelocity();
    void     SetVelocity(const Vector3f& velocity);

    Vector3f GetAngularVelocity();
    void     SetAngularVelocity(const Vector3f& angularVel);

    void Translate(const Vector3f& vec);

    void     SetTransform(const Matrix4f& matrix);
    Matrix4f GetTransform();

    bool IsMoveable() { return m_IsMoveable; }
    void SetMoveable(bool enabled);

    bool CanSleep() { return m_IsSleepable; }
    void SetSleepable(bool enabled);

    bool IsCCD() { return m_IsCCD; }
    void SetCCD(bool enabled);

    bool IsGeneratesHitEvents() { return m_IsGeneratesHitEvents; }
    void SetGeneratedHitEvents(bool enabled) { m_IsGeneratesHitEvents = enabled; }

    bool IsKinematic() { return m_IsKinematic; }
    void SetKinematic(bool enabled);

    bool SetupData();

    void Highlight();
    
protected:
    Ref<btCollisionObject> m_CollisionObject;
    unsigned int           m_CollisionGroup;
    unsigned int           m_CollisionMask;

    Ref<btCollisionShape> m_CollisionShape;
    btRigidBody*          m_Body = nullptr;
    bool                  m_IsGeneratesHitEvents;
    btScalar              m_Mass;
    Vector3f              m_Gravity;
    Vector3f              m_AngularFactor;
    Vector3f              m_Offset;
    float                 m_Volume;
    bool                  m_IsMoveable;
    bool                  m_IsKinematic;
    bool                  m_IsSleepable;
    bool                  m_IsCCD;
    PhysicsMaterial       m_MaterialID;

    btVector3 m_LocalInertia;

    void DetachCollisionObject();
    void AttachCollisionObject();

    void GetWorldTransform(btTransform& worldTrans) const;
    void SetWorldTransform(const btTransform& worldTrans);

    void UpdateTransform();

    //void handleHit(Hit* hit) override;
};

} // namespace Vision