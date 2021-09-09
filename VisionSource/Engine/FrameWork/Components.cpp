
#include "Components.h"
#include "../Core/Physics/PhysicsEngine.h"
#include "imgui.h"

namespace Vision
{

void BoundingBoxComponent::CreateBoundBoxPSO(TEXTURE_FORMAT RTVFmt, TEXTURE_FORMAT DSVFmt, Renderer& renderer)
{
    ShaderCreateInfo                               ShaderCI;
    RefCntAutoPtr<IShaderSourceInputStreamFactory> pShaderSourceFactory;
    renderer.GetEngineFactory()->CreateDefaultShaderSourceStreamFactory("shaders", &pShaderSourceFactory);
    ShaderCI.pShaderSourceStreamFactory = pShaderSourceFactory;
    ShaderCI.SourceLanguage             = SHADER_SOURCE_LANGUAGE_HLSL;
    ShaderCI.UseCombinedTextureSamplers = true;

    ShaderCI.Desc.ShaderType = SHADER_TYPE_VERTEX;
    ShaderCI.Desc.Name       = "BoundBox VS";
    ShaderCI.EntryPoint      = "BoundBoxVS";
    ShaderCI.FilePath        = "BoundBox.vsh";
    RefCntAutoPtr<IShader> pVS;
    renderer.GetRenderDevice()->CreateShader(ShaderCI, &pVS);

    ShaderCI.Desc.Name       = "BoundBox PS";
    ShaderCI.EntryPoint      = "BoundBoxPS";
    ShaderCI.FilePath        = "BoundBox.psh";
    ShaderCI.Desc.ShaderType = SHADER_TYPE_PIXEL;
    RefCntAutoPtr<IShader> pPS;
    renderer.GetRenderDevice()->CreateShader(ShaderCI, &pPS);


    GraphicsPipelineStateCreateInfo PSOCreateInfo;
    PipelineStateDesc&              PSODesc          = PSOCreateInfo.PSODesc;
    GraphicsPipelineDesc&           GraphicsPipeline = PSOCreateInfo.GraphicsPipeline;

    PSODesc.Name = "BoundBox PSO";

    GraphicsPipeline.NumRenderTargets = 1;
    GraphicsPipeline.RTVFormats[0]    = RTVFmt;
    GraphicsPipeline.DSVFormat        = DSVFmt;

    PSOCreateInfo.pVS = pVS;
    PSOCreateInfo.pPS = pPS;

    GraphicsPipeline.RTVFormats[0]              = renderer.GetSwapChain()->GetDesc().ColorBufferFormat;
    GraphicsPipeline.NumRenderTargets           = 1;
    GraphicsPipeline.DSVFormat                  = renderer.GetSwapChain()->GetDesc().DepthBufferFormat;
    GraphicsPipeline.PrimitiveTopology          = PRIMITIVE_TOPOLOGY_LINE_LIST;
    GraphicsPipeline.DepthStencilDesc.DepthFunc = COMPARISON_FUNC_LESS_EQUAL;

    renderer.GetRenderDevice()->CreateGraphicsPipelineState(PSOCreateInfo, &m_BoundBoxPSO);
    m_BoundBoxPSO->GetStaticVariableByName(SHADER_TYPE_VERTEX, "cbCameraAttribs")->Set(renderer.GetCamAttribs());
    m_BoundBoxPSO->CreateShaderResourceBinding(&m_BoundBoxSRB, true);
}

void MeshComponent::LoadModel(const char* Path, Renderer& renderer)
{
    if (m_Model)
    {
        m_PlayAnimation  = false;
        m_AnimationIndex = 0;
        m_AnimationTimers.clear();
    }

    GLTF::Model::CreateInfo ModelCI;
    ModelCI.FileName   = Path;
    ModelCI.pCacheInfo = m_bUseResourceCache ? &m_CacheUseInfo : nullptr;
    m_Model.reset(new GLTF::Model{renderer.GetRenderDevice(), renderer.GetDeviceContext(), ModelCI});

    m_ModelResourceBindings = renderer.CreateResourceBindings(*m_Model, renderer.GetCamAttribs(), renderer.GetLightAttribs());

    // Center and scale model
    float3 ModelDim{m_Model->AABBTransform[0][0], m_Model->AABBTransform[1][1], m_Model->AABBTransform[2][2]};
    float  Scale     = (1.0f / std::max(std::max(ModelDim.x, ModelDim.y), ModelDim.z)) * 0.5f;
    auto   Translate = -float3(m_Model->AABBTransform[3][0], m_Model->AABBTransform[3][1], m_Model->AABBTransform[3][2]);
    Translate += -0.5f * ModelDim;
    float4x4 InvYAxis = float4x4::Identity();
    InvYAxis._22      = -1;

    auto ModelTransform = float4x4::Translation(Translate) * float4x4::Scale(Scale) * InvYAxis;
    m_Model->Transform(ModelTransform);

    if (!m_Model->Animations.empty())
    {
        m_AnimationTimers.resize(m_Model->Animations.size());
        m_AnimationIndex = 0;
        m_PlayAnimation  = true;
    }

    m_CameraId = 0;
    m_Cameras.clear();
    for (const auto* node : m_Model->LinearNodes)
    {
        if (node->pCamera && node->pCamera->Type == GLTF::Camera::Projection::Perspective)
            m_Cameras.push_back(node->pCamera.get());
    }
}

void MeshComponent::CreateGLTFResourceCache(Renderer& renderer)
{
    Array<BufferSuballocatorCreateInfo, 3> Buffers = {};

    Buffers[0].Desc.Name          = "GLTF basic vertex attribs buffer";
    Buffers[0].Desc.BindFlags     = BIND_VERTEX_BUFFER;
    Buffers[0].Desc.Usage         = USAGE_DEFAULT;
    Buffers[0].Desc.uiSizeInBytes = sizeof(GLTF::Model::VertexBasicAttribs) * 16 << 10;

    Buffers[1].Desc.Name          = "GLTF skin attribs buffer";
    Buffers[1].Desc.BindFlags     = BIND_VERTEX_BUFFER;
    Buffers[1].Desc.Usage         = USAGE_DEFAULT;
    Buffers[1].Desc.uiSizeInBytes = sizeof(GLTF::Model::VertexSkinAttribs) * 16 << 10;

    Buffers[2].Desc.Name          = "GLTF index buffer";
    Buffers[2].Desc.BindFlags     = BIND_INDEX_BUFFER;
    Buffers[2].Desc.Usage         = USAGE_DEFAULT;
    Buffers[2].Desc.uiSizeInBytes = sizeof(Uint32) * 8 << 10;

    std::array<DynamicTextureAtlasCreateInfo, 1> Atlases;
    Atlases[0].Desc.Name      = "GLTF texture atlas";
    Atlases[0].Desc.Type      = RESOURCE_DIM_TEX_2D_ARRAY;
    Atlases[0].Desc.Usage     = USAGE_DEFAULT;
    Atlases[0].Desc.BindFlags = BIND_SHADER_RESOURCE;
    Atlases[0].Desc.Format    = TEX_FORMAT_RGBA8_UNORM;
    Atlases[0].Desc.Width     = 4096;
    Atlases[0].Desc.Height    = 4096;
    Atlases[0].Desc.MipLevels = 6;

    GLTF::ResourceManager::CreateInfo ResourceMgrCI;
    ResourceMgrCI.BuffSuballocators    = Buffers.data();
    ResourceMgrCI.NumBuffSuballocators = static_cast<Uint32>(Buffers.size());
    ResourceMgrCI.TexAtlases           = Atlases.data();
    ResourceMgrCI.NumTexAtlases        = static_cast<Uint32>(Atlases.size());

    ResourceMgrCI.DefaultAtlasDesc.Desc.Type      = RESOURCE_DIM_TEX_2D_ARRAY;
    ResourceMgrCI.DefaultAtlasDesc.Desc.Usage     = USAGE_DEFAULT;
    ResourceMgrCI.DefaultAtlasDesc.Desc.BindFlags = BIND_SHADER_RESOURCE;
    ResourceMgrCI.DefaultAtlasDesc.Desc.Width     = 4096;
    ResourceMgrCI.DefaultAtlasDesc.Desc.Height    = 4096;
    ResourceMgrCI.DefaultAtlasDesc.Desc.MipLevels = 6;

    m_pResourceMgr = GLTF::ResourceManager::Create(renderer.GetRenderDevice(), ResourceMgrCI);

    m_CacheUseInfo.pResourceMgr     = m_pResourceMgr;
    m_CacheUseInfo.VertexBuffer0Idx = 0;
    m_CacheUseInfo.VertexBuffer1Idx = 1;
    m_CacheUseInfo.IndexBufferIdx   = 2;

    m_CacheUseInfo.BaseColorFormat    = TEX_FORMAT_RGBA8_UNORM;
    m_CacheUseInfo.PhysicalDescFormat = TEX_FORMAT_RGBA8_UNORM;
    m_CacheUseInfo.NormalFormat       = TEX_FORMAT_RGBA8_UNORM;
    m_CacheUseInfo.OcclusionFormat    = TEX_FORMAT_RGBA8_UNORM;
    m_CacheUseInfo.EmissiveFormat     = TEX_FORMAT_RGBA8_UNORM;
}

RigidBodyComponent::RigidBodyComponent(int collisionGroup, int collisionMask) :
    m_CollisionGroup(collisionGroup), m_CollisionMask(collisionMask)
{
}

RigidBodyComponent::RigidBodyComponent(
    const PhysicsMaterial&       material,
    float                        volume,
    const Vector3f&               offset,
    const Vector3f&               gravity,
    const Vector3f&               angularFactor,
    int                          collisionGroup,
    int                          collisionMask,
    bool                         isMoveable,
    bool                         isKinematic,
    bool                         generatesHitEvents,
    bool                         isSleepable,
    bool                         isCCD,
    const Ref<btCollisionShape>& collisionShape) :
    RigidBodyComponent(collisionGroup, collisionMask), m_MaterialID(material), m_Volume(volume), m_Offset(offset), m_Gravity(gravity), m_AngularFactor(angularFactor), m_IsMoveable(isMoveable), m_IsGeneratesHitEvents(generatesHitEvents), m_IsSleepable(isSleepable), m_IsKinematic(isKinematic), m_IsCCD(isCCD)
{
    m_CollisionShape = collisionShape;
}

void RigidBodyComponent::RemovePhysicsBody()
{
    if (m_CollisionObject)
    {
        PhysicsEngine::GetSingleton()->RemoveCollisionObject(m_CollisionObject.get());
        m_CollisionObject.reset();
    }
}

void RigidBodyComponent::Draw()
{
    static bool showInEditor = true;
    ImGui::Checkbox("Show in Editor", &showInEditor);
    if (showInEditor)
    {
        Highlight();
    }

    ImGui::Combo("Physics Material", (int*)&m_MaterialID, PhysicsEngine::GetSingleton()->GetMaterialNames());

    if (ImGui::DragFloat3("##Offset", &m_Offset.x(), 0.01f))
    {
        SetOffset(m_Offset);
    }
    ImGui::SameLine();
    if (ImGui::Button("Offset"))
    {
        SetOffset({0.0f, 0.0f, 0.0f});
    }

    if (ImGui::DragFloat3("Gravity", &m_Gravity.x(), 0.01f))
    {
        SetGravity(m_Gravity);
    }

    if (ImGui::DragFloat3("Angular Factor", &m_AngularFactor.x(), 0.01f))
    {
        SetAngularFactor(m_AngularFactor);
    }

    if (ImGui::Button("Apply Axis Lock"))
    {
        SetAxisLock(true);
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove Axis Lock"))
    {
        SetAxisLock(false);
    }

    if (ImGui::Checkbox("Moveable", &m_IsMoveable))
    {
        SetMoveable(m_IsMoveable);
    }

    if (ImGui::Checkbox("Kinematic", &m_IsKinematic))
    {
        SetKinematic(m_IsKinematic);
    }

    if (ImGui::Checkbox("Sleepable", &m_IsSleepable))
    {
        SetSleepable(m_IsSleepable);
    }

    ImGui::Checkbox("Generates Hit Events", &m_IsGeneratesHitEvents);

    if (ImGui::Checkbox("CCD", &m_IsCCD))
    {
        SetCCD(m_IsCCD);
    }

    UpdateTransform();


    if (ImGui::TreeNodeEx("Collision Group"))
    {
        DisplayCollisionLayers(m_CollisionGroup);
        ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Collision Mask"))
    {
        DisplayCollisionLayers(m_CollisionMask);
        ImGui::TreePop();
    }
}

void RigidBodyComponent::DisplayCollisionLayers(unsigned int& collision)
{
    ImGui::CheckboxFlags("Player", &collision, (int)CollisionMask::Player);
    ImGui::CheckboxFlags("Enemy", &collision, (int)CollisionMask::Enemy);
    ImGui::CheckboxFlags("Architecture", &collision, (int)CollisionMask::Architecture);
    ImGui::CheckboxFlags("TriggerVolume", &collision, (int)CollisionMask::TriggerVolume);
    ImGui::CheckboxFlags("All", &collision, (int)CollisionMask::All);
}

void RigidBodyComponent::DetachCollisionObject()
{
    PhysicsEngine::GetSingleton()->RemoveCollisionObject(m_CollisionObject.get());
}

void RigidBodyComponent::AttachCollisionObject()
{
    PhysicsEngine::GetSingleton()->AddCollisionObject(m_CollisionObject.get(), m_CollisionGroup, m_CollisionMask);
}

bool RigidBodyComponent::SetupData()
{
    if (m_Body)
    {
        PhysicsEngine::GetSingleton()->RemoveRigidBody(m_Body);
    }

    if (m_IsMoveable)
    {
        m_Mass = m_Volume * PhysicsEngine::GetSingleton()->GetMaterialData(m_MaterialID).GravityScale;
        m_CollisionShape->calculateLocalInertia(m_Mass, m_LocalInertia);
    }
    else
    {
        m_Mass = 0.0f;
    }

    btRigidBody::btRigidBodyConstructionInfo rbInfo(m_Mass, this, m_CollisionShape.get(), m_LocalInertia);
    {
        const PhysicsMaterialData& materialData = PhysicsEngine::GetSingleton()->GetMaterialData(m_MaterialID);
        rbInfo.m_restitution                    = materialData.Restitution;
        rbInfo.m_friction                       = materialData.Friction;
    }

    m_CollisionObject.reset(new btRigidBody(rbInfo));
    m_Body = (btRigidBody*)m_CollisionObject.get();
    m_Body->setUserPointer((RigidBodyComponent*)this);
    PhysicsEngine::GetSingleton()->AddRigidBody(m_Body, m_CollisionGroup, m_CollisionMask);

    SetGravity(m_Gravity);
    SetMoveable(m_IsMoveable);
    SetKinematic(m_IsKinematic);
    SetAngularFactor(m_AngularFactor);
    SetSleepable(m_IsSleepable);
    SetCCD(m_IsCCD);

    return true;
}

void RigidBodyComponent::GetWorldTransform(btTransform& worldTrans) const
{
    //worldTrans = MatTobtTransform(getTransformComponent()->getRotationPosition() * Matrix::CreateTranslation(m_Offset));
}

void RigidBodyComponent::SetWorldTransform(const btTransform& worldTrans)
{
    //getTransformComponent()->setAbsoluteRotationPosition(Matrix::CreateTranslation(-m_Offset) * BtTransformToMat(worldTrans));
}

void RigidBodyComponent::UpdateTransform()
{
    btTransform transform;
    GetWorldTransform(transform);
    m_Body->activate(true);
    m_Body->setWorldTransform(transform);
}

void RigidBodyComponent::ApplyForce(const Vector3f& force)
{
    m_Body->activate(true);
   // m_Body->applyCentralImpulse(VecTobtVector3(force));
}

void RigidBodyComponent::ApplyTorque(const Vector3f& torque)
{
    m_Body->activate(true);
    //m_Body->applyTorqueImpulse(VecTobtVector3(torque));
}


void RigidBodyComponent::SetAngularFactor(const Vector3f& factors)
{
    m_AngularFactor = factors;
    m_Body->activate(true);
    //m_Body->setAngularFactor(VecTobtVector3(factors));
}

void RigidBodyComponent::SetAxisLock(bool enabled)
{
    if (enabled)
    {
        SetAngularFactor({0.0f, 0.0f, 0.0f});
    }
    else
    {
        SetAngularFactor({1.0f, 1.0f, 1.0f});
    }
}

void RigidBodyComponent::SetOffset(const Vector3f& offset)
{
    m_Offset = offset;
    m_Body->activate(true);
    SetupData();
}

void RigidBodyComponent::SetTransform(const Matrix4f& mat)
{
    m_Body->activate(true);
    //m_Body->setWorldTransform(MatTobtTransform(mat));
}

Matrix4f RigidBodyComponent::GetTransform()
{
    //return BtTransformToMat(m_Body->getCenterOfMassTransform());
    return Matrix4f();
}

void RigidBodyComponent::SetMoveable(bool enabled)
{
    m_IsMoveable = enabled;
    if (enabled)
    {
        m_Mass = m_Volume * PhysicsEngine::GetSingleton()->GetMaterialData(m_MaterialID).GravityScale;
        m_Body->activate(true);
        m_Body->setCollisionFlags(m_Body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
    }
    else
    {
        m_Mass = 0.0f;
        m_Body->activate(true);
        m_Body->setCollisionFlags(m_Body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    }
    m_Body->setMassProps(m_Mass, m_LocalInertia);
}

void RigidBodyComponent::SetSleepable(bool enabled)
{
    m_IsSleepable = enabled;
    if (enabled)
    {
        m_Body->forceActivationState(ACTIVE_TAG);
    }
    else
    {
        m_Body->forceActivationState(DISABLE_DEACTIVATION);
    }
}

void RigidBodyComponent::SetCCD(bool enabled)
{
    // https://github.com/godotengine/godot/blob/46de553473b4bea49176fb4316176a5662931160/modules/bullet/rigid_body_bullet.cpp#L722

    if (enabled)
    {
        // This threshold enable CCD if the object moves more than
        // 1 meter in one simulation frame
        m_Body->setCcdMotionThreshold(1e-7f);

        /// Calculate using the rule write below the CCD swept sphere radius
        ///     CCD works on an embedded sphere of radius, make sure this radius
        ///     is embedded inside the convex objects, preferably smaller:
        ///     for an object of dimensions 1 metre, try 0.2
        btScalar radius(1.0f);
        if (m_Body->getCollisionShape())
        {
            btVector3 center;
            m_Body->getCollisionShape()->getBoundingSphere(center, radius);
        }
        m_Body->setCcdSweptSphereRadius(radius * 0.2f);
    }
    else
    {
        m_Body->setCcdMotionThreshold(0.0f);
        m_Body->setCcdSweptSphereRadius(0.0f);
    }
}

void RigidBodyComponent::SetKinematic(bool enabled)
{
    m_IsKinematic = enabled;
    if (enabled)
    {
        m_Body->forceActivationState(DISABLE_DEACTIVATION);
        m_Body->setCollisionFlags(m_Body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    }
    else
    {
        m_Body->forceActivationState(ACTIVE_TAG);
        m_Body->setActivationState(ACTIVE_TAG);
        m_Body->setCollisionFlags(m_Body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    }
}

void RigidBodyComponent::Highlight()
{
    //PhysicsEngine::GetSingleton()->DebugDrawComponent(
     //   m_Body->getWorldTransform(),
      //  m_CollisionShape.get(),
      //  VecTobtVector3({0.8f, 0.1f, 0.1f}));
}

void RigidBodyComponent::SetVelocity(const Vector3f& velocity)
{
    m_Body->activate(true);
    //m_Body->setLinearVelocity(VecTobtVector3(velocity));
}

Vector3f RigidBodyComponent::GetVelocity()
{
    //return BtVector3ToVec(m_Body->getLinearVelocity());
    return Vector3f();
}

void RigidBodyComponent::SetAngularVelocity(const Vector3f& angularVel)
{
    m_Body->activate(true);
    //m_Body->setAngularVelocity(VecTobtVector3(angularVel));
}

Vector3f RigidBodyComponent::GetAngularVelocity()
{
    //return BtVector3ToVec(m_Body->getAngularVelocity());
    return Vector3f();
}

void RigidBodyComponent::Translate(const Vector3f& vec)
{
    m_Body->activate(true);
    //m_Body->translate(VecTobtVector3(vec));
}

void RigidBodyComponent::SetGravity(const Vector3f& gravity)
{
    m_Body->activate(true);
    //m_Body->setGravity(VecTobtVector3(gravity));
}

PhysicsMaterial RigidBodyComponent::GetMaterialID() const
{
    return m_MaterialID;
}

} // namespace Vision