
#include "Scene.h"
#include "ECS.h"
#include "Components.h"

namespace Vision
{
static SceneID NextSceneID = ROOT_SCENE_ID + 1;
Vector<Scene*> Scene::s_Scenes;

Scene::Scene(SceneID id, const String& name, const SceneSettings& settings, ImportStyle importStyle, const String& sceneFile) :
    m_Name(name), m_ID(id), m_Settings(settings), m_ImportStyle(importStyle), m_SceneFile(sceneFile)
{
    setName(m_Name);
    s_Scenes.push_back(this);
}

Scene::Scene() {}

Scene::~Scene()
{
    int index = -1;
    for (int i = 0; i < s_Scenes.size(); i++)
    {
        if (s_Scenes[i] == this)
        {
            index = i;
        }
    }
    if (index != -1)
    {
        s_Scenes.erase(s_Scenes.begin() + index);
        m_ChildrenScenes.clear();
        VISION_CORE_INFO("Deleted scene: " + getFullName());
    }
    else
    {
        VISION_CORE_INFO("Could Not Find Scene Index");
    }
}

Ptr<Scene> Scene::Create()
{
    // Decide ID
    SceneID thisSceneID;
    thisSceneID = NextSceneID;
    NextSceneID++;

    SceneSettings sSettings;

    Ptr<Scene> thisScene(std::make_unique<Scene>(thisSceneID, "MainScene", sSettings, ImportStyle::Local, "MainScene"));

    return thisScene;
}

Entity Scene::CreateEntity(const String& name)
{
    Entity entity = {m_Registry.create(), this};
    entity.AddComponent<TransformComponent>(); // Add default transform component

    entity.AddComponent<BoundingBoxComponent>(); // Add Bounding Box Component

    auto& tag = entity.AddComponent<TagComponent>();
    tag.Tag   = name.empty() ? "Entity" : name; // Set entity tag

    return entity;
}

void Scene::DestroyEntity(Entity entity)
{
    VISION_CORE_INFO("Removed Entity" + entity.GetComponent<TagComponent>().Tag);
    m_Registry.destroy(entity);
}

void GetRaySphereIntersection(float3        f3RayOrigin,
                              const float3& f3RayDirection,
                              const float3& f3SphereCenter,
                              float         fSphereRadius,
                              float2&       f2Intersections)
{
    // http://wiki.cgsociety.org/index.php/Ray_Sphere_Intersection
    f3RayOrigin -= f3SphereCenter;
    float A = dot(f3RayDirection, f3RayDirection);
    float B = 2 * dot(f3RayOrigin, f3RayDirection);
    float C = dot(f3RayOrigin, f3RayOrigin) - fSphereRadius * fSphereRadius;
    float D = B * B - 4 * A * C;
    // If discriminant is negative, there are no real roots hence the ray misses the
    // sphere
    if (D < 0)
    {
        f2Intersections = float2(-1, -1);
    }
    else
    {
        D = sqrt(D);

        f2Intersections = float2(-B - D, -B + D) / (2 * A); // A must be positive here!!
    }
}

void ComputeApproximateNearFarPlaneDist(const float3&   CameraPos,
                                        const float4x4& ViewMatr,
                                        const float4x4& ProjMatr,
                                        const float3&   EarthCenter,
                                        float           fEarthRadius,
                                        float           fMinRadius,
                                        float           fMaxRadius,
                                        float&          fNearPlaneZ,
                                        float&          fFarPlaneZ)
{
    float4x4 ViewProjMatr = ViewMatr * ProjMatr;
    float4x4 ViewProjInv  = ViewProjMatr.Inverse();

    // Compute maximum view distance for the current camera altitude
    float3 f3CameraGlobalPos   = CameraPos - EarthCenter;
    float  fCameraElevationSqr = dot(f3CameraGlobalPos, f3CameraGlobalPos);
    float  fMaxViewDistance =
        (float)(sqrt((double)fCameraElevationSqr - (double)fEarthRadius * fEarthRadius) +
                sqrt((double)fMaxRadius * fMaxRadius - (double)fEarthRadius * fEarthRadius));
    float fCameraElev = sqrt(fCameraElevationSqr);

    fNearPlaneZ = 50.f;
    if (fCameraElev > fMaxRadius)
    {
        // Adjust near clipping plane
        fNearPlaneZ = (fCameraElev - fMaxRadius) / sqrt(1 + 1.f / (ProjMatr._11 * ProjMatr._11) + 1.f / (ProjMatr._22 * ProjMatr._22));
    }

    fNearPlaneZ = std::max(fNearPlaneZ, 50.f);
    fFarPlaneZ  = 1000;

    const int iNumTestDirections = 5;
    for (int i = 0; i < iNumTestDirections; ++i)
    {
        for (int j = 0; j < iNumTestDirections; ++j)
        {
            float3 PosPS, PosWS, DirFromCamera;
            PosPS.x = (float)i / (float)(iNumTestDirections - 1) * 2.f - 1.f;
            PosPS.y = (float)j / (float)(iNumTestDirections - 1) * 2.f - 1.f;
            PosPS.z = 0; // Far plane is at 0 in complimentary depth buffer
            PosWS   = PosPS * ViewProjInv;

            DirFromCamera = PosWS - CameraPos;
            DirFromCamera = normalize(DirFromCamera);

            float2 IsecsWithBottomBoundSphere;
            GetRaySphereIntersection(CameraPos, DirFromCamera, EarthCenter, fMinRadius, IsecsWithBottomBoundSphere);

            float fNearIsecWithBottomSphere = IsecsWithBottomBoundSphere.x > 0 ? IsecsWithBottomBoundSphere.x : IsecsWithBottomBoundSphere.y;
            if (fNearIsecWithBottomSphere > 0)
            {
                // The ray hits the Earth. Use hit point to compute camera space Z
                float3 HitPointWS = CameraPos + DirFromCamera * fNearIsecWithBottomSphere;
                float3 HitPointCamSpace;
                HitPointCamSpace = HitPointWS * ViewMatr;
                fFarPlaneZ       = std::max(fFarPlaneZ, HitPointCamSpace.z);
            }
            else
            {
                // The ray misses the Earth. In that case the whole earth could be seen
                fFarPlaneZ = fMaxViewDistance;
            }
        }
    }
}

void Scene::Update(Diligent::InputController& controller, IRenderDevice* pDevice, ISwapChain* pSwapChain, float dt)
{
    const auto& SCDesc = pSwapChain->GetDesc();
    // Set world/view/proj matrices and global shader constants
    float aspectRatio   = (float)SCDesc.Width / SCDesc.Height;
    bool  m_bIsGLDevice = pDevice->GetDeviceInfo().IsGLDevice();

    // This projection matrix is only used to set up directions in view frustum
    // Actual near and far planes are ignored
    float    FOV      = PI_F / 4.f;
    float4x4 mTmpProj = float4x4::Projection(FOV, aspectRatio, 50.f, 500000.f, m_bIsGLDevice);

    auto& scenes = Scene::FindAllScenes();

    for (int i = 0; i < scenes.size(); ++i)
    {
        auto& pRegistry          = scenes[i]->GetSceneRegistry();
        auto  viewCamera         = pRegistry.view<CameraComponent>();
        auto  viewModel          = pRegistry.view<MeshComponent>();
        auto  viewTransformModel = pRegistry.view<TransformComponent, MeshComponent>();

        // Update Camera
        for (auto entity : viewCamera)
        {
            auto& camera = viewCamera.get<CameraComponent>(entity);
            if (camera.Active)
            {
                camera.m_Camera.Update(controller, dt);
            }
        }

        // Update model
        for (auto entity : viewModel)
        {
            auto& model = viewModel.get<MeshComponent>(entity);

            if (model.m_Model != nullptr)
            {
                if (!model.m_Model->Animations.empty() && model.m_PlayAnimation)
                {
                    float& AnimationTimer = model.m_AnimationTimers[model.m_AnimationIndex];
                    AnimationTimer += dt;
                    AnimationTimer = std::fmod(AnimationTimer, model.m_Model->Animations[model.m_AnimationIndex].End);
                    model.m_Model->UpdateAnimation(model.m_AnimationIndex, AnimationTimer);
                }
            }
        }

        // Update model transform
        // for (auto [entity, transform, model] : viewTransformModel) {}
        for (auto entity : viewTransformModel)
        {
            auto& transform = viewTransformModel.get<TransformComponent>(entity);
            auto& model     = viewTransformModel.get<MeshComponent>(entity);

            model.m_RenderParams.ModelTransform = transform.GetTransform();
        }
    }
}

template <typename T>
void Scene::OnComponentAdded(Entity entity, T& component)
{
    static_assert(false);
}

template <>
void Scene::OnComponentAdded<TagComponent>(Entity entity, TagComponent& component)
{
    VISION_CORE_INFO("Added A Tag Component");
}

template <>
void Scene::OnComponentAdded<TransformComponent>(Entity entity, TransformComponent& component)
{
    VISION_CORE_INFO("Added A Transform Component");
}

template <>
void Scene::OnComponentAdded<BoundingBoxComponent>(Entity entity, BoundingBoxComponent& component)
{
    VISION_CORE_INFO("Added A Bounding Box Component");
}

template <>
void Scene::OnComponentAdded<DirectionalLightComponent>(Entity entity, DirectionalLightComponent& component)
{
    VISION_CORE_INFO("Added A Directional Light Component");
}

template <>
void Scene::OnComponentAdded<CameraComponent>(Entity entity, CameraComponent& component)
{
    VISION_CORE_INFO("Added A Camera Component");
}

template <>
void Scene::OnComponentAdded<MeshComponent>(Entity entity, MeshComponent& component)
{
    VISION_CORE_INFO("Added A Mesh Component");
}

void Scene::ResetNextID()
{
    NextSceneID = ROOT_SCENE_ID + 1;
}

Ptr<Scene> Scene::CreateRootScene()
{
    static bool called = false;
    if (called)
    {
        VISION_CORE_ERROR("Denied creating a second root scene. Scene hierarchy may be corrupted.");
        return nullptr;
    }

    Ptr<Scene> root = std::make_unique<Scene>(ROOT_SCENE_ID, "Root", SceneSettings(), ImportStyle::Local, "");

    called = true;
    return root;
}

Vector<Scene*> Scene::FindScenesByName(const String& name)
{
    Vector<Scene*> foundScenes;
    for (auto& scene : s_Scenes)
    {
        if (scene->m_Name == name)
        {
            foundScenes.push_back(scene);
        }
    }
    return foundScenes;
}

Scene* Scene::FindSceneByID(const SceneID& id)
{
    for (auto& scene : s_Scenes)
    {
        if (scene->m_ID == id)
        {
            return scene;
        }
    }
    return nullptr;
}

const Vector<Scene*>& Scene::FindAllScenes()
{
    return s_Scenes;
}


Scene* Scene::findScene(SceneID scene)
{
    if (scene == getID())
    {
        return this;
    }
    for (auto& child : m_ChildrenScenes)
    {
        if (Scene* entityScene = child->findScene(scene))
        {
            return entityScene;
        }
    }
    return nullptr;
}

void Scene::reimport()
{
    if (m_ImportStyle != ImportStyle::External)
    {
        VISION_CORE_WARN("Did not reimport local scene. Needs to be external to be reimported.");
        return;
    }
}

void Scene::onLoad()
{
    for (auto& child : m_ChildrenScenes)
    {
        child->onLoad();
    }
}

bool Scene::snatchChild(Scene* child)
{
    if (!checkCycle(child))
    {
        return false;
    }

    Vector<Ptr<Scene>>& children = child->getParent()->getChildren();
    for (int i = 0; i < children.size(); i++)
    {
        if (children.at(i).get() == child)
        {
            m_ChildrenScenes.push_back(std::move(children[i]));
            children.erase(children.begin() + i);
        }
    }
    child->m_ParentScene = this;
    return true;
}

bool Scene::checkCycle(Scene* child)
{
    if (child->findScene(m_ID) != nullptr)
    {
        VISION_CORE_WARN("Tried to make a scene its own child's child");
        return false;
    }
    return true;
}

bool Scene::addChild(Ptr<Scene>& child)
{
    if (!child)
    {
        VISION_CORE_WARN("Tried to add a null scene to: " + getFullName() + ". Denied.");
        return false;
    }
    if (!checkCycle(child.get()))
    {
        return false;
    }
    // Removed auto&
    auto findIt = std::find(m_ChildrenScenes.begin(), m_ChildrenScenes.end(), child);
    if (findIt == m_ChildrenScenes.end())
    {
        child->m_ParentScene = this;
        m_ChildrenScenes.emplace_back(std::move(child));
        //ScriptSystem::GetSingleton()->addEnterScriptEntity(&m_ChildrenScenes.back()->getEntity());
    }
    else
    {
        VISION_CORE_ERROR("Tried to add a duplicate child " + child->getFullName() + " to " + getFullName());
        return false;
    }
    return true;
}

bool Scene::removeChild(Scene* toRemove)
{
    // Removed auto&
    for (auto child = m_ChildrenScenes.begin(); child != m_ChildrenScenes.end(); child++)
    {
        if ((*child).get() == toRemove)
        {
            m_ChildrenScenes.erase(child);
            return true;
        }
    }
    return false;
}

void Scene::setName(const String& name)
{
    m_Name     = name;
    m_FullName = name + " # " + std::to_string(m_ID);
}

} // namespace Vision