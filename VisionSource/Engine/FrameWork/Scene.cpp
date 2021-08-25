
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

    auto& tag = entity.AddComponent<TagComponent>();
    tag.Tag   = name.empty() ? "Entity" : name; // Set entity tag

    return entity;
}

void Scene::DestroyEntity(Entity entity)
{
    VISION_CORE_INFO("Removed Entity" + entity.GetComponent<TagComponent>().Tag);
    m_Registry.destroy(entity);
}

void Scene::Update(float dt)
{
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