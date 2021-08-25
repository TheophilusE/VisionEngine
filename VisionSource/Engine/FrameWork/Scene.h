
#pragma once

#include "entt/entt.hpp"
#include "../Input/InputManager.h"
#include "InputController.hpp"
#include "GLTFLoader.hpp"
#include "GLTF_PBR_Renderer.hpp"

#define ROOT_SCENE_ID 1

namespace Vision
{
using namespace Diligent;
class Entity;
typedef unsigned int SceneID;

struct SceneSettings
{
    HashMap<String, InputScheme> inputSchemes;
    String                       startScheme = {};
};

enum class ImportStyle
{
    /// If scene is not imported but created raw inside this scene
    Local,
    /// If scene is linked to another scene file
    External
};

class Scene
{
public:
    Scene(SceneID id, const String& name, const SceneSettings& settings, ImportStyle importStyle, const String& sceneFile);
    Scene();
    ~Scene();

    Entity CreateEntity(const String& name = String());
    void   DestroyEntity(Entity entity);

    void Update(Diligent::InputController& controller, IRenderDevice* pDevice, ISwapChain* pSwapChain, float dt);

    static void ResetNextID();

	static Ptr<Scene> CreateEmpty();
    static Ptr<Scene> Create();
    static Ptr<Scene> CreateRootScene();

	static Vector<Scene*> FindScenesByName(const String& name);
	static Scene* FindSceneByID(const SceneID& id);
	static const Vector<Scene*>& FindAllScenes();

	Scene* findScene(SceneID scene);
	void reimport();

	void onLoad();
	bool snatchChild(Scene* child);
	bool addChild(Ptr<Scene>& child);
	bool removeChild(Scene* toRemove);

	void setName(const String& name);

	Vector<Ptr<Scene>>& getChildren() { return m_ChildrenScenes; }
	SceneID getID() const { return m_ID; }
	ImportStyle getImportStyle() const { return m_ImportStyle; }
	String getScenePath() const { return m_SceneFile; }
	Scene* getParent() const { return m_ParentScene; }
	const String& getName() const { return m_Name; }
	const String& getFullName() const { return m_FullName; }
	SceneSettings& getSettings() { return m_Settings; }
    entt::registry& GetSceneRegistry() { return m_Registry; }

private:
    template <typename T>
    void OnComponentAdded(Entity entity, T& component);

private:
    entt::registry m_Registry;

    friend class Entity;

    static Vector<Scene*> s_Scenes;

    SceneID     m_ID;
    String      m_Name;
    String      m_FullName;
    ImportStyle m_ImportStyle;
    /// Contains the current file name if local, else contains the linked scene file
    String        m_SceneFile;
    SceneSettings m_Settings;

    Scene*             m_ParentScene = nullptr;
    Vector<Ptr<Scene>> m_ChildrenScenes;

    bool checkCycle(Scene* child);
};
} // namespace Vision