
#pragma once

#include "entt/entt.hpp"
#include "../Input/InputManager.h"

namespace Vision
{
class Entity;

struct SceneSettings
{
    HashMap<String, InputScheme> inputSchemes;
    String                       startScheme = {};
};

class Scene
{
public:
    Scene();
    ~Scene();

    Entity CreateEntity(const String& name = String());
    void   DestroyEntity(Entity entity);

    void Update(float dt);

private:
    template <typename T>
    void OnComponentAdded(Entity entity, T& component);

private:
    entt::registry m_Registry;

    friend class Entity;
};
} // namespace Vision