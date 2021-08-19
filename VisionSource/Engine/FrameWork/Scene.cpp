
#pragma once

#include "Scene.h"
#include "ECS.h"
#include "Components.h"

namespace Vision
{
Scene::Scene() {}
Scene::~Scene() {}

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
}

template <>
void Scene::OnComponentAdded<TransformComponent>(Entity entity, TransformComponent& component)
{
}

} // namespace Vision