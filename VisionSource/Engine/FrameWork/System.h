#pragma once

#include "../Core/OS/OS.h"

/// ECS style System interface that allows iterating over components directly.
class System
{
public:
    enum class UpdateOrder
    {
        Input,
        PreUpdate,
        FixedUpdate,
        Update,
        PostUpdate,
        Render,
        PostRender,
        RenderUI,
        UI,
        GameRender,
        Editor,
        Async,
        End
    };

protected:
    static Vector<Vector<System*>> s_Systems;

    friend class Entity;

    String      m_SystemName;
    UpdateOrder m_UpdateOrder;
    bool        m_IsActive;

public:
    static const Vector<Vector<System*>>& GetSystems() { return s_Systems; }

    System(const String& name, const UpdateOrder& order, bool isGameplay);
    System(System&) = delete;
    virtual ~System();

    virtual bool Initialize();
    virtual void setConfig(const SceneSettings& sceneSettings);

    virtual void Start()                              = 0;
    virtual void PreUpdate()                          = 0;
    virtual void FixedUpdate(float dt = 1.0f / 60.0f) = 0;
    virtual void Update(float deltaMilliseconds)      = 0;
    virtual void PostUpdate()                         = 0;
    virtual void End()                                = 0;

    String             getName() const { return m_SystemName; }
    const UpdateOrder& getUpdateOrder() const { return m_UpdateOrder; }
    bool               isActive() const { return m_IsActive; }

    void setActive(bool enabled);

    virtual void Render();
};