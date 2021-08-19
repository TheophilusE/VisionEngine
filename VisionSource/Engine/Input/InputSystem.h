
#pragma once

#include "../FrameWork/System.h"
#include "../Core/Event/EventManager.h"

namespace Vision
{
class InputSystem : public System
{
    EventBinder<InputSystem> m_Binder;

    InputSystem();
    InputSystem(InputSystem&) = delete;
    ~InputSystem()            = default;

    Variant windowResized(const Event* event);

public:
    static InputSystem* GetSingleton();

    void loadSchemes(const HashMap<String, InputScheme>& schemes);
    void addScheme(const String& name, const InputScheme& scheme);
    void pushScheme(const String& name);
    void popScheme();
    void flushSchemes();

    bool Initialize() override;
    void setConfig(const SceneSettings& sceneSettings) override;

    virtual void Start() override {};
    virtual void PreUpdate() override {};
    virtual void FixedUpdate(float dt) override {};
    virtual void Update(float deltaMilliseconds) override;
    virtual void PostUpdate() override {};
    virtual void End() override {};
};
} // namespace Vision