
#pragma once

#include "Application.h"

namespace Vision
{
class App : public Application
{
public:
    virtual bool Initialize() override;
    virtual void Start() override;
    virtual void PreUpdate() override;
    virtual void FixedUpdate(float dt) override;
    virtual void Update(float dt) override;
    virtual void PostUpdate() override;
    virtual void Render() override;
    virtual void KeyEvent(Key key, KeyState state) override;
    virtual void MouseEvent(float2 pos) override;

    virtual const Char* GetSampleName() const override { return "Vision Engine Editor"; }

protected:
    const float ClearColor[4] = {};
    bool        m_ShowDemoWindow = true;
    struct
    {
        const float MaxDT = 1.0f / 60.0f;

    } Constants;

    
    const HashMap<String, InputScheme> m_InputScheme;
};
} // namespace Vision