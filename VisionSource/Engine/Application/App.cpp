
#include <random>
#include <vector>

#include "App.h"

namespace Vision
{
Application* CreateGLFWApp()
{
    return new App{};
}

bool App::Initialize()
{
    return true;
}

void App::Start()
{
    Application::Start();
}

void App::PreUpdate()
{
}

void App::FixedUpdate(float dt)
{
}

void App::Update(float dt)
{
    dt = std::min(dt, Constants.MaxDT);
}

void App::PostUpdate()
{
}

void App::Render()
{
    auto* pContext   = GetContext();
    auto* pSwapchain = GetSwapChain();

}

void App::KeyEvent(Key key, KeyState state)
{
    if (state == KeyState::Press || state == KeyState::Repeat)
    {
        /*
        switch (key)
        {
        }
        */
    }
}

void App::MouseEvent(float2 pos)
{
}


} // namespace Vision