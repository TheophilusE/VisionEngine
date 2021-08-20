
#include "InputSystem.h"
#include "InputManager.h"

namespace Vision
{
InputSystem::InputSystem()
    : System("InputSystem", UpdateOrder::Input, true)
{
	m_Binder.bind(VisionEvents::WindowResized, this, &InputSystem::windowResized);
}

Variant InputSystem::windowResized(const Event* event)
{
	const Diligent::float2& newSize = Extract<Diligent::float2>(event->getData());
	InputManager::GetSingleton()->setDisplaySize(newSize);
	return true;
}

InputSystem* InputSystem::GetSingleton()
{
	static InputSystem singleton;
	return &singleton;
}

void InputSystem::loadSchemes(const HashMap<String, InputScheme>& schemes)
{
	InputManager::GetSingleton()->loadSchemes(schemes);
}

void InputSystem::addScheme(const String& name, const InputScheme& scheme)
{
	InputManager::GetSingleton()->addScheme(name, scheme);
}

void InputSystem::pushScheme(const String& name)
{
	InputManager::GetSingleton()->pushScheme(name);
}

void InputSystem::popScheme()
{
	InputManager::GetSingleton()->popScheme();
}

void InputSystem::flushSchemes()
{
	InputManager::GetSingleton()->flushSchemes();
}

bool InputSystem::Initialize()
{
	//InputManager::GetSingleton()->initialize(systemData["width"], systemData["height"]);
	return true;
}

void InputSystem::setConfig(const SceneSettings& sceneSettings)
{
	loadSchemes(sceneSettings.inputSchemes);
	pushScheme(sceneSettings.startScheme);
}

void InputSystem::Update(float deltaMilliseconds)
{
	InputManager::GetSingleton()->update();
}
}