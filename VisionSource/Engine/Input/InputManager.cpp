
#include "InputManager.h"
#include "../Core/Event/EventManager.h"

#include <functional>

namespace Vision
{
void InputManager::Initialize(unsigned int width, unsigned int height)
{
	m_GainputManager.SetDisplaySize(width, height);
	m_Width = width;
	m_Height = height;

	DeviceIDs[Device::Mouse] = m_GainputManager.CreateDevice<gainput::InputDeviceMouse>();
	DeviceIDs[Device::Keyboard] = m_GainputManager.CreateDevice<gainput::InputDeviceKeyboard>();
	//DeviceIDs[Device::Pad1] = m_GainputManager.CreateDevice<gainput::InputDevicePad>();
	//DeviceIDs[Device::Pad2] = m_GainputManager.CreateDevice<gainput::InputDevicePad>();

	m_Listener.setID(1);
	setEnabled(true);
}

void InputManager::buildBindings()
{
	m_InputEventIDNames.clear();
	m_InputEventNameIDs.clear();
	m_GainputMap.Clear();

	// Condense the scheme stack to reduce redundant work
	Vector<String> newStack;
	for (auto& schemeName = m_CurrentSchemeStack.rbegin(); schemeName != m_CurrentSchemeStack.rend(); schemeName++)
	{
		if (std::find(newStack.begin(), newStack.end(), *schemeName) == newStack.end())
		{
			newStack.push_back(*schemeName);
		}
	}
	m_CurrentSchemeStack = Vector<String>(newStack.rbegin(), newStack.rend());

	for (auto& scheme : m_CurrentSchemeStack)
	{
		for (auto& boolBinding : m_InputSchemes.at(scheme).bools)
		{
			mapBool(boolBinding.inputEvent, boolBinding.device, boolBinding.button);
		}
		for (auto& floatBinding : m_InputSchemes.at(scheme).floats)
		{
			mapFloat(floatBinding.inputEvent, floatBinding.device, floatBinding.button);
		}
	}
}

void InputManager::setEnabled(bool enabled)
{
	if (enabled)
	{
		VISION_CORE_INFO("Input enabled");
		m_GainputMap.AddListener(&m_Listener);
		buildBindings();
	}
	else
	{
		VISION_CORE_INFO("Input disabled");
		m_GainputMap.Clear();
		m_GainputManager.RemoveListener(m_Listener.getID());
	}
	m_IsEnabled = enabled;
}

void InputManager::loadSchemes(const HashMap<String, InputScheme>& inputSchemes)
{
	int count = static_cast<int>(m_InputSchemes.size());
	for (auto& [name, scheme] : inputSchemes)
	{
		addScheme(name, scheme);
	}
	VISION_CORE_INFO("Registered " + std::to_string(inputSchemes.size()) + " more input schemes on top of " + std::to_string(count) + " existing ones");
}

void InputManager::addScheme(const String& name, const InputScheme& inputScheme)
{
	if (m_InputSchemes.find(name) != m_InputSchemes.end())
	{
		VISION_CORE_WARN("Overriding input scheme: " + name);
	}
	m_InputSchemes[name] = inputScheme;
}

void InputManager::popScheme()
{
	if (m_CurrentSchemeStack.empty())
	{
		VISION_CORE_WARN("Input scheme stack is empty. We don't pop anymore.");
		return;
	}

	String toPop = m_CurrentSchemeStack.back();
	m_CurrentSchemeStack.pop_back();
	buildBindings();

	VISION_CORE_INFO(toPop + " input scheme was popped");
}

void InputManager::flushSchemes()
{
	m_InputSchemes.clear();
	m_CurrentSchemeStack.clear();
	buildBindings();
}

void InputManager::pushScheme(const String& name)
{
	if (m_InputSchemes.find(name) == m_InputSchemes.end())
	{
		if (!name.empty())
		{
			VISION_CORE_WARN("Could not find a registered input scheme of name: " + name);
		}
		return;
	}

	m_CurrentSchemeStack.push_back(name);
	buildBindings();
	VISION_CORE_INFO("Pushed input scheme: " + name);
}

void InputManager::mapBool(const Event::Type& action, Device device, DeviceButtonID button)
{
	m_InputEventNameIDs[action] = getNextID((int)device, (int)button);
	m_InputEventIDNames[m_InputEventNameIDs[action]] = action;
	if (!m_GainputMap.MapBool((gainput::UserButtonId)m_InputEventNameIDs[action], DeviceIDs[device], button))
	{
		VISION_CORE_WARN("Bool mapping could not done: " + action);
	}
}

void InputManager::mapFloat(const Event::Type& action, Device device, DeviceButtonID button)
{
	m_InputEventNameIDs[action] = getNextID((int)device, (int)button);
	m_InputEventIDNames[m_InputEventNameIDs[action]] = action;
	if (!m_GainputMap.MapFloat((gainput::UserButtonId)m_InputEventNameIDs[action], DeviceIDs[device], button))
	{
		VISION_CORE_WARN("Float mapping could not done: " + action);
	}
}

void InputManager::unmap(const Event::Type& action)
{
	m_GainputMap.Unmap(m_InputEventNameIDs[action]);
}

bool InputManager::isPressed(const Event::Type& action)
{
	return m_IsEnabled && m_GainputMap.GetBool((gainput::UserButtonId)m_InputEventNameIDs.at(action));
}

bool InputManager::wasPressed(const Event::Type& action)
{
	return m_IsEnabled && m_GainputMap.GetBoolWasDown((gainput::UserButtonId)m_InputEventNameIDs.at(action));
}

bool InputManager::hasPressed(const Event::Type& action)
{
	return m_IsEnabled && m_GainputMap.GetBoolIsNew((gainput::UserButtonId)m_InputEventNameIDs.at(action));
}

float InputManager::getFloat(const Event::Type& action)
{
	if (m_IsEnabled)
	{
		return m_GainputMap.GetFloat((gainput::UserButtonId)m_InputEventNameIDs.at(action));
	}
	return 0;
}

float InputManager::getFloatDelta(const Event::Type& action)
{
	if (m_IsEnabled)
	{
		return m_GainputMap.GetFloatDelta((gainput::UserButtonId)m_InputEventNameIDs.at(action));
	}
	return 0;
}

Diligent::float2 InputManager::getMousePosition()
{
	return { getMouse()->GetFloat(MouseButton::MouseAxisX), getMouse()->GetFloat(MouseButton::MouseAxisY) };
}

void InputManager::update()
{
	m_GainputManager.Update();
}

void InputManager::setDisplaySize(const Diligent::float2& newSize)
{
	m_GainputManager.SetDisplaySize(static_cast<int>(newSize.x), static_cast<int>(newSize.y));
}

unsigned int InputManager::getNextID(int device, int button)
{
	static unsigned int count = 0;
	static HashMap<int, HashMap<int, int>> cache;

	if (cache.find(device) != cache.end())
	{
		if (cache.at(device).find(button) != cache.at(device).end())
		{
			return cache.at(device).at(button);
		}
	}

	cache[device][button] = count;
	return count++;
}

InputManager* InputManager::GetSingleton()
{
	static InputManager singleton;
	return &singleton;
}

bool InputManager::BoolListen(int userButton, bool oldValue, bool newValue)
{
	EventManager::GetSingleton()->call(GetSingleton()->m_InputEventIDNames.at(userButton), Diligent::float2{float(oldValue), float(newValue)});
	return true;
}

bool InputManager::FloatListen(int userButton, float oldValue, float newValue)
{
	EventManager::GetSingleton()->call(GetSingleton()->m_InputEventIDNames.at(userButton), Diligent::float2{oldValue, newValue});
	return true;
}

InputManager::InputManager()
    : m_GainputMap(m_GainputManager)
    , m_Listener(BoolListen, FloatListen)
    , m_Width(0)
    , m_Height(0)
{
}

void InputManager::forwardMessage(const MSG& msg)
{
	m_GainputManager.HandleMessage(msg);
}

}