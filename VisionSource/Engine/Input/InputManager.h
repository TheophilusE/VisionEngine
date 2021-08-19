
#pragma once

#include "../Core/OS/OS.h"
#include "BasicMath.hpp"
#include "InputListener.h"
#include "../Core/Event/Event.h"

namespace Vision
{

/// Alias for a keyboard device. Allows various keyboard specific operations.
typedef gainput::Key KeyboardButton;
/// Alias for a mouse device. Allows various mouse specific operations.
typedef gainput::MouseButton MouseButton;
/// Alias for a game controller/pad device. Allows various pad specific operations. Gainput (our input library) only supports pads that allow XInput. So non-XInput controllers are not supported.
typedef gainput::PadButton PadButton;
/// ID of any key, button or analog on the device hardware.
typedef gainput::DeviceButtonId DeviceButtonID;

/// Enumeration of the devices supported. Any new device can be added here.
enum class Device
{
	Mouse,
	Keyboard,
	/// Player 1 pad. Usually the first pad connected.
	Pad1,
	/// Player 2 pad. Usually the second pad connected.
	Pad2
};

struct InputDescription
{
	Device device;
	DeviceButtonID button;
	Event::Type inputEvent;
};

struct InputScheme
{
	Vector<InputDescription> bools;
	Vector<InputDescription> floats;
};

/// Allows interfacing to game controlling hardware, including mouse, keyboard and XInput controllers.
/// Allows detecting inputs through Event dispatch.
/// Event data for boolean buttons consists of a Vector2 where Vector2.x and Vector2.y carry the old and new values for the input event respectively.
/// Float buttons should be queried directly by invoking InputManager.
class InputManager
{
	/// Callback from Gainput's internals. Called when a key with bool value is activated.
	static bool BoolListen(int userButton, bool oldValue, bool newValue);
	/// Callback from Gainput's internals. Called when a key with float value is activated.
	static bool FloatListen(int userButton, float oldValue, float newValue);

	gainput::InputManager m_GainputManager;
	gainput::InputMap m_GainputMap;
	InputListener m_Listener;
	bool m_IsEnabled;
	HashMap<Device, unsigned int> DeviceIDs;
	HashMap<String, InputScheme> m_InputSchemes;
	Vector<String> m_CurrentSchemeStack;

	HashMap<unsigned int, Event::Type> m_InputEventIDNames;
	HashMap<Event::Type, unsigned int> m_InputEventNameIDs;

	unsigned int m_Width;
	unsigned int m_Height;

	InputManager();
	InputManager(InputManager&) = delete;
	~InputManager() = default;

	void forwardMessage(const MSG& msg);

	friend class Window;

	unsigned int getNextID(int device, int button);
	void buildBindings();

public:
	static InputManager* GetSingleton();
	static void SetEnabled(bool enabled) { GetSingleton()->setEnabled(enabled); };
	static void MapBool(const Event::Type& action, Device device, DeviceButtonID button) { GetSingleton()->mapBool(action, device, button); };
	static void MapFloat(const Event::Type& action, Device device, DeviceButtonID button) { GetSingleton()->mapBool(action, device, button); };
	static bool IsPressed(const Event::Type& action) { return GetSingleton()->isPressed(action); };
	static bool HasPressed(const Event::Type& action) { return GetSingleton()->hasPressed(action); };
	static bool WasPressed(const Event::Type& action) { return GetSingleton()->wasPressed(action); };
	static float GetFloat(const Event::Type& action) { return GetSingleton()->getFloat(action); };
	static float GetFloatDelta(const Event::Type& action) { return GetSingleton()->getFloatDelta(action); };
	static void Unmap(const Event::Type& action) { GetSingleton()->unmap(action); };
	static Diligent::float2 GetMousePosition() { return GetSingleton()->getMousePosition(); };

	void initialize(unsigned int width, unsigned int height);

	void setEnabled(bool enabled);

	void loadSchemes(const HashMap<String, InputScheme>& inputSchemes);
	void addScheme(const String& name, const InputScheme& inputScheme);
	void pushScheme(const String& schemeName);
	void popScheme();
	void flushSchemes();

	/// Bind an event to a button on a device.
	void mapBool(const Event::Type& action, Device device, DeviceButtonID button);
	/// Bind an event to a float on a device.
	void mapFloat(const Event::Type& action, Device device, DeviceButtonID button);

	void unmap(const Event::Type& action);

	bool isPressed(const Event::Type& action);
	bool hasPressed(const Event::Type& action);
	bool wasPressed(const Event::Type& action);
	float getFloat(const Event::Type& action);
	float getFloatDelta(const Event::Type& action);

	Diligent::float2 getMousePosition();

	void update();
	void setDisplaySize(const Diligent::float2& newSize);

	const gainput::InputMap& getMap() const { return m_GainputMap; }
	gainput::InputDeviceMouse* getMouse() { return static_cast<gainput::InputDeviceMouse*>(m_GainputManager.GetDevice(DeviceIDs[Device::Mouse])); }
	gainput::InputDeviceKeyboard* getKeyboard() { return static_cast<gainput::InputDeviceKeyboard*>(m_GainputManager.GetDevice(DeviceIDs[Device::Keyboard])); }
	gainput::InputDevicePad* getPad1() { return static_cast<gainput::InputDevicePad*>(m_GainputManager.GetDevice(DeviceIDs[Device::Pad1])); }
	gainput::InputDevicePad* getPad2() { return static_cast<gainput::InputDevicePad*>(m_GainputManager.GetDevice(DeviceIDs[Device::Pad2])); }
};

}