
#pragma once

#include "gainput/gainput.h"

#include <functional>
typedef std::function<bool(int userButton, bool oldValue, bool newValue)> InputBoolListenerFunction;
typedef std::function<bool(int userButton, float oldValue, float newValue)> InputFloatListenerFunction;

namespace Vision
{
    /// Helper class to receive inputs from Gainput.
class InputListener : public gainput::MappedInputListener
{
	InputBoolListenerFunction m_BoolListenerFunction;
	InputFloatListenerFunction m_FloatListenerFunction;
	int m_ID;

public:
	InputListener(InputBoolListenerFunction boolListener, InputFloatListenerFunction floatListener);
	InputListener(InputListener&) = delete;
	~InputListener() = default;

	virtual bool OnUserButtonBool(gainput::UserButtonId userButton, bool oldValue, bool newValue) override;
	virtual bool OnUserButtonFloat(gainput::UserButtonId userButton, float oldValue, float newValue) override;

	int getID() const { return m_ID; }
	void setID(int id) { m_ID = id; }
};
}