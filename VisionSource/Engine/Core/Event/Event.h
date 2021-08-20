
#pragma once

#include "../OS/OS.h"
#include "BasicMath.hpp"

#define DEFINE_EVENT(eventName, ...) const static inline Event::Type eventName = #eventName

namespace Vision
{

/// An Event that is sent out by EventManager.
class Event
{
public:
	/// String defining the type of the event.
	typedef String Type;

private:
	Type m_Type;
	Variant m_Data;

public:
	Event(const Type& type, const Variant& data);
	Event(Event&) = delete;
	~Event() = default;

	const Type& getType() const { return m_Type; };
	/// Returns the payload data sent with an event. Extract typed data after getting the data.
	const Variant& getData() const { return m_Data; }
};

struct VisionEvents
{
	/// Application exited the main loop
	DEFINE_EVENT(ApplicationExit);

	// Window Close Request
	DEFINE_EVENT(QuitWindowRequest);

    /// Window has resized to size passed in
	DEFINE_EVENT(WindowResized, Diligent::float2);

    /// Toggle the application window fullscreen mode
	DEFINE_EVENT(WindowToggleFullscreen);

    /// A string was printed to output
	DEFINE_EVENT(OSPrint, String);
};

}