
#include "System.h"

namespace Vision
{
Vector<Vector<System*>> System::s_Systems;

System::System(const String& name, const UpdateOrder& order, bool isGameplay)
    : m_SystemName(name)
    , m_UpdateOrder(order)
{
	if ((int)order >= s_Systems.size())
	{
		s_Systems.resize((int)order + 1, {});
	}

	s_Systems[(int)order].push_back(this);
	setActive(isGameplay);
}

System::~System()
{
	const int updateOrderInt = (int)m_UpdateOrder;
	auto& findIt = std::find(s_Systems[updateOrderInt].begin(), s_Systems[updateOrderInt].end(), this);
	if (findIt != s_Systems[updateOrderInt].end())
	{
		s_Systems[updateOrderInt].erase(findIt);
	}
}

bool System::Initialize()
{
	VISION_CORE_INFO("On demand initialization skipped for: " + m_SystemName);
	return true;
}

void System::setConfig(const SceneSettings& sceneSettings)
{
}

void System::Begin()
{
}

void System::Update(float deltaMilliseconds)
{
}

void System::End()
{
}

void System::setActive(bool enabled)
{
	m_IsActive = enabled;
}

void System::Render()
{

}
} // namespace Vision