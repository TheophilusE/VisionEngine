#pragma once

#include "../Core/OS/OS.h"
#include "Scene.h"

namespace Vision
{
/// ECS style System interface that allows iterating over components directly.
class System
{
public:
	enum class UpdateOrder
	{
		Input,
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

	String m_SystemName;
	UpdateOrder m_UpdateOrder;
	bool m_IsActive;

public:
	static const Vector<Vector<System*>>& GetSystems() { return s_Systems; }

	System(const String& name, const UpdateOrder& order, bool isGameplay);
	System(System&) = delete;
	virtual ~System();

	virtual bool Initialize();
	virtual void setConfig(const SceneSettings& sceneSettings);

	virtual void Begin();
	virtual void Update(float deltaMilliseconds);
	virtual void End();

	String getName() const { return m_SystemName; }
	const UpdateOrder& getUpdateOrder() const { return m_UpdateOrder; }
	bool isActive() const { return m_IsActive; }

	void setActive(bool enabled);

	virtual void Render();
};
}