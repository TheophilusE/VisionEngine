
#include "PhysicsEngine.h"

namespace Vision
{
PhysicsEngine* PhysicsEngine::GetSingleton()
{
    static PhysicsEngine singleton;
    return &singleton;
}

PhysicsEngine::PhysicsEngine()
{
    Enabled = true;
}

PhysicsEngine::~PhysicsEngine()
{
}

void PhysicsEngine::Initialize(PhysicsInitInfo& InitInfo)
{
    Enabled           = InitInfo.Enabled;
    SimulationEnabled = InitInfo.SimulationEnabled;
    Accuracy          = InitInfo.Accuracy;
    Gravity           = InitInfo.Gravity;
}

void PhysicsEngine::Update(float TimeStep)
{
}

void PhysicsEngine::SetAccuracy(int value)
{
    Accuracy = value;
    VISION_CORE_INFO("Physics Engine Accuracy Set: " + Accuracy);
}

int PhysicsEngine::GetAccuracy()
{
    return Accuracy;
}

void PhysicsEngine::SetEnabled(bool value)
{
    Enabled = value;
    VISION_CORE_INFO("Phyics Engine Enabled: " + Enabled);
}

bool PhysicsEngine::IsEnabled()
{
    return Enabled;
}

void PhysicsEngine::SetSimulationEnabled(bool value)
{
    SimulationEnabled = value;
    VISION_CORE_INFO("Phyics Engine Simulation Enabled: " + SimulationEnabled);
}

bool PhysicsEngine::IsSimulationEnabled()
{
    return SimulationEnabled;
}

} // namespace Vision