
#pragma once

//#include <pal/pal.h>
//#include <pal/palFactory.h>
//#include <pal/palActivation.h>

#include "../OS/OS.h"
#include "../Math/VMath.h"

#define PHYSICS_ENGINE_BULLET 1
#define PHYSICS_ENGINE_PHYSX 1

namespace Vision
{
struct PhysicsInitInfo
{
    bool Enabled           = true;
    bool SimulationEnabled = true;
    int  Accuracy          = 10;
    int  SubSteps          = 1;
    Vector3f Gravity       = Vector3f(0.0f, -9.81f, 0.0f);
};

enum PhysicsMaterial
{
	Default = 0,
	End
};

struct PhysicsMaterialData
{
	float Restitution = 1.0f;
	float Friction = 1.0f;
	float GravityScale = 1.0f;
};

class PhysicsEngine
{
public:
    static PhysicsEngine* GetSingleton();

    void Initialize(PhysicsInitInfo& InitInfo);
    void Update(float TimeStep);

    // Set the accuracy of the simulation
    //	This value corresponds to maximum simulation step count
    //	Higher values will be slower but more accurate
    //	Default is 10
    void SetAccuracy(int value);
    int  GetAccuracy();

    // Enable/disable the physics engine all together
    void SetEnabled(bool value);
    bool IsEnabled();

    // Enable/disable the physics simulation.
    //	Physics engine state will be updated but not simulated
    void SetSimulationEnabled(bool value);
    bool IsSimulationEnabled();

protected:
    /** Construction, destruction, and Cleanup should only be done by
	 * the Class. */
    PhysicsEngine();
    virtual ~PhysicsEngine();

    bool Enabled           = true;
    bool SimulationEnabled = true;
    int  Accuracy          = 10;
    int  SubSteps          = 1;

    Vector3f Gravity = Vector3f(0.0f, -9.81f, 0.0f);
};

} // namespace Vision