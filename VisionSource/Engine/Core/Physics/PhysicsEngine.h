
#pragma once

//#include <pal/pal.h>
//#include <pal/palFactory.h>
//#include <pal/palActivation.h>

#include "../OS/OS.h"
#include "../Math/VMath.h"


#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"

#define PHYSICS_ENGINE_BULLET 1
#define PHYSICS_ENGINE_PHYSX 1

namespace Vision
{

struct PhysicsInitInfo
{
    bool Enabled           = true;
    bool SimulationEnabled = true;
    int  Accuracy          = 10;
    int  SubSteps          = 10;
    Vector3f Gravity       = Vector3f(0.0f, -9.81f, 0.0f);
};

enum PhysicsMaterial
{
	Default = 0,
    Air     = 1,
    Water   = 2,
    Wood    = 3,
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

    /// Callback from bullet for each physics time step.
    static void InternalTickCallback(btDynamicsWorld* const physicsWorld, btScalar const timeStep);

    bool Initialize(PhysicsInitInfo& InitInfo);
    void Update(float TimeStep);

    void AddRigidBody(btRigidBody* rigidBody, int group, int mask);
    void RemoveRigidBody(btRigidBody* rigidBody);

    void AddCollisionObject(btCollisionObject* collisionObject, int group, int mask);
    void RemoveCollisionObject(btCollisionObject* collisionObject);

    const PhysicsMaterialData& GetMaterialData(PhysicsMaterial material);
    const char*                GetMaterialNames();

    btCollisionWorld::AllHitsRayResultCallback ReportAllRayHits(const btVector3& m_From, const btVector3& m_To);
    btCollisionWorld::ClosestRayResultCallback ReportClosestRayHits(const btVector3& m_From, const btVector3& m_To);

    void DebugDrawComponent(const btTransform& worldTransform, const btCollisionShape* shape, const btVector3& color);

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

    void AssignPhysicsMaterials();

    bool Enabled           = true;
    bool SimulationEnabled = true;
    int  Accuracy          = 10;
    int  SubSteps          = 10;

    // Interface for several dynamics implementations, basic, discrete, parallel, and continuous etc.
    Ptr<btDynamicsWorld> m_DynamicsWorld;

    // An interface to detect aabb-overlapping object pairs.
    Ptr<btBroadphaseInterface> m_Broadphase;

    // btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
    // Time of Impact, Closest Points and Penetration Depth.
    Ptr<btCollisionDispatcher> m_Dispatcher;

    // Provides solver interface.
    Ptr<btConstraintSolver> m_Solver;

    // Allows to configure Bullet collision detection.
    Ptr<btDefaultCollisionConfiguration> m_CollisionConfiguration;

    // Pair callback so ghost objects can cache their collision pairs
    Ptr<btGhostPairCallback> m_GhostPairCallback;


    Vector<PhysicsMaterialData> m_PhysicsMaterialTable;
    String                      m_PhysicsMaterialNames;

    Vector3f Gravity = Vector3f(0.0f, -9.81f, 0.0f);
};

} // namespace Vision