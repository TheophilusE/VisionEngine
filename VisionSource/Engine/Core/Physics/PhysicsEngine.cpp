
#include "PhysicsEngine.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

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
    int i;
    for (i = m_DynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* collisionObject = m_DynamicsWorld->getCollisionObjectArray()[i];
        m_DynamicsWorld->removeCollisionObject(collisionObject);
    }
}

bool PhysicsEngine::Initialize(PhysicsInitInfo& InitInfo)
{
    Enabled           = InitInfo.Enabled;
    SimulationEnabled = InitInfo.SimulationEnabled;
    Accuracy          = InitInfo.Accuracy;
    Gravity           = InitInfo.Gravity;
    SubSteps          = InitInfo.SubSteps;

    m_CollisionConfiguration.reset(new btDefaultCollisionConfiguration());
    m_Dispatcher.reset(new btCollisionDispatcher(m_CollisionConfiguration.get()));
    m_Broadphase.reset(new btDbvtBroadphase());
    m_Solver.reset(new btSequentialImpulseConstraintSolver());
    m_GhostPairCallback.reset(new btGhostPairCallback());
    m_DynamicsWorld.reset(new btDiscreteDynamicsWorld(m_Dispatcher.get(), m_Broadphase.get(), m_Solver.get(), m_CollisionConfiguration.get()));
    m_PhysicsMaterialTable.resize(PhysicsMaterial::End);
    AssignPhysicsMaterials();

    if (!m_CollisionConfiguration || !m_Dispatcher || !m_Broadphase || !m_Solver || !m_DynamicsWorld || m_PhysicsMaterialTable.empty())
    {
        VISION_CORE_ERROR("Physics Engine Initialization Failed");
        return false;
    }

    m_DynamicsWorld->setInternalTickCallback(InternalTickCallback);
    m_DynamicsWorld->getPairCache()->setInternalGhostPairCallback(m_GhostPairCallback.get());
    m_DynamicsWorld->setWorldUserInfo(this);
    //m_DynamicsWorld->setDebugDrawer(&m_DebugDrawer);
}

// This function is called after bullet performs its internal update.
// To detect collisions between objects.
void PhysicsEngine::InternalTickCallback(btDynamicsWorld* const physicsWorld, btScalar const timeStep)
{
    PhysicsEngine* const physicsEngine = static_cast<PhysicsEngine*>(physicsWorld->getWorldUserInfo());

    // look at all existing contacts
    btDispatcher* const dispatcher = physicsWorld->getDispatcher();
    for (int manifoldIdx = 0; manifoldIdx < dispatcher->getNumManifolds(); ++manifoldIdx)
    {
        // get the "manifold", the set of data corresponding to a contact point
        // between two physics objects
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(manifoldIdx);

        // get the two bodies used in the manifold.
        btCollisionObject const* const body0 = static_cast<btCollisionObject const*>(manifold->getBody0());
        btCollisionObject const* const body1 = static_cast<btCollisionObject const*>(manifold->getBody1());

        //CollisionComponent* collider0 = (CollisionComponent*)body0->getUserPointer();
        //CollisionComponent* collider1 = (CollisionComponent*)body1->getUserPointer();

        //Entity& entity0 = collider0->getOwner();
        //Entity& entity1 = collider1->getOwner();

        //Hit h0 = Hit(&entity0, &entity1);
        //collider0->handleHit(&h0);
        //Hit h1 = Hit(&entity1, &entity0);
        //collider1->handleHit(&h1);
    }
}

void PhysicsEngine::Update(float TimeStep)
{
    m_DynamicsWorld->stepSimulation(TimeStep, SubSteps);
}

void PhysicsEngine::AddRigidBody(btRigidBody* rigidBody, int group, int mask)
{
    m_DynamicsWorld->addRigidBody(rigidBody, group, mask);
}

void PhysicsEngine::RemoveRigidBody(btRigidBody* rigidBody)
{
    m_DynamicsWorld->removeRigidBody(rigidBody);
}

void PhysicsEngine::AddCollisionObject(btCollisionObject* rigidBody, int group, int mask)
{
    m_DynamicsWorld->addCollisionObject(rigidBody, group, mask);
}

void PhysicsEngine::RemoveCollisionObject(btCollisionObject* collisionObject)
{
    m_DynamicsWorld->removeCollisionObject(collisionObject);
}

const PhysicsMaterialData& PhysicsEngine::GetMaterialData(PhysicsMaterial material)
{
    return m_PhysicsMaterialTable[material];
}

const char* PhysicsEngine::GetMaterialNames()
{
    return "Default\0Air\0Water\0Wood\0";
}

void PhysicsEngine::AssignPhysicsMaterials()
{
    m_PhysicsMaterialTable[PhysicsMaterial::Default] = {1.f, 1.f, 1.f};
    m_PhysicsMaterialTable[PhysicsMaterial::Air]     = {0.0f, 0.0f, 1.03e-3f};
    m_PhysicsMaterialTable[PhysicsMaterial::Water]   = {0.2f, 0.1f, 1.0f};
    m_PhysicsMaterialTable[PhysicsMaterial::Wood]    = {0.5f, 0.65f, 1.54f};
}

btCollisionWorld::AllHitsRayResultCallback PhysicsEngine::ReportAllRayHits(const btVector3& from, const btVector3& to)
{
    m_DynamicsWorld->updateAabbs();
    m_DynamicsWorld->computeOverlappingPairs();
    btCollisionWorld::AllHitsRayResultCallback allResults(from, to);
    allResults.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
    allResults.m_flags |= btTriangleRaycastCallback::kF_UseSubSimplexConvexCastRaytest;
    m_DynamicsWorld->rayTest(from, to, allResults);
    return allResults;
}

btCollisionWorld::ClosestRayResultCallback PhysicsEngine::ReportClosestRayHits(const btVector3& from, const btVector3& to)
{
    btCollisionWorld::ClosestRayResultCallback closestResults(from, to);
    closestResults.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;
    m_DynamicsWorld->rayTest(from, to, closestResults);
    return closestResults;
}

void PhysicsEngine::DebugDrawComponent(const btTransform& worldTransform, const btCollisionShape* shape, const btVector3& color)
{
    //m_DynamicsWorld->debugDrawObject(worldTransform, shape, color);
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