//#define USE_LISTEN_COLLISION

#include <limits>
#include <cfloat>
#include "bullet_pal.h"
#include "bullet_palVehicle.h"
#include "bullet_palCharacter.h"
#include "bullet_palLinks.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btIDebugDraw.h"
//#include <iostream>

#include <pal/pal.inl>

#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"

#include "LinearMath/btConvexHull.h"
#include <set>
#ifdef INTERNAL_DEBUG
#include <iostream>
#endif


#if BT_BULLET_VERSION >= 283 && defined(BULLET_MULTITHREADING)
#include "bullet_multithreaded.h"
#endif

FACTORY_CLASS_IMPLEMENTATION_BEGIN_GROUP;
FACTORY_CLASS_IMPLEMENTATION(palBulletPhysics);

FACTORY_CLASS_IMPLEMENTATION(palBulletBoxGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletSphereGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletCapsuleGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletCylinderGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletConvexGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletConcaveGeometry);
FACTORY_CLASS_IMPLEMENTATION(palBulletCustomConcaveGeometry);

FACTORY_CLASS_IMPLEMENTATION(palBulletGenericBody);

FACTORY_CLASS_IMPLEMENTATION(palBulletOrientatedTerrainPlane);
FACTORY_CLASS_IMPLEMENTATION(palBulletTerrainPlane);
FACTORY_CLASS_IMPLEMENTATION(palBulletTerrainMesh);
FACTORY_CLASS_IMPLEMENTATION(palBulletTerrainHeightmap);

FACTORY_CLASS_IMPLEMENTATION(palBulletSphericalLink);
FACTORY_CLASS_IMPLEMENTATION(palBulletRevoluteLink);
FACTORY_CLASS_IMPLEMENTATION(palBulletRevoluteSpringLink);
FACTORY_CLASS_IMPLEMENTATION(palBulletPrismaticLink);
FACTORY_CLASS_IMPLEMENTATION(palBulletGenericLink);
FACTORY_CLASS_IMPLEMENTATION(palBulletRigidLink);

FACTORY_CLASS_IMPLEMENTATION(palBulletVehicle);

FACTORY_CLASS_IMPLEMENTATION(palBulletCharacterController);

FACTORY_CLASS_IMPLEMENTATION(palBulletPSDSensor);

FACTORY_CLASS_IMPLEMENTATION(palBulletMotor);
FACTORY_CLASS_IMPLEMENTATION(palBulletGenericLinkSpring);

FACTORY_CLASS_IMPLEMENTATION(palBulletPatchSoftBody);
FACTORY_CLASS_IMPLEMENTATION(palBulletTetrahedralSoftBody);

FACTORY_CLASS_IMPLEMENTATION_END_GROUP;

class palBulletDebugDraw : public btIDebugDraw
{
public:
	palBulletDebugDraw() : m_pPalDebugDraw(0) {}

	bool inRange(const btVector3& point) {
		if (m_pPalDebugDraw == NULL)
			return false;

		if (m_pPalDebugDraw->GetRange() > 0.0f) {
			palVector3 difference(m_pPalDebugDraw->m_vRefPoint.x - point.x(), m_pPalDebugDraw->m_vRefPoint.y - point.y(),m_pPalDebugDraw-> m_vRefPoint.z - point.z());
			return m_pPalDebugDraw->GetRange2() >
			vec_mag2(&difference);
		}
		return true;
	}

	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
		if (inRange(from)) {
			m_pPalDebugDraw->m_Lines.m_vVertices.push_back(palVector3(from.x(),
					from.y(), from.z()));
			m_pPalDebugDraw->m_Lines.m_vVertices.push_back(palVector3(to.x(),
					to.y(), to.z()));
			for (unsigned i = 0; i < 2; ++i) {
				m_pPalDebugDraw->m_Lines.m_vColors.push_back(palVector4(
						color.x(), color.y(), color.z(), 1.0f));
			}
		}
	}

	virtual void	drawBox (const btVector3& boxMin, const btVector3& boxMax, const btVector3& color, btScalar alpha)
	{
	}

	virtual void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
	{
		drawSpherePatch(p, btVector3(0.0, 0.0, 1.0), btVector3(0.0, 1.0, 0.0), radius, 0.0, SIMD_2_PI, 0.0, SIMD_2_PI, color, 3.0f);
	}


	virtual void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& /*n0*/,const btVector3& /*n1*/,const btVector3& /*n2*/,const btVector3& color, btScalar alpha) {
		drawTriangle(v0,v1,v2,color,alpha);
	}

	virtual void drawTriangle(const btVector3& v0, const btVector3& v1,
			const btVector3& v2, const btVector3& color, btScalar alpha) {
		if (inRange(v1)) {
			m_pPalDebugDraw->m_Triangles.m_vVertices.push_back(palVector3(
					v0.x(), v0.y(), v0.z()));
			m_pPalDebugDraw->m_Triangles.m_vVertices.push_back(palVector3(
					v1.x(), v1.y(), v1.z()));
			m_pPalDebugDraw->m_Triangles.m_vVertices.push_back(palVector3(
					v2.x(), v2.y(), v2.z()));
			for (unsigned i = 0; i < 3; ++i) {
				m_pPalDebugDraw->m_Triangles.m_vColors.push_back(palVector4(
						color.x(), color.y(), color.z(), alpha));
			}
		}
	}

	virtual void drawContactPoint(const btVector3& PointOnB,
			const btVector3& normalOnB, btScalar distance, int lifeTime,
			const btVector3& color) {
		if (inRange(PointOnB)) {
			m_pPalDebugDraw->m_Points.m_vVertices.push_back(palVector3(
					PointOnB.x(), PointOnB.y(), PointOnB.z()));
			m_pPalDebugDraw->m_Points.m_vColors.push_back(palVector4(color.x(),
					color.y(), color.z(), 1.0f));
		}
	}

	virtual void reportErrorWarning(const char* warningString) {
	}

	virtual void draw3dText(const btVector3& location, const char* textString) {
		if (inRange(location)) {
			palDebugText debugText;
			for (unsigned i = 0; i < 3; ++i) {
				debugText.m_vPos._vec[i] = location[i];
			}
			debugText.text = textString;
			m_pPalDebugDraw->m_vTextItems.push_back(debugText);
		}
	}

	virtual void setDebugMode(int debugMode) {}

	virtual int getDebugMode() const { return DBG_DrawFeaturesText |
			DBG_DrawContactPoints |
			DBG_DrawText |
			DBG_DrawWireframe |
			//DBG_ProfileTimings |
			//DBG_EnableSatComparison |
			//DBG_DisableBulletLCP |
			//DBG_EnableCCD |
			DBG_DrawConstraints |
			DBG_FastWireframe |
			DBG_DrawConstraintLimits
			; }

	void SetPalDebugDraw(palDebugDraw *newDebugDraw) { m_pPalDebugDraw = newDebugDraw; }
	palDebugDraw *GetPalDebugDraw() { return m_pPalDebugDraw; }
private:
	palDebugDraw *m_pPalDebugDraw;
};

static bool g_bEnableCustomMaterials = false;

#if BT_BULLET_VERSION < 280
static bool CustomMaterialCombinerCallback(btManifoldPoint& mp, const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
#else
static bool CustomMaterialCombinerCallback(btManifoldPoint& mp, const btCollisionObjectWrapper* colObj0,int partId0,int index0,const btCollisionObjectWrapper* colObj1,int partId1,int index1)
#endif
{
	if (g_bEnableCustomMaterials && colObj1->getCollisionObject()->getCollisionShape()->getShapeType() != CUSTOM_CONCAVE_SHAPE_TYPE)
	{
		btAdjustInternalEdgeContacts(mp, colObj1, colObj0, partId1, index1);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		//btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	}

#if BT_BULLET_VERSION < 280
	palBodyBase* body0 = static_cast<palBodyBase*>(colObj0->getUserPointer());
	palBodyBase* body1 = static_cast<palBodyBase*>(colObj1->getUserPointer());
#else
	palBodyBase* body0 = static_cast<palBodyBase*>(colObj0->getCollisionObject()->getUserPointer());
	palBodyBase* body1 = static_cast<palBodyBase*>(colObj1->getCollisionObject()->getUserPointer());
#endif
	if (body0 != NULL && body1 != NULL)
	{
		palMaterial* mat0 = body0->GetMaterial();
		palMaterial* mat1 = body1->GetMaterial();
		palMaterialDesc matResult;
		matResult.m_fStatic = mp.m_combinedFriction;
#if BT_BULLET_VERSION >= 280
		matResult.m_fKinetic = mp.m_combinedRollingFriction;
#endif
		matResult.m_fRestitution = mp.m_combinedRestitution;
		palContactPoint contactResult;
		convertManifoldPtToContactPoint(mp, contactResult);
		contactResult.m_pBody1 = body0;
		contactResult.m_pBody2 = body1;
		if (static_cast<palPhysics*>(body0->GetParent())->GetMaterials()->HandleCustomInteraction(mat0, mat1, matResult, contactResult, true))
		{
			convertContactPointToManifoldPt(contactResult, mp);
			mp.m_combinedFriction = matResult.m_fStatic;
			mp.m_combinedRestitution = matResult.m_fRestitution;
#if BT_BULLET_VERSION >= 280
			mp.m_combinedRollingFriction = matResult.m_fKinetic;
#endif
		}
	}
	//if (cp.m_contactCFM1)
	return true;
}

struct CustomOverlapFilterCallback: public btOverlapFilterCallback
{
	CustomOverlapFilterCallback(palBulletPhysics* physicInst)
	: m_pPhysics(physicInst)
	{
	}

	virtual ~CustomOverlapFilterCallback()
	{}
	// return true when pairs need collision
	virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const {

		short proxy0GroupBits = proxy0->m_collisionFilterGroup;
		short proxy1GroupBits = proxy1->m_collisionFilterGroup;

		palGroup p0Group = convert_to_pal_group(proxy0GroupBits);
		palGroup p1Group = convert_to_pal_group(proxy1GroupBits);

		// if it hasn't been set, then default to colliding.
		size_t maskVectorSize = m_pPhysics->m_CollisionMasks.size();
		if (maskVectorSize <= size_t(p0Group)
				|| maskVectorSize <= size_t(p1Group)) {
			return true;
		}

		short proxy0Mask = m_pPhysics->m_CollisionMasks[p0Group];
		short proxy1Mask = m_pPhysics->m_CollisionMasks[p1Group];

		proxy0->m_collisionFilterMask = proxy0Mask;
		proxy1->m_collisionFilterMask = proxy1Mask;

		bool collides = ((proxy0GroupBits & proxy1Mask) != 0) && ((proxy1GroupBits & proxy0Mask) != 0);
		return collides;
	}
	palBulletPhysics* m_pPhysics;
};

////////////////////////////////////////////////////
static void AddMeshToTrimesh(btTriangleIndexVertexArray *trimesh, const Float *pVertices, int nVertices, const int *pIndices, int nIndices)
{
	btIndexedMesh meshIndex;
	meshIndex.m_numTriangles = nIndices / 3;
	meshIndex.m_numVertices = nVertices;
	meshIndex.m_vertexStride = 3 * sizeof (Float);
	meshIndex.m_triangleIndexStride = 3 * sizeof (int);
	meshIndex.m_indexType = PHY_INTEGER;

	meshIndex.m_triangleIndexBase = reinterpret_cast<const unsigned char*>(pIndices);
	meshIndex.m_vertexBase = reinterpret_cast<const unsigned char*>(pVertices);
#ifdef DOUBLE_PRECISION
	meshIndex.m_vertexType = PHY_DOUBLE;
#else
	meshIndex.m_vertexType = PHY_FLOAT;
#endif

	trimesh->addIndexedMesh(meshIndex);
}

////////////////////////////////////////////////////
class palBulletAction : public btActionInterface {
public:
	palBulletAction(palAction& action)
	: mAction(action)
	{
	}

	virtual ~palBulletAction() {}

	// Need to call and pass the pal debug drawer to the action.
	virtual void debugDraw(btIDebugDraw *debugDrawer) {}
private:
	virtual void updateAction(btCollisionWorld *collisionWorld, btScalar deltaTimeStep)
	{
		mAction(deltaTimeStep);
	}

	palAction& mAction;
};

////////////////////////////////////////////////////
void palBulletPhysics::SetGroupCollision(palGroup a, palGroup b, bool enabled) {
	short bits = convert_group(a);
	short other_bits = convert_group(b);

	if (m_CollisionMasks.size() <= size_t(std::max(a, b)))
	{
		m_CollisionMasks.resize(std::max(a,b)+1, ~0);
	}

	if (enabled) {
		m_CollisionMasks[a] = m_CollisionMasks[a] | other_bits;
		m_CollisionMasks[b] = m_CollisionMasks[b] | bits;
	} else {
		m_CollisionMasks[a] = m_CollisionMasks[a] & ~other_bits;
		m_CollisionMasks[b] = m_CollisionMasks[b] & ~bits;
	}

	class	CleanPairForGroupCallback : public btOverlapCallback
	{
		short m_groupBitsA, m_groupBitsB;
	public:
		CleanPairForGroupCallback(short groupBitsA, short groupBitsB)
	: m_groupBitsA(groupBitsA)
	, m_groupBitsB(groupBitsB)
	{
	}

		virtual	bool	processOverlap(btBroadphasePair& pair)
		{
			return (
					(pair.m_pProxy0->m_collisionFilterGroup == m_groupBitsA) ||
					(pair.m_pProxy1->m_collisionFilterGroup == m_groupBitsA) ||
					(pair.m_pProxy0->m_collisionFilterGroup == m_groupBitsB) ||
					(pair.m_pProxy1->m_collisionFilterGroup == m_groupBitsB)
			);
		}

	};

	CleanPairForGroupCallback callback(a, b);

	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->processAllOverlappingPairs(&callback,
			m_dynamicsWorld->getDispatcher());

}

void palBulletPhysics::SetCollisionAccuracy(Float fAccuracy) {
	;//
}

void palBulletPhysics::RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range, palRayHit& hit) const {

	btVector3 from(x,y,z);
	btVector3 dir(dx,dy,dz);
	btVector3 to = from + dir * range;
	btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

	rayCallback.m_collisionFilterGroup = ~0;
	rayCallback.m_collisionFilterMask = ~0;

	m_dynamicsWorld->rayTest(from, to, rayCallback);
	if (rayCallback.hasHit())
	{
		hit.Clear();
		hit.SetHitPosition(rayCallback.m_hitPointWorld.x(),rayCallback.m_hitPointWorld.y(),rayCallback.m_hitPointWorld.z());
		hit.SetHitNormal(rayCallback.m_hitNormalWorld.x(),rayCallback.m_hitNormalWorld.y(),rayCallback.m_hitNormalWorld.z());
		hit.m_bHit = true;
		hit.m_fDistance = range*rayCallback.m_closestHitFraction;

		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
		if (body)
		{
			hit.m_pBody = static_cast<palBodyBase *>(body->getUserPointer());
		}
	}
}

struct palBulletCustomResultCallback : public btCollisionWorld::RayResultCallback
{
	palBulletCustomResultCallback(const btVector3& rayFromWorld,const btVector3& rayToWorld, btScalar range,
			palRayHitCallback& callback, palGroupFlags groupFilter)
			: m_rayFromWorld(rayFromWorld)
			  , m_rayToWorld(rayToWorld)
			  , m_range(range)
			  , m_callback(callback)
			  , m_groupFilter(groupFilter)
			  , m_lastFraction(1.0)
			  {
		m_collisionFilterGroup = ~0;
		m_collisionFilterMask = (short) groupFilter;
			  }

	btVector3       m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3       m_rayToWorld;
	btScalar        m_range;
	palRayHitCallback& m_callback;
	palGroupFlags   m_groupFilter;
	btScalar        m_lastFraction;

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		//m_collisionObject = rayResult.m_collisionObject;

		btVector3 hitNormalWorld, hitPointWorld;
		if (normalInWorldSpace) {
			hitNormalWorld = rayResult.m_hitNormalLocal;
		}
		else {
			///need to transform normal into worldspace
			hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}

		hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);

		const btRigidBody* body = btRigidBody::upcast(rayResult.m_collisionObject);

		palRayHit hit;
		hit.Clear();
		hit.SetHitPosition(hitPointWorld.x(), hitPointWorld.y(), hitPointWorld.z());
		hit.SetHitNormal(hitNormalWorld.x(), hitNormalWorld.y(), hitNormalWorld.z());
		hit.m_bHit = true;
		hit.m_fDistance = m_range * rayResult.m_hitFraction;

		if (body != NULL) {
			hit.m_pBody = static_cast<palBodyBase *>(body->getUserPointer());
		}

		m_lastFraction = m_callback.AddHit(hit) / m_range;
		if (m_lastFraction < m_closestHitFraction)
		{
			m_closestHitFraction = rayResult.m_hitFraction;
		}

		return m_lastFraction;
	}
};

void palBulletPhysics::RayCast(Float x, Float y, Float z,
		Float dx, Float dy, Float dz, Float range,
		palRayHitCallback& callback, palGroupFlags groupFilter) const {
	btVector3 from(x,y,z);
	btVector3 dir(dx,dy,dz);
	btVector3 to = from + dir * range;

	palBulletCustomResultCallback rayCallback(from, to, range, callback, groupFilter);

	m_dynamicsWorld->rayTest(from, to, rayCallback);
}

void palBulletPhysics::AddRigidBody(palBulletBodyBase* body) {
	if (body != nullptr && body->m_pbtBody != nullptr) {
		//reset the group to get rid of the default groups.
		palGroup group = body->GetGroup();
		m_dynamicsWorld->addRigidBody(body->m_pbtBody, convert_group(group), m_CollisionMasks[group]);
	}
}

void palBulletPhysics::RemoveRigidBody(palBulletBodyBase* body) {
	if (body != nullptr && body->m_pbtBody != nullptr) {
		m_dynamicsWorld->removeRigidBody(body->m_pbtBody);
	}
}

void palBulletPhysics::ClearBroadPhaseCachePairs(palBulletBodyBase *body) {
	btBroadphaseProxy *proxy = body->BulletGetRigidBody()->getBroadphaseProxy();
	if (proxy != nullptr) {
		proxy->m_collisionFilterMask = m_CollisionMasks[body->GetGroup()];
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(proxy,
				m_dynamicsWorld->getDispatcher());
	}
}

void palBulletPhysics::AddBulletConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies)
{
	if (constraint != NULL) {
		m_dynamicsWorld->addConstraint(constraint, disableCollisionsBetweenLinkedBodies);
	}
}

void palBulletPhysics::RemoveBulletConstraint(btTypedConstraint* constraint)
{
	if (constraint != NULL && constraint->getUserConstraintId() != INT_MIN) {
		m_dynamicsWorld->removeConstraint(constraint);
		constraint->setUserConstraintId(INT_MIN);
	}
}

void palBulletPhysics::AddAction(palAction *action) {
	if (action != NULL) {
		palBulletAction* bulletAction = new palBulletAction(*action);
		m_BulletActions[action] = bulletAction;
		m_dynamicsWorld->addAction(bulletAction);
	}
}

void palBulletPhysics::RemoveAction(palAction *action) {
	if (action != NULL) {
		PAL_MAP<palAction*, btActionInterface*>::iterator item = m_BulletActions.find(action);
		if (item != m_BulletActions.end()) {
			btActionInterface* bulletAction = item->second;
			if (bulletAction != NULL) {
				m_dynamicsWorld->removeAction(bulletAction);
				delete bulletAction;
				bulletAction = NULL;
			}
			m_BulletActions.erase(item);
		}
	}
}

void palBulletPhysics::CallActions(Float timestep) {
	// Do nothing here.  The dynamics world does this stuff.
}


typedef PAL_MULTIMAP <palBodyBase*, palBodyBase*> ListenMap;
typedef ListenMap::iterator ListenIterator;
typedef ListenMap::const_iterator ListenConstIterator;
ListenMap pallisten;

#ifdef USE_LISTEN_COLLISION
static bool listenCollision(palBodyBase *body1, palBodyBase *body2) {
	ListenConstIterator itr;

	// The greater one is the key, which also works for NULL.
	palBodyBase* b0 = body1 > body2 ? body1: body2;
	palBodyBase* b1 = body1 < body2 ? body1: body2;

	std::pair<ListenIterator, ListenIterator> range = pallisten.equal_range(b0);
	for (ListenIterator i = range.first; i != range.second; ++i) {
		if (i->second ==  b1 || i->second == NULL) {
			return true;
		}
	}
	return false;
}
#endif

void palBulletPhysics::NotifyCollision(palBodyBase *body1, palBodyBase *body2, bool enabled) {

	bool found = false;
	std::pair<ListenIterator, ListenIterator> range;

	// The greater one is the key, which also works for NULL.
	palBodyBase* b0 = body1 > body2 ? body1: body2;
	palBodyBase* b1 = body1 < body2 ? body1: body2;

	if (b0 != NULL) {
		range = pallisten.equal_range(b0);

		for (ListenIterator i = range.first; i != range.second; ++i) {
			if (i->second ==  b1) {
				if (enabled) {
					found = true;
				} else {
					pallisten.erase(i);
				}
				break;
			}
		}

		if (!found && enabled) {
			pallisten.insert(range.second, std::make_pair(b0, b1));
		}
	}
}

void palBulletPhysics::NotifyCollision(palBodyBase *pBody, bool enabled) {
	NotifyCollision(pBody, NULL, enabled);
}

void palBulletPhysics::CleanupNotifications(palBodyBase *pBody) {
	std::pair<ListenIterator, ListenIterator> range;

	if (pBody != NULL) {
		range = pallisten.equal_range(pBody);
		// erase the forward list for the one passed in.
		pallisten.erase(range.first, range.second);

		// since only GREATER keys will have this one as a value, just search starting at range.second.
		// plus range.second is not invalidated by the erase.
		ListenIterator i = range.second;
		while (i != pallisten.end()) {
			if (i->second == pBody) {
				ListenIterator oldI = i;
				++i;
				pallisten.erase(oldI);
			}
			else {
				++i;
			}
		}
	}
}


palBulletPhysics::palBulletPhysics()
: m_fFixedTimeStep(0.0f)
, set_substeps(1)
, set_pe(1)
, m_dynamicsWorld(NULL)
, m_dispatcher(NULL)
, m_solver(NULL)
, m_collisionConfiguration(NULL)
, m_overlapCallback(NULL)
, m_ghostPairCallback(NULL)
, m_pbtDebugDraw(NULL)
{}

const char* palBulletPhysics::GetPALVersion() const {
	static char verbuf[512];
	sprintf(verbuf,"PAL SDK V%d.%d.%d\nPAL Bullet V:%d.%d.%d\nFile: %s\nCompiled: %s %s\nModified:%s",
			PAL_SDK_VERSION_MAJOR,PAL_SDK_VERSION_MINOR,PAL_SDK_VERSION_BUGFIX,
			BULLET_PAL_SDK_VERSION_MAJOR,BULLET_PAL_SDK_VERSION_MINOR,BULLET_PAL_SDK_VERSION_BUGFIX,
			__FILE__,__TIME__,__DATE__,__TIMESTAMP__);
	return verbuf;
}

const char* palBulletPhysics::GetVersion() const {
	static char verbuf[256];
	int v = btGetVersion();
	sprintf(verbuf,"Bullet V%d.%d",v/100,v%100);
	return verbuf;
}

void palBulletPhysics::SetFixedTimeStep(Float fixedStep) {
	m_fFixedTimeStep = fixedStep;
}

void palBulletPhysics::SetPE(int n) {
	set_pe = n;
}

void palBulletPhysics::SetSubsteps(int n) {
	set_substeps = n;
}

void palBulletPhysics::SetHardware(bool status) {
	//TODO: enable SPE's
}

bool palBulletPhysics::GetHardware(void) const {
	//TODO: return if using SPE's
	return false;
}

void palBulletPhysics::GetPropertyDocumentation(PAL_MAP<PAL_STRING, PAL_STRING>& descriptions) const
{
	palPhysics::GetPropertyDocumentation(descriptions);
	//descriptions["Bullet_UseMultithreadedDispatcher"] = "Enables the multithreaded physics solver.  See palSolver::SetPE.  This defaults to false.";
	descriptions["Bullet_UseInternalEdgeUtility"] = "Enables the callback for the internal edge checked on the collision detection. Defaults false"
			"This is extra overhead, but it prevents issues related to colliding with the back side and internal edges of triangle meshes."
			"This defaults to false";
	descriptions["Bullet_UseAxisSweepBroadphase"] = "Enables the axis sweep broadphase as opposed to the dynamic bounding volume tree version (Dvbt).  This defaults to false.";
	descriptions["Bullet_AxisSweepBroadphase_RangeX"] = "If Bullet_UseAxisSweepBroadphase is true, this is the X range -X to +X. It defaults to 1000.";
	descriptions["Bullet_AxisSweepBroadphase_RangeY"] = "If Bullet_UseAxisSweepBroadphase is true, this is the Y range -Y to +Y. It defaults to 1000";
	descriptions["Bullet_AxisSweepBroadphase_RangeZ"] = "If Bullet_UseAxisSweepBroadphase is true, this is the Z range -Z to +Z. It defaults to 1000";
	descriptions["Bullet_Solver"] = "Which solver to use.  btSequentialImpulseConstraintSolver (or fast) is the default and only option before 2.8.2.  Others are btNNCGConstraintSolver, btMLCPSolver - btSolveProjectedGaussSeidel(or accurate), btMLCPSolver - btDantzigSolver, btMLCPSolver - btLemkeSolver";

	descriptions["WorldERP"] = "The Global value of the Error Reduction Parameter. Used as Baumgarte factor. Default is 0.2. See http://www.ode.org/ode-latest-userguide.html#sec_3_8_2";
	descriptions["WorldERP2"] = "The Global value of the Error Reduction Parameter. Used in Split Impulse. Default is 0.1. See http://www.ode.org/ode-latest-userguide.html#sec_3_8_2";
	descriptions["WorldCFM"] = "The Global value of the Constraint Force Mixing Parameter. Default is 0.0.  See http://www.ode.org/ode-latest-userguide.html#sec_3_8_2";
	descriptions["WorldDamping"] = "A default world damping value used in non-collision constraints. It's actually a scalar where 1.0 is no damping and 0.0 is completely damped. Default is 1.0.";
	descriptions["LinearSlop"] = "This value, which defaults to 0.0, is subtracted from the penetration when two objects collide, and it helps stop large collision forces";
	descriptions["WarmstartingFactor"] = "This value, which defaults to 0.85, is a multiplier used when warmstarting, i.e. carry over, constraint forces between ticks.";
}


void palBulletPhysics::Init(const palPhysicsDesc& desc) {
	palPhysics::Init(desc);


#if BT_BULLET_VERSION >= 283 && defined(BULLET_MULTITHREADING)
	bool parallel_solver = GetInitProperty("Bullet_UseMultithreadedDispatcher") == "true";
#else
	bool parallel_solver = false;
#endif

	btBroadphaseInterface*	broadphase = NULL;

	if (GetInitProperty("Bullet_UseAxisSweepBroadphase") == "true")
	{
		btScalar maxX = GetInitProperty<btScalar>("Bullet_AxisSweepBroadphase_RangeX", 1000.0f, PAL_FLOAT_EPSILON, PAL_MAX_FLOAT);
		btScalar maxY = GetInitProperty<btScalar>("Bullet_AxisSweexpBroadphase_RangeY", 1000.0f, PAL_FLOAT_EPSILON, PAL_MAX_FLOAT);
		btScalar maxZ = GetInitProperty<btScalar>("Bullet_AxisSweepBroadphase_RangeZ", 1000.0f, PAL_FLOAT_EPSILON, PAL_MAX_FLOAT);
		btVector3 worldMin(-maxX,-maxY,-maxZ);
		btVector3 worldMax(maxX,maxY,maxZ);
		broadphase = new btAxisSweep3(worldMin,worldMax);
	}
	else
	{
		btDbvtBroadphase* dbvtBroadphase = new btDbvtBroadphase();
		// otherwise static/kinematic collisions can be detected that are unintentional and impact performance
		// and lead to inconsistent behavior.
		dbvtBroadphase->m_deferedcollide = true;
		broadphase = dbvtBroadphase;

	}
	m_overlapCallback = new CustomOverlapFilterCallback(this);
	broadphase->getOverlappingPairCache()->setOverlapFilterCallback(m_overlapCallback);
	// so ghosts and the character controller will work.
	m_ghostPairCallback = new btGhostPairCallback;
	broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_ghostPairCallback);
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = 65536;
	cci.m_defaultMaxCollisionAlgorithmPoolSize = 65536;
	m_collisionConfiguration = //new btDefaultCollisionConfiguration(cci);
			new btSoftBodyRigidBodyCollisionConfiguration(cci);

	if (!parallel_solver)
	{
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		std::string solverName = GetInitProperty("Bullet_Solver", "btSequentialImpulseConstraintSolver");
#if BT_BULLET_VERSION >= 282
		if (solverName == "btNNCGConstraintSolver")
		{
			m_solver = new btNNCGConstraintSolver;
		}
		else if (solverName == "btMLCPSolver - btSolveProjectedGaussSeidel" || solverName == "accurate")
		{
			m_solver = new btMLCPSolver(new btSolveProjectedGaussSeidel());
		}
		else if (solverName == "btMLCPSolver - btDantzigSolver")
		{
			m_solver = new btMLCPSolver(new btDantzigSolver());
		}
		else if (solverName == "btMLCPSolver - btLemkeSolver")
		{
			m_solver = new btMLCPSolver(new btLemkeSolver());
		}
		else
#endif
		{
			if (solverName != "btSequentialImpulseConstraintSolver" && solverName != "fast" && solverName != "default")
			{
				printf("Pal-bullet: bullet solver named \"%s\" is not one of the options. Pal provides palPhysics::GetPropertyDocumentation(...) to get a listing of available options.", solverName.c_str());
			}
			m_solver = new btSequentialImpulseConstraintSolver;
		}
	}

#if BT_BULLET_VERSION >= 283 && defined(BULLET_MULTITHREADING)
	else
	{
		m_dispatcher = NULL;

		//m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		m_dispatcher = new	ParallelCollisionDispatcher( m_collisionConfiguration );

		// this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
		// (this may no longer be an issue, not sure)
		m_dispatcher->setDispatcherFlags( btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION );

		btAlignedObjectArray<btConstraintSolver*> solvers;
		solvers.reserve( set_pe );

		std::string solverName = GetInitProperty("Bullet_Solver", "btSequentialImpulseConstraintSolver");
		if (solverName == "btNNCGConstraintSolver")
		{
			for ( int i = 0; i < set_pe; ++i )
			{
				solvers.push_back( new btNNCGConstraintSolver);
			}
		}
		else if (solverName == "btMLCPSolver - btSolveProjectedGaussSeidel" || solverName == "accurate")
		{
			for ( int i = 0; i < set_pe; ++i )
			{
				solvers.push_back( new btMLCPSolver(new btSolveProjectedGaussSeidel()));
			}
		}
		else if (solverName == "btMLCPSolver - btDantzigSolver")
		{
			for ( int i = 0; i < set_pe; ++i )
			{
				solvers.push_back( new btMLCPSolver(new btDantzigSolver()));
			}
		}
		else if (solverName == "btMLCPSolver - btLemkeSolver")
		{
			for ( int i = 0; i < set_pe; ++i )
			{
				solvers.push_back( new btMLCPSolver(new btLemkeSolver()));
			}
		}
		else
		{
			if (solverName != "btSequentialImpulseConstraintSolver" && solverName != "fast" && solverName != "default")
			{
				printf("Pal-bullet: bullet solver named \"%s\" is not one of the options. Pal provides palPhysics::GetPropertyDocumentation(...) to get a listing of available options.", solverName.c_str());
			}

			for ( int i = 0; i < set_pe; ++i )
			{
				solvers.push_back( new btSequentialImpulseConstraintSolver );
			}
		}

		m_solver = new ParallelConstraintSolverPool(&solvers[ 0 ], solvers.size() );

		btDiscreteDynamicsWorld* world = new ParallelDiscreteDynamicsWorld( m_dispatcher, broadphase, m_solver, m_collisionConfiguration );
		m_dynamicsWorld = world;

		world->getSimulationIslandManager()->setIslandDispatchFunction( parallelIslandDispatch );
		//world->getSolverInfo().m_numIterations = 4;
		//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see
		//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
		//world->getSolverInfo().m_solverMode = SOLVER_SIMD + SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
	}
#endif

	if (GetInitProperty("Bullet_UseInternalEdgeUtility") == "true") {
		g_bEnableCustomMaterials = true;
	} else {
		g_bEnableCustomMaterials = false;
	}
	gContactAddedCallback = &CustomMaterialCombinerCallback;


	if (m_dynamicsWorld == nullptr)
	{
		//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,broadphase,m_solver, m_collisionConfiguration);
		btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, broadphase, m_solver,m_collisionConfiguration);
		m_dynamicsWorld = dynamicsWorld;
	}

	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	m_softBodyWorldInfo.m_broadphase = broadphase;

	btVector3 gravity(m_fGravityX, m_fGravityY, m_fGravityZ);
	m_dynamicsWorld->setGravity(gravity);
	m_softBodyWorldInfo.m_gravity = gravity;

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	m_pbtDebugDraw = new palBulletDebugDraw;

	m_CollisionMasks.resize(32U, ~0);

	m_dynamicsWorld->getSolverInfo().m_solverMode =
#if BT_BULLET_VERSION < 280
			SOLVER_USE_FRICTION_WARMSTARTING | SOLVER_USE_2_FRICTION_DIRECTIONS
			| SOLVER_RANDMIZE_ORDER | SOLVER_USE_WARMSTARTING | SOLVER_SIMD;
#else
	SOLVER_USE_WARMSTARTING | SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_FRICTION_SEPARATE
	| SOLVER_RANDMIZE_ORDER | SOLVER_ENABLE_FRICTION_DIRECTION_CACHING | SOLVER_SIMD
	| SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS | SOLVER_ALLOW_ZERO_LENGTH_FRICTION_DIRECTIONS;
#endif

	m_dynamicsWorld->getSolverInfo().m_erp = GetInitProperty("WorldERP", m_dynamicsWorld->getSolverInfo().m_erp, btScalar(PAL_FLOAT_EPSILON), btScalar(1.0));
	m_dynamicsWorld->getSolverInfo().m_globalCfm = GetInitProperty("WorldCFM", m_dynamicsWorld->getSolverInfo().m_globalCfm, btScalar(0.0), btScalar(1.0));
	m_dynamicsWorld->getSolverInfo().m_erp2 = GetInitProperty("WorldERP2", m_dynamicsWorld->getSolverInfo().m_erp2, btScalar(PAL_FLOAT_EPSILON), btScalar(1.0));
	m_dynamicsWorld->getSolverInfo().m_damping = GetInitProperty("WorldDamping", m_dynamicsWorld->getSolverInfo().m_damping, btScalar(0.0), btScalar(1.0));
	m_dynamicsWorld->getSolverInfo().m_linearSlop = GetInitProperty("LinearSlop", m_dynamicsWorld->getSolverInfo().m_linearSlop, btScalar(0.0), btScalar(BT_LARGE_FLOAT));
	m_dynamicsWorld->getSolverInfo().m_warmstartingFactor = GetInitProperty("WarmstartingFactor", m_dynamicsWorld->getSolverInfo().m_warmstartingFactor, btScalar(0.0), btScalar(1.0));
	m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 16;

	m_dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = btScalar(0.0001);

	//m_dynamicsWorld->getSimulationIslandManager()->setSplitIslands(!parallel_solver);
	//m_dynamicsWorld->getDispatchInfo().m_enableSPU = parallel_solver;

	// Reset so it assigns it to the world properly
	SetSolverAccuracy(palSolver::GetSolverAccuracy());
}

void palBulletPhysics::Cleanup() {
	delete m_dynamicsWorld;
	delete m_dispatcher;
	delete m_pbtDebugDraw;
	delete m_solver;
	delete m_softBodyWorldInfo.m_broadphase;
	delete m_collisionConfiguration;
	delete m_overlapCallback;
	delete m_ghostPairCallback;

	m_dynamicsWorld = NULL;
	m_dispatcher = NULL;
	m_pbtDebugDraw = NULL;
	m_solver = NULL;
	m_softBodyWorldInfo.m_broadphase = NULL;
	m_collisionConfiguration = NULL;
	m_overlapCallback = NULL;
	m_ghostPairCallback = NULL;

	PAL_MAP<palAction*, btActionInterface*>::iterator i, iend;
	i = m_BulletActions.begin();
	iend = m_BulletActions.end();
	for (; i != iend; ++i) {
		btActionInterface* bulletAction = i->second;
		delete bulletAction;
	}
	// This isn't really necessary, I just don't like bad pointers hanging around.
	m_BulletActions.clear();
}

void palBulletPhysics::StartIterate(Float timestep) {
	ClearContacts();

	if (m_dynamicsWorld && m_dynamicsWorld->getCollisionObjectArray().size() > 0) {

		palDebugDraw* debugDraw = GetDebugDraw();
		if (debugDraw != NULL) {
			m_pbtDebugDraw->SetPalDebugDraw(debugDraw);
			m_dynamicsWorld->setDebugDrawer(m_pbtDebugDraw);
		} else {
			m_dynamicsWorld->setDebugDrawer(NULL);
		}

		if (m_fFixedTimeStep > 0.0) {
			m_dynamicsWorld->stepSimulation(timestep,set_substeps,m_fFixedTimeStep);
		} else {
			m_dynamicsWorld->stepSimulation(timestep,0);
		}

		if (debugDraw != NULL) {
			m_dynamicsWorld->debugDrawWorld();
		}

		//collision iteration
		int i;
		int numManifolds = m_dispatcher->getNumManifolds();
		for (i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
#if BT_BULLET_VERSION < 280
			const btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			const btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
#else
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();
#endif
			palBodyBase *body1=static_cast<palBodyBase *>(obA->getUserPointer());
			palBodyBase *body2=static_cast<palBodyBase *>(obB->getUserPointer());
#ifdef USE_LISTEN_COLLISION
			if (listenCollision(body1, body2)) {
#endif
				int numContacts = contactManifold->getNumContacts();
				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					palContactPoint cp;
					cp.m_pBody1=body1;
					cp.m_pBody2=body2;
					convertManifoldPtToContactPoint(pt, cp);

					EmitContact(cp);
				}
#ifdef USE_LISTEN_COLLISION
			}
#endif
		}
	}
}

void palBulletPhysics::SetSolverAccuracy(Float fAccuracy) {
	palSolver::SetSolverAccuracy(fAccuracy);
	if (m_dynamicsWorld != NULL) {
		if (fAccuracy < 1.0f)
			fAccuracy = 1.0f;
		m_dynamicsWorld->getSolverInfo().m_numIterations = int(fAccuracy);
	}
}

float palBulletPhysics::GetSolverAccuracy() const {
	// if they set 0-1, we want to return that, otherwise, return int value.
	float result = palSolver::GetSolverAccuracy();
	if (result >= 1.0f && m_dynamicsWorld != NULL) {
		// Add one because the accuracy is 0 based.
		result = float(m_dynamicsWorld->getSolverInfo().m_numIterations);
	}
	return result;
}

bool palBulletPhysics::QueryIterationComplete() const {
	return true;
}
void palBulletPhysics::WaitForIteration() {
	return;
}

void palBulletPhysics::Iterate(Float timestep) {
	StartIterate(timestep);
	WaitForIteration();
}


///////////////
palBulletBodyBase::palBulletBodyBase()
: m_pbtBody(0)
, m_fSkinWidth() {}

palBulletBodyBase::~palBulletBodyBase() {
	palBulletPhysics* bulletPhysics = static_cast<palBulletPhysics*>(GetParent());
	if (bulletPhysics) {
		bulletPhysics->RemoveRigidBody(this);
	}
	if (m_pbtBody) {
		Cleanup();

		while (m_pbtBody->getNumConstraintRefs() > 0) {
			bulletPhysics->RemoveBulletConstraint(m_pbtBody->getConstraintRef(0));
		}

		delete m_pbtBody->getMotionState();
		delete m_pbtBody->getBroadphaseHandle();
		delete m_pbtBody;
		m_pbtBody = NULL;

		bulletPhysics->CleanupNotifications(this);
	}
}

void palBulletBodyBase::SetMaterial(palMaterial *material) {
	/*	for (unsigned int i=0;i<m_Geometries.size();i++) {
		palBulletGeometry *pbg = m_Geometries[i];
		pbg->m_pbtShape->s
	}*/
	palBodyBase::SetMaterial(material);
	if (m_pbtBody) {
		//m_pbtBody->
		m_pbtBody->setFriction(material->m_fStatic);
		m_pbtBody->setRestitution(material->m_fRestitution);
#if BT_BULLET_VERSION > 280
		m_pbtBody->setRollingFriction(material->m_fKinetic);
#endif
		if (material->m_bEnableAnisotropicFriction)
		{
			btVector3 btVec;
			for (unsigned i = 0; i < 3; ++i)
			{
				btVec[i] = material->m_vStaticAnisotropic._vec[i];
			}
			m_pbtBody->setAnisotropicFriction(btVec);
		}
	}
}

void palBulletBodyBase::BuildBody(const palMatrix4x4& pos, Float mass,
		palDynamicsType dynType,
		btCollisionShape *btShape,
		const palVector3& palInertia) {

	btTransform trans;

	convertPalMatToBtTransform(trans, pos);

	btVector3 localInertia(palInertia.x, palInertia.y, palInertia.z);

	btDefaultMotionState* motionState = new btDefaultMotionState(trans);
	btCollisionShape *pShape = btShape;

	//no given shape? get from geom
	if (!btShape) {
		palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (m_Geometries[0]);
		localInertia.setValue(pbtg->m_fInertiaXX, pbtg->m_fInertiaYY, pbtg->m_fInertiaZZ);
		pShape = pbtg->m_pbtShape;
	}

	if (dynType != PALBODY_DYNAMIC) {
		mass = 0;
		localInertia.setValue(0.0f, 0.0f, 0.0f);
	}

	m_pbtBody = new btRigidBody(mass,motionState,pShape,localInertia);
	m_pbtBody->setUserPointer(dynamic_cast<palBodyBase*>(this));
	// Disabling deactivition is really bad.  objects will never go to sleep. which is bad for
	// performance
	//m_pbtBody->setActivationState(DISABLE_DEACTIVATION);

	AssignDynamicsType(dynType, mass, localInertia);

	m_pbtBody->setCollisionFlags(btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK | m_pbtBody->getCollisionFlags());
	static_cast<palBulletPhysics*>(GetParent())->AddRigidBody(this);
#ifdef INTERNAL_DEBUG
	std::cout << "palBulletBodyBase::BuildBody: created body: " << m_pbtBody << std::endl;
#endif
}

void palBulletBodyBase::AssignDynamicsType(palDynamicsType dynType, Float mass, const btVector3& inertia) {
	int currFlags = m_pbtBody->getCollisionFlags();

	switch (dynType) {
	case PALBODY_DYNAMIC:
	{
		currFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
		currFlags &= (~btCollisionObject::CF_KINEMATIC_OBJECT);
		m_pbtBody->setMassProps(mass, inertia);
		break;
	}
	case PALBODY_STATIC:
	{
		currFlags |= btCollisionObject::CF_STATIC_OBJECT;
		currFlags &= (~btCollisionObject::CF_KINEMATIC_OBJECT);
		m_pbtBody->setMassProps(btScalar(0.0), btVector3(0.0f, 0.0f, 0.0f));
		break;
	}
	case PALBODY_KINEMATIC:
	{
		currFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
		currFlags |= btCollisionObject::CF_KINEMATIC_OBJECT;
		m_pbtBody->setMassProps(btScalar(0.0), btVector3(0.0f, 0.0f, 0.0f));
		break;
	}
	}

	m_pbtBody->setCollisionFlags(currFlags);
	m_pbtBody->updateInertiaTensor();

}

void palBulletBodyBase::SetPosition(const palMatrix4x4& location) {
	if (m_pbtBody) {
		btTransform newloc;
		convertPalMatToBtTransform(newloc, location);
		if (m_pbtBody->getMotionState() != NULL)
		{
			m_pbtBody->getMotionState()->setWorldTransform(newloc);
		}
		m_pbtBody->setCenterOfMassTransform(newloc);
		m_pbtBody->activate();
	} else {
		palBodyBase::SetPosition(location);
	}
}

const btTransform palBulletBodyBase::GetWorldTransform() const {
	btTransform xform;
	if (m_pbtBody) {
		if (m_pbtBody->getMotionState() != NULL) {
			m_pbtBody->getMotionState()->getWorldTransform(xform);
		}
		else {
			xform = m_pbtBody->getWorldTransform();
		}
	}
	else {
		convertPalMatToBtTransform(xform, m_mLoc);
	}

	return xform;
}

const palMatrix4x4& palBulletBodyBase::GetLocationMatrixInterpolated() const
{
	if (m_pbtBody && m_pbtBody->getMotionState() != NULL)
	{
		btTransform xform;
		m_pbtBody->getMotionState()->getWorldTransform(xform);
		convertBtTransformToPalMat(m_mLoc, xform);
		return m_mLoc;
	}
	return GetLocationMatrix();
}

const palMatrix4x4& palBulletBodyBase::GetLocationMatrix() const {
	if (m_pbtBody) {
		convertBtTransformToPalMat(m_mLoc, m_pbtBody->getWorldTransform());
	}

	return m_mLoc;
}

palGroup palBulletBodyBase::GetGroup() const {
	if (!m_pbtBody || m_pbtBody->getBroadphaseProxy() == NULL)
		return palBodyBase::GetGroup();
	return convert_to_pal_group(m_pbtBody->getBroadphaseProxy()->m_collisionFilterGroup);
}

void palBulletBodyBase::SetGroup(palGroup group) {
	bool changing = group != GetGroup();
	palBodyBase::SetGroup(group);

	if (!changing || !m_pbtBody || m_pbtBody->getBroadphaseProxy() == NULL)
		return;
	palBulletPhysics* bulletPhysics = static_cast<palBulletPhysics*>(GetParent());

	m_pbtBody->getBroadphaseProxy()->m_collisionFilterGroup = convert_group(group);

	bulletPhysics->ClearBroadPhaseCachePairs(this);
}

Float palBulletBodyBase::GetSkinWidth() const {
	if (m_pbtBody != NULL) {
		return m_pbtBody->getContactProcessingThreshold();
	}
	return m_fSkinWidth;
}

bool palBulletBodyBase::SetSkinWidth(Float skinWidth) {
	m_fSkinWidth = skinWidth;
	if (m_pbtBody != NULL) {
		m_pbtBody->setContactProcessingThreshold(btScalar(skinWidth));
	}
	return true;
}

///////////////
palBulletBody::palBulletBody() {
}

palBulletBody::~palBulletBody() {
}

/*
 * palActivation implementation
 */
// Bullet supports them all
const std::bitset<palBulletBody::DUMMY_ACTIVATION_SETTING_TYPE>
palBulletBody::SUPPORTED_SETTINGS = std::bitset<palBulletBody::DUMMY_ACTIVATION_SETTING_TYPE>(int(~(0xFFFFFFFF << palBulletBody::DUMMY_ACTIVATION_SETTING_TYPE)));

Float palBulletBody::GetActivationLinearVelocityThreshold() const {
	Float velocity;
	if (m_pbtBody) {
		velocity = m_pbtBody->getLinearSleepingThreshold();
	}
	else {
		velocity = Float(-1.0);
	}
	return velocity;
}

void palBulletBody::SetActivationLinearVelocityThreshold(Float velocity) {
	if (m_pbtBody) {
		m_pbtBody->setSleepingThresholds(btScalar(velocity), btScalar(m_pbtBody->getAngularSleepingThreshold()));
	}
}

Float palBulletBody::GetActivationAngularVelocityThreshold() const {
	Float velocity;
	if (m_pbtBody) {
		velocity = m_pbtBody->getAngularSleepingThreshold();
	}
	else {
		velocity = Float(-1.0);
	}
	return velocity;
}

void palBulletBody::SetActivationAngularVelocityThreshold(Float omega) {
	if (m_pbtBody) {
		m_pbtBody->setSleepingThresholds(btScalar(m_pbtBody->getLinearSleepingThreshold()), btScalar(omega));
	}
}

Float palBulletBody::GetActivationTimeThreshold() const {
	Float timeThreshold;
	//if (m_pbtBody) {
	timeThreshold = gDeactivationTime;
	//}
	//else {
	//	timeThreshold = Float(-1.0);
	//}
	return timeThreshold;
}

void palBulletBody::SetActivationTimeThreshold(Float timeThreshold) {
	//if (m_pbtBody) {
	// Yes, it's global, and yes if you set it, it sets the global one.
	gDeactivationTime = timeThreshold;
	//}
}

const std::bitset<palBulletBody::DUMMY_ACTIVATION_SETTING_TYPE>& palBulletBody::GetSupportedActivationSettings() const {
	return SUPPORTED_SETTINGS;
}


//////////////////////
static void DeleteBvhTriangleShape(btBvhTriangleMeshShape*& shapeToDelete) {
	if (shapeToDelete != NULL) {
		delete shapeToDelete->getMeshInterface();
		delete shapeToDelete;
		shapeToDelete = NULL;
	}
}
//////////////////////

///////////////
palBulletGenericBody::palBulletGenericBody()
: m_bGravityEnabled(true)
, m_pCompound(NULL)
, m_pConcave(NULL)
{

}

palBulletGenericBody::~palBulletGenericBody() {
	delete m_pCompound;
	m_pCompound = NULL;
	DeleteBvhTriangleShape(m_pConcave);
}

void palBulletGenericBody::Init(const palMatrix4x4 &pos) {

	palGenericBody::Init(pos);
	palVector3 pvInertia;
	GetInertia(pvInertia.x, pvInertia.y, pvInertia.z);

	for (unsigned i = 0; i < 3; ++i) {
		if (pvInertia._vec[i] < SIMD_EPSILON) {
			pvInertia._vec[i] = 1.0f;
		}
	}

	if (IsUsingOneCenteredGeometry()) {
		BuildBody(pos, m_fMass, GetDynamicsType(), NULL, pvInertia);
	} else if (IsUsingConcaveShape()) {
		RebuildConcaveShapeFromGeometry();
		BuildBody(pos, m_fMass, GetDynamicsType(), m_pConcave, pvInertia);
	} else {
		InitCompoundIfNull();
		BuildBody(pos, m_fMass, GetDynamicsType(), m_pCompound, pvInertia);
	}

	//Reset now that the body is created.
	SetGravityEnabled(IsGravityEnabled());
}

bool palBulletGenericBody::IsDynamic() const {
	if (m_pbtBody != NULL) {
		return !m_pbtBody->isStaticOrKinematicObject();
	}
	return palBulletGenericBody::IsDynamic();
}

bool palBulletGenericBody::IsKinematic() const {
	if (m_pbtBody != NULL) {
		return m_pbtBody->isKinematicObject();
	}
	return palBulletGenericBody::IsKinematic();
}

bool palBulletGenericBody::IsStatic() const {
	if (m_pbtBody != NULL) {
		return m_pbtBody->isStaticObject();
	}
	return palBulletGenericBody::IsStatic();
}

void palBulletGenericBody::SetDynamicsType(palDynamicsType dynType) {
	palGenericBody::SetDynamicsType(dynType);

	if (m_pbtBody == NULL) {
		return;
	}

	btVector3 inertia(m_fInertiaXX, m_fInertiaYY, m_fInertiaZZ);
	AssignDynamicsType(dynType, m_fMass, inertia);
	//Have to reset gravity after setting the dynamics type because statics and kinematics have 0 gravity.
	SetGravityEnabled(IsGravityEnabled());
	palBulletPhysics* physics = static_cast<palBulletPhysics*>(GetParent());
	physics->RemoveRigidBody(this);
	physics->AddRigidBody(this);
}


void palBulletGenericBody::SetMass(Float mass) {
	palGenericBody::SetMass(mass);
	if (m_pbtBody && m_eDynType == PALBODY_DYNAMIC) {
		btVector3 inertia(m_fInertiaXX, m_fInertiaYY, m_fInertiaZZ);
		m_pbtBody->setMassProps(btScalar(mass), inertia);
		m_pbtBody->updateInertiaTensor();
	}
}

void palBulletGenericBody::SetGravityEnabled(bool enabled) {
	if (m_pbtBody != NULL) {
		if (enabled && m_eDynType == PALBODY_DYNAMIC) {
			palVector3 pv;
			static_cast<palBulletPhysics*>(GetParent())->GetGravity(pv);
			m_pbtBody->setGravity(btVector3(pv.x, pv.y, pv.z));
		} else {
			m_pbtBody->setGravity(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		}
	}
	m_bGravityEnabled = enabled;
}

bool palBulletGenericBody::IsGravityEnabled() const {
	return m_bGravityEnabled;
}

void palBulletGenericBody::SetCollisionResponseEnabled(bool enabled) {
	if (m_pbtBody != NULL) {
		int collisionFlags = m_pbtBody->getCollisionFlags();
		if (enabled) {
			m_pbtBody->setCollisionFlags(collisionFlags & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
		} else {
			m_pbtBody->setCollisionFlags(collisionFlags | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		}
	}
}

bool palBulletGenericBody::IsCollisionResponseEnabled() const {
	bool result = true;
	if (m_pbtBody != NULL) {
		int collisionFlags = m_pbtBody->getCollisionFlags();
		result = !(collisionFlags & btCollisionObject::CF_NO_CONTACT_RESPONSE);
	}
	return result;
}

void palBulletGenericBody::SetInertia(Float Ixx, Float Iyy, Float Izz) {
	palGenericBody::SetInertia(Ixx, Iyy, Izz);
	if (m_pbtBody && m_eDynType == PALBODY_DYNAMIC) {
		btVector3 inertia(m_fInertiaXX, m_fInertiaYY, m_fInertiaZZ);
		m_pbtBody->setMassProps(btScalar(m_fMass), inertia);
		m_pbtBody->updateInertiaTensor();
	}
}

void palBulletGenericBody::SetLinearDamping(Float damping)
{
	palGenericBody::SetLinearDamping(damping);
	if (m_pbtBody)
	{
		m_pbtBody->setDamping(btScalar(damping), m_pbtBody->getAngularDamping());
	}
}

Float palBulletGenericBody::GetLinearDamping() const
{
	if (m_pbtBody)
	{
		return Float(m_pbtBody->getLinearDamping());
	}
	return palGenericBody::GetLinearDamping();
}

void palBulletGenericBody::SetAngularDamping(Float damping)
{
	palGenericBody::SetAngularDamping(damping);
	if (m_pbtBody)
	{
		m_pbtBody->setDamping(m_pbtBody->getLinearDamping(), damping);
	}
}

Float palBulletGenericBody::GetAngularDamping() const
{
	if (m_pbtBody)
	{
		return Float(m_pbtBody->getAngularDamping());
	}
	return palGenericBody::GetAngularDamping();
}

void palBulletGenericBody::SetMaxAngularVelocity(Float maxAngVel)
{
	palGenericBody::SetMaxAngularVelocity(maxAngVel);
	// TODO this will have to be done at tick time.
}

Float palBulletGenericBody::GetMaxAngularVelocity() const
{
	return palGenericBody::GetMaxAngularVelocity();
}


void palBulletGenericBody::InitCompoundIfNull() {
	if (m_pCompound == NULL) {
		m_pCompound = new btCompoundShape(false);
		for (unsigned i = 0; i < m_Geometries.size(); ++i)
			AddShapeToCompound(m_Geometries[i]);
	}
}

void palBulletGenericBody::AddShapeToCompound(palGeometry* pGeom) {
	if (m_pCompound == NULL)
		return;

	palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (pGeom);
	palMatrix4x4 m = pbtg->GetOffsetMatrix();//GetLocationMatrix();
	btTransform localTrans;
	convertPalMatToBtTransform(localTrans, m);
	if (pbtg->BulletGetCollisionShape()->isCompound() || pbtg->BulletGetCollisionShape()->isConvex()) {
		// Ugh, Can't add a concave shape to a compound shape.
		m_pCompound->addChildShape(localTrans, pbtg->BulletGetCollisionShape());
	}
}

void palBulletGenericBody::RemoveShapeFromCompound(palGeometry* pGeom) {
	if (m_pCompound == NULL) {
		return;
	}

	palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (pGeom);
	m_pCompound->removeChildShape(pbtg->BulletGetCollisionShape());
}

bool palBulletGenericBody::IsUsingOneCenteredGeometry() const {
	bool result = false;
	if (m_Geometries.size() == 1) {
		palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (m_Geometries[0]);
		palMatrix4x4 m = pbtg->GetOffsetMatrix();
		if (mat_is_identity(&m))
			result = true;
	}
	return result;
}

bool palBulletGenericBody::IsUsingConcaveShape() const {
	bool concaveFound = false;
	if (!m_Geometries.empty()) {
		for (size_t i = 0; i < m_Geometries.size(); ++i) {
			palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *>(m_Geometries[i]);
			concaveFound = pbtg->BulletGetCollisionShape()->isConcave();
			if (concaveFound)
				break;
		}
	}
	return concaveFound;
}

void palBulletGenericBody::RebuildConcaveShapeFromGeometry() {
	DeleteBvhTriangleShape(m_pConcave);

	btTriangleIndexVertexArray *trimesh = new btTriangleIndexVertexArray();

	for (size_t i = 0; i < m_Geometries.size(); ++i) {
		palGeometry* pGeom = m_Geometries[i];
		int* pIndices = pGeom->GenerateMesh_Indices();
		Float* pVertices = pGeom->GenerateMesh_Vertices();
		int nIndices = pGeom->GetNumberOfIndices();

		AddMeshToTrimesh(trimesh, pVertices, pGeom->GetNumberOfVertices(), pIndices, nIndices);
	}
	m_pConcave = new btBvhTriangleMeshShape(trimesh, true, true);
}

void palBulletGenericBody::ConnectGeometry(palGeometry* pGeom) {
	palGenericBody::ConnectGeometry(pGeom);

	if (m_pbtBody != NULL)
	{
		// what about just clearing the broadphase cache?
		palBulletPhysics* physics = static_cast<palBulletPhysics*>(GetParent());
		physics->RemoveRigidBody(this);

		if (IsUsingOneCenteredGeometry()) {
			delete m_pCompound;
			m_pCompound = NULL;
			DeleteBvhTriangleShape(m_pConcave);
			palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (pGeom);
			btCollisionShape* shape = pbtg->BulletGetCollisionShape();
			m_pbtBody->setCollisionShape(shape);
		} else if (IsUsingConcaveShape()) {
			delete m_pCompound;
			m_pCompound = NULL;
			RebuildConcaveShapeFromGeometry();
			m_pbtBody->setCollisionShape(m_pConcave);
		} else {
			DeleteBvhTriangleShape(m_pConcave);
			AddShapeToCompound(pGeom);
			// This is done after the above on purpose
			InitCompoundIfNull();
			m_pbtBody->setCollisionShape(m_pCompound);
		}

		physics->AddRigidBody(this);
	}
}

void palBulletGenericBody::RemoveGeometry(palGeometry* pGeom)
{
	palGenericBody::RemoveGeometry(pGeom);
	if (m_pbtBody != NULL) {
		palBulletPhysics* physics = static_cast<palBulletPhysics*>(GetParent());
		physics->RemoveRigidBody(this);

		if (IsUsingOneCenteredGeometry()) {
			delete m_pCompound;
			m_pCompound = NULL;
			DeleteBvhTriangleShape(m_pConcave);
			palBulletGeometry *pbtg=dynamic_cast<palBulletGeometry *> (pGeom);
			btCollisionShape* shape = pbtg->BulletGetCollisionShape();
			m_pbtBody->setCollisionShape(shape);
		} else if (IsUsingConcaveShape()) {
			delete m_pCompound;
			m_pCompound = NULL;
			RebuildConcaveShapeFromGeometry();
			m_pbtBody->setCollisionShape(m_pConcave);
		} else {
			DeleteBvhTriangleShape(m_pConcave);
			RemoveShapeFromCompound(pGeom);
			// This is done after the above on purpose
			InitCompoundIfNull();
			m_pbtBody->setCollisionShape(m_pCompound);
		}
		// what about just clearing the broadphase cache?
		physics->AddRigidBody(this);
	}
}


bool palBulletBody::IsActive() const {
	return m_pbtBody->isActive();
}
/*
#define ACTIVE_TAG 1
#define ISLAND_SLEEPING 2
#define WANTS_DEACTIVATION 3
#define DISABLE_DEACTIVATION 4
#define DISABLE_SIMULATION 5
 */
void palBulletBody::SetActive(bool active) {
	if (active) {
		m_pbtBody->activate();
		//m_pbtBody->setActivationState(DISABLE_DEACTIVATION);
	} else {
		m_pbtBody->setActivationState(ISLAND_SLEEPING);
	}
}



void palBulletBody::ApplyForce(Float fx, Float fy, Float fz) {
	btVector3 force(fx,fy,fz);
	m_pbtBody->applyCentralForce(force);
}

void palBulletBody::ApplyTorque(Float tx, Float ty, Float tz) {
	btVector3 torque(tx,ty,tz);
	m_pbtBody->applyTorque(torque);
}

void palBulletBody::ApplyImpulse(Float fx, Float fy, Float fz) {
	btVector3 impulse(fx,fy,fz);
	m_pbtBody->applyCentralImpulse(impulse);
}

void palBulletBody::ApplyAngularImpulse(Float fx, Float fy, Float fz) {
	btVector3 impulse(fx,fy,fz);
	m_pbtBody->applyTorqueImpulse(impulse);
}

void palBulletBody::GetLinearVelocity(palVector3& velocity) const {
	btVector3 vel = m_pbtBody->getLinearVelocity();
	velocity.x = vel.x();
	velocity.y = vel.y();
	velocity.z = vel.z();
}
void palBulletBody::GetAngularVelocity(palVector3& velocity) const {
	btVector3 vel = m_pbtBody->getAngularVelocity();
	velocity.x = vel.x();
	velocity.y = vel.y();
	velocity.z = vel.z();
}

void palBulletBody::SetLinearVelocity(const palVector3& velocity) {
	m_pbtBody->setLinearVelocity(btVector3(velocity.x,velocity.y,velocity.z));
}

void palBulletBody::SetAngularVelocity(const palVector3& velocity) {
	m_pbtBody->setAngularVelocity(btVector3(velocity.x,velocity.y,velocity.z));
}


palBulletGeometry::palBulletGeometry()
: m_pbtShape(NULL) {}

palBulletGeometry::~palBulletGeometry() {
	delete m_pbtShape;
	m_pbtShape = NULL;
}

Float palBulletGeometry::GetMargin() const {
	return m_pbtShape == NULL ? Float(0.0) : Float(m_pbtShape->getMargin());
}

bool palBulletGeometry::SetMargin(Float margin) {
	if (m_pbtShape)
	{
		if (margin < 0.0) {
			m_pbtShape->setMargin(btScalar(0.04));
		} else {
			m_pbtShape->setMargin(btScalar(margin));
		}
	}
	return true;
}

palBulletBoxGeometry::palBulletBoxGeometry()
: m_pbtBoxShape(NULL) {}

void palBulletBoxGeometry::Init(const palMatrix4x4 &pos, Float width, Float height, Float depth, Float mass) {
	palBoxGeometry::Init(pos,width,height,depth,mass);
	palVector3 dim = GetXYZDimensions();

	m_pbtBoxShape = new btBoxShape(btVector3(dim.x*(Float)0.5,dim.y*(Float)0.5,dim.z*(Float)0.5));
	m_pbtShape = m_pbtBoxShape;
	//m_pbtShape->setMargin(0.0f);
}


palBulletSphereGeometry::palBulletSphereGeometry()
: m_btSphereShape(0) {}

void palBulletSphereGeometry::Init(const palMatrix4x4 &pos, Float radius, Float mass) {
	palSphereGeometry::Init(pos,radius,mass);
	m_btSphereShape = new btSphereShape(radius); // this seems wrong!
	m_pbtShape = m_btSphereShape;
	//m_pbtShape->setMargin(0.0f);
}

palBulletCapsuleGeometry::palBulletCapsuleGeometry()
: m_btCapsuleShape(0) {}

void palBulletCapsuleGeometry::Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass) {
	palCapsuleGeometry::Init(pos,radius,length,mass);
	palAxis upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();

	switch (upAxis) {
	case PAL_Z_AXIS:
		m_btCapsuleShape = new btCapsuleShapeZ(radius, length);
		break;
	case PAL_X_AXIS:
		m_btCapsuleShape = new btCapsuleShapeX(radius, length);
		break;
	case PAL_Y_AXIS:
		m_btCapsuleShape = new btCapsuleShape(radius, length);
		break;
	default:
		throw new palException("Invalid axis is 'up'. This should never happen.");
	}
	m_pbtShape = m_btCapsuleShape;
	//m_pbtShape->setMargin(0.0f);
}

palBulletCylinderGeometry::palBulletCylinderGeometry()
: m_btCylinderShape(0) {}

void palBulletCylinderGeometry::Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass) {
	palCylinderGeometry::Init(pos,radius,length,mass);
	palAxis upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();

	switch (upAxis) {
	case PAL_Z_AXIS:
		m_btCylinderShape = new btCylinderShapeZ(btVector3(radius, radius, length/2.0f)); //Half lengths
		break;
	case PAL_X_AXIS:
		m_btCylinderShape = new btCylinderShapeX(btVector3(length/2.0f, radius, radius)); //Half lengths
		break;
	case PAL_Y_AXIS:
		m_btCylinderShape = new btCylinderShape(btVector3(radius,length/2.0f,radius)); //Half lengths
		break;
	default:
		throw new palException("Invalid axis is 'up'. This should never happen.");
	}
	m_pbtShape = m_btCylinderShape;
	//m_pbtShape->setMargin(0.0f);
}


palBulletOrientatedTerrainPlane::palBulletOrientatedTerrainPlane()
: m_pbtPlaneShape(0) {}

palBulletOrientatedTerrainPlane::~palBulletOrientatedTerrainPlane() {
	delete m_pbtPlaneShape;
}

void palBulletOrientatedTerrainPlane::Init(Float x, Float y, Float z, Float nx, Float ny, Float nz, Float min_size) {
	palOrientatedTerrainPlane::Init(x,y,z,nx,ny,nz,min_size);

	btVector3 normal(nx,ny,nz);
	normal.normalize();
	m_pbtPlaneShape = new btStaticPlaneShape(normal,CalculateD());

	palMatrix4x4 mat;
	mat_identity(&mat);
	mat_set_translation(&mat,x,y,z);
	BuildBody(mat, 0, PALBODY_STATIC, m_pbtPlaneShape);
#if 0
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	btTransform final_groundTransform;
	final_groundTransform.setIdentity();
	final_groundTransform.setOrigin(btVector3(0,0,0));
	//	final_groundTransform.setFromOpenGLMatrix(m_mLoc._mat);

	btVector3 localInertia(0,0,0);
	m_pbtMotionState = new btDefaultMotionState(final_groundTransform);
	m_pbtBody = new btRigidBody(0,m_pbtMotionState,m_pbtPlaneShape,localInertia);

	m_pbtBody->setCollisionFlags(m_pbtBody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	g_DynamicsWorld->addRigidBody(m_pbtBody);
#endif
}


palBulletTerrainPlane::palBulletTerrainPlane()
: m_pbtBoxShape(0) {}

void palBulletTerrainPlane::Init(Float x, Float y, Float z, Float min_size) {
	palTerrainPlane::Init(x,y,z,min_size);

	unsigned int upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();
	palMatrix4x4 mat;
	mat_identity(&mat);

	if (upAxis == 2)
	{
		m_pbtBoxShape = new btBoxShape(btVector3(min_size*(Float)0.5, min_size*(Float)0.5, (Float)1.0));
		mat_set_translation(&mat,x,y,z-1.0f);
	}
	else if (upAxis == 0)
	{
		m_pbtBoxShape = new btBoxShape(btVector3(1.0, min_size*(Float)0.5, min_size*(Float)0.5));
		mat_set_translation(&mat,x-1.0f,y,z);
	}
	else
	{
		m_pbtBoxShape = new btBoxShape(btVector3(min_size*(Float)0.5, (Float)1.0, min_size*(Float)0.5));
		mat_set_translation(&mat,x,y-1.0f,z);
	}
	BuildBody(mat, 0, PALBODY_STATIC, m_pbtBoxShape);
}

palBulletTerrainMesh::palBulletTerrainMesh()
: m_pbtTriMeshShape(0) {}

palBulletTerrainMesh::~palBulletTerrainMesh() {
	DeleteBvhTriangleShape(m_pbtTriMeshShape);
}

void palBulletTerrainMesh::Init(Float x, Float y, Float z, const Float *pVertices, int nVertices, const int *pIndices, int nIndices) {
	m_Indices.reserve(nIndices);
	for (int i = 0; i < nIndices; ++i)
	{
		m_Indices.push_back(pIndices[i]);
	}

	int nVertFloats = nVertices * 3;
	m_Vertices.reserve(nVertFloats);
	for (int i = 0; i < nVertFloats; ++i)
	{
		m_Vertices.push_back(pVertices[i]);
	}

	palTerrainMesh::Init(x,y,z,&m_Vertices.front(),nVertices,&m_Indices.front(),nIndices);

	btTriangleIndexVertexArray *trimesh = new btTriangleIndexVertexArray();
	AddMeshToTrimesh(trimesh, &m_Vertices.front(), nVertices, &m_Indices.front(), nIndices);
	//	btTriangleMesh* trimesh = new btTriangleMesh(true, false);
	//	int pi;
	//	for (int i=0;i<nIndices/3;i++) {
	//		pi = pIndices[i*3+0];
	//		btVector3 v0(	pVertices[pi*3+0],
	//						pVertices[pi*3+1],
	//						pVertices[pi*3+2]);
	//		pi = pIndices[i*3+1];
	//		btVector3 v1(	pVertices[pi*3+0],
	//						pVertices[pi*3+1],
	//						pVertices[pi*3+2]);
	//		pi = pIndices[i*3+2];
	//		btVector3 v2(	pVertices[pi*3+0],
	//						pVertices[pi*3+1],
	//						pVertices[pi*3+2]);
	//		trimesh->addTriangle(v0,v1,v2);
	//	}

	m_pbtTriMeshShape = new btBvhTriangleMeshShape(trimesh,true);
	palMatrix4x4 mat;
	mat_identity(&mat);
	mat_set_translation(&mat,x,y,z);
	BuildBody(mat, 0, PALBODY_STATIC, m_pbtTriMeshShape);
}
/*
palMatrix4x4& palBulletTerrainMesh::GetLocationMatrix() {
	memset(&m_mLoc,0,sizeof(m_mLoc));
	m_mLoc._11=1;m_mLoc._22=1;m_mLoc._33=1;m_mLoc._44=1;
	m_mLoc._41=m_fPosX;
	m_mLoc._42=m_fPosY;
	m_mLoc._43=m_fPosZ;
	return m_mLoc;
}

void palBulletTerrainMesh::SetMaterial(palMaterial *material) {
	m_pbtBody->setFriction(material->m_fStatic);
	m_pbtBody->setRestitution(material->m_fRestitution);
}
 */
palBulletTerrainHeightmap::palBulletTerrainHeightmap() {
}

void palBulletTerrainHeightmap::Init(Float px, Float py, Float pz, Float width, Float depth, int terrain_data_width, int terrain_data_depth, const Float *pHeightmap) {
	palTerrainHeightmap::Init(px,py,pz,width,depth,terrain_data_width,terrain_data_depth,pHeightmap);
	int iTriIndex;
	float fTerrainX, fTerrainZ;
	int x,z;

	int nv=m_iDataWidth*m_iDataDepth;
	int ni=(m_iDataWidth-1)*(m_iDataDepth-1)*2*3;

	Float *v = new Float[nv*3];
	int *ind = new int[ni];

	// Set the vertex values
	fTerrainZ = -m_fDepth/2;
	for (z=0; z<m_iDataDepth; z++)
	{
		fTerrainX = -m_fWidth/2;
		for (x=0; x<m_iDataWidth; x++)
		{
			v[(x + z*m_iDataWidth)*3+0]=fTerrainX;
			v[(x + z*m_iDataWidth)*3+1]=pHeightmap[x+z*m_iDataWidth];
			v[(x + z*m_iDataWidth)*3+2]=fTerrainZ;

			fTerrainX += (m_fWidth / (m_iDataWidth-1));
		}
		fTerrainZ += (m_fDepth / (m_iDataDepth-1));
	}

	iTriIndex = 0;
	int xDim=m_iDataWidth;
	int yDim=m_iDataDepth;
	int y;
	for (y=0;y < yDim-1;y++)
		for (x=0;x < xDim-1;x++) {
			ind[iTriIndex*3+0]=(y*xDim)+x;
			ind[iTriIndex*3+1]=(y*xDim)+xDim+x;
			ind[iTriIndex*3+2]=(y*xDim)+x+1;
			// Move to the next triangle in the array
			iTriIndex += 1;

			ind[iTriIndex*3+0]=(y*xDim)+x+1;
			ind[iTriIndex*3+1]=(y*xDim)+xDim+x;
			ind[iTriIndex*3+2]=(y*xDim)+x+xDim+1;
			// Move to the next triangle in the array
			iTriIndex += 1;
		}
	palBulletTerrainMesh::Init(px,py,pz,v,nv,ind,ni);

	delete [] v;
	delete [] ind;
}

palBulletConvexGeometry::palBulletConvexGeometry()
: m_pbtConvexShape(0) {}

void palBulletConvexGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, Float mass) {
	palConvexGeometry::Init(pos,pVertices,nVertices,mass);
	InternalInit(pVertices, nVertices, 0, 0);
}

void palBulletConvexGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass){
	palConvexGeometry::Init(pos,pVertices,nVertices,pIndices,nIndices,mass);
	InternalInit(pVertices, nVertices, pIndices, nIndices);
}

struct btVecLess
{
	bool operator()( const btVector3& lhs, const btVector3& rhs ) const
	{
		if (lhs.getX() < rhs.getX())
			return true;
		else if (lhs.getX() == rhs.getX() && lhs.getY() < rhs.getY())
			return true;
		else if (lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY() && lhs.getZ() < rhs.getZ())
			return true;
		return false;
	}
};

void palBulletConvexGeometry::InternalInit(const Float *pVertices, unsigned int nVertices, const int *pIndices, int nIndices)
{
//	btVector3* supportPoints = new btVector3[nVertices];
//
//	for (unsigned i = 0; i < nVertices; ++i)
//	{
//		supportPoints[i] = btVector3(pVertices[3*i + 0], pVertices[3*i + 1], pVertices[3*i + 2]);
//		printf("InVec: %lf, %lf, %lf\n", supportPoints[i].getX(), supportPoints[i].getY(), supportPoints[i].getZ());
//	}
//
//	HullDesc hd;
//	hd.mFlags = QF_TRIANGLES;
//	hd.mVcount = nVertices;
//
//	hd.mVertices = supportPoints;
//	hd.mVertexStride = sizeof(btVector3);
//	hd.mMaxVertices = nVertices;
//
//	HullLibrary hl;
//	HullResult hr;
//	if (hl.CreateConvexHull (hd, hr) == QE_FAIL)
//	{
//		printf("Hull creation failed\n");
//		return;
//	}

#if BT_FLOAT_IS_PAL_FLOAT
	m_pbtConvexShape = new btConvexHullShape(pVertices,nVertices,sizeof(btScalar)*3);
#else
	m_pbtConvexShape = new btConvexHullShape();

	//for (unsigned i = 0; i < hr.mNumOutputVertices; ++i)
	//{
	//std::set<btVector3, btVecLess> theSet;
	for (unsigned i = 0; i < nVertices/*unsigned(nIndices)*/; ++i)
	{
		//btVector3& nextv = hr.m_OutputVertices[i];
		int idx = i;//pIndices[i];
		btVector3 nextv(pVertices[3*idx + 0], pVertices[3*idx + 1], pVertices[3*idx + 2]);
//		if (theSet.find(nextv) == theSet.end())
//		{
//			theSet.insert(nextv);
				//printf("OutVec: %lf, %lf, %lf\n", nextv.getX(), nextv.getY(), nextv.getZ());
#if BT_BULLET_VERSION < 282
		m_pbtConvexShape->addPoint(nextv);
#else
		m_pbtConvexShape->addPoint(nextv, false);
#endif
//		}
	}

	//printf("nVerts %d vs AddedVerts %d\n", nVertices, m_pbtConvexShape->getNumPoints());

#if BT_BULLET_VERSION > 281
	m_pbtConvexShape->recalcLocalAabb();
#endif
#endif
	//	}
	// default margin is 0.04
	//	m_pbtConvexShape = convexShape;
	m_pbtShape = m_pbtConvexShape;
	//m_pbtShape->setMargin(0.0f);
	// free temporary hull result that we just copied
//	hl.ReleaseResult (hr);
//	delete[] supportPoints;
}

palBulletConcaveGeometry::palBulletConcaveGeometry()
: m_pbtTriMeshShape()
, m_pInternalEdgeInfo()
{
}

palBulletConcaveGeometry::~palBulletConcaveGeometry() {
	if (m_pbtTriMeshShape) {
		/* You might think Bullet would clean this up when
		   m_pbtTriMeshShape gets deleted by ~palBulletGeometry, but
		   Bullet doesn't clean up the mesh interface. */
		delete m_pbtTriMeshShape->getMeshInterface();
		m_pbtTriMeshShape = NULL;
	}
	delete m_pInternalEdgeInfo;
	m_pInternalEdgeInfo = NULL;
}

void palBulletConcaveGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass) {
	palConcaveGeometry::Init(pos,pVertices,nVertices,pIndices,nIndices,mass);

	btTriangleIndexVertexArray *trimesh = new btTriangleIndexVertexArray();
	AddMeshToTrimesh(trimesh, m_pUntransformedVertices, nVertices, m_pIndices, nIndices);
	m_pbtTriMeshShape = new btBvhTriangleMeshShape(trimesh,true);

	m_pInternalEdgeInfo = new btTriangleInfoMap();
	btGenerateInternalEdgeInfo(m_pbtTriMeshShape, m_pInternalEdgeInfo);

	m_pbtShape = m_pbtTriMeshShape;
	//m_pbtShape = new btSphereShape(50);
	//m_pbtShape->setMargin(0.0f);
}

class BulletTriangleCallbackAdapter: public palTriangleCallback
{
public:
	BulletTriangleCallbackAdapter(btTriangleCallback* bulletTriCallback)
	: m_BulletTriCallback(bulletTriCallback)
	{
	}

	~BulletTriangleCallbackAdapter() override {}

	void ProcessTriangle(palTriangle palTri, int partId, int triangleIndex) override
	{
		btVector3 btTri[3];
		for (unsigned i = 0; i < 3; ++i)
		{
			btTri[i].setValue(btScalar(palTri.vertices[i][0]), btScalar(palTri.vertices[i][1]), btScalar(palTri.vertices[i][2]));
		}
		m_BulletTriCallback->processTriangle(btTri, partId, triangleIndex);
	}

	btTriangleCallback* m_BulletTriCallback;
};

class CustomBulletConcaveShape: public btConcaveShape
{
public:
	CustomBulletConcaveShape(palCustomGeometryCallback& callback)
	: m_pCallback(&callback)
	, m_localScaling(btScalar(1.0),btScalar(1.0),btScalar(1.0))
	{
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE;

	}

	virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const
	{
		palVector3 out, size = m_pCallback->GetBoundingBox().max - m_pCallback->GetBoundingBox().min;
		palGeometry::CalculateBoxInertia(size, Float(mass), out);
		inertia.setValue(btScalar(out.x), btScalar(out.y), btScalar(out.z));
	}

	void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}

	const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	//debugging
	const char*	getName() const override {return "CustomBulletConcaveShape";}

	void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const override
	{
		palBoundingBox bb;
		bb.min.Set( Float(aabbMin.x()), Float(aabbMin.y()), Float(aabbMin.z()) );
		bb.max.Set( Float(aabbMax.x()), Float(aabbMax.y()), Float(aabbMax.z()) );
		BulletTriangleCallbackAdapter adapter(callback);
		(*m_pCallback)(bb, adapter);
	}

	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const override
	{
		const palBoundingBox& pbb = m_pCallback->GetBoundingBox();
		aabbMin.setValue(btScalar(pbb.min.x), btScalar(pbb.min.y), btScalar(pbb.min.z));
		aabbMax.setValue(btScalar(pbb.max.x), btScalar(pbb.max.y), btScalar(pbb.max.z));
		aabbMin = t(aabbMin);
		aabbMax = t(aabbMax);
	}

	palCustomGeometryCallback* m_pCallback;
	btVector3 m_localScaling;
};

palBulletCustomConcaveGeometry::palBulletCustomConcaveGeometry()
: m_pCustomShape()
, m_pInternalEdgeInfo()
{
}

palBulletCustomConcaveGeometry::~palBulletCustomConcaveGeometry()
{
}

void palBulletCustomConcaveGeometry::Init(const palMatrix4x4& pos, Float mass, palCustomGeometryCallback& callback)
{
	palCustomConcaveGeometry::Init(pos, mass, callback);
	m_pCustomShape = new CustomBulletConcaveShape(callback);
	m_pbtShape = m_pCustomShape;
}

palBulletPSDSensor::palBulletPSDSensor()
: m_fRelativePosX()
, m_fRelativePosY()
, m_fRelativePosZ()
{
}

void palBulletPSDSensor::Init(palBody *body, Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range) {
	palPSDSensor::Init(body,x,y,z,dx,dy,dz,range);
	palVector3 pos;
	body->GetPosition(pos);
	m_fRelativePosX = m_fPosX - pos.x;
	m_fRelativePosY = m_fPosY - pos.y;
	m_fRelativePosZ = m_fPosZ - pos.z;
}
#if 0
#include <GL/gl.h>
#pragma comment (lib, "opengl32.lib")
#endif
Float palBulletPSDSensor::GetDistance() const {
	// TODO simplify this by using vec_mat_mul instead of mat_multiply
	btVector3 from;
	palMatrix4x4 m;
	palMatrix4x4 bodypos = m_pBody->GetLocationMatrix();
	palMatrix4x4 out;

	mat_identity(&m);
	mat_translate(&m,m_fRelativePosX,m_fRelativePosY,m_fRelativePosZ);
	mat_multiply(&out,&bodypos,&m);
	from.setX(out._41);
	from.setY(out._42);
	from.setZ(out._43);

	mat_identity(&m);
	mat_translate(&m,m_fAxisX,m_fAxisY,m_fAxisZ);
	mat_multiply(&out,&bodypos,&m);

	palVector3 newaxis;
	newaxis.x=out._41-bodypos._41;
	newaxis.y=out._42-bodypos._42;
	newaxis.z=out._43-bodypos._43;
	vec_norm(&newaxis);


	palRayHit hit;
	static_cast<const palBulletPhysics*>(GetParent())->RayCast(from.x(), from.y(), from.z(),
			newaxis.x, newaxis.y, newaxis.z,
			m_fRange, hit);
	/*
	 * Checking hit position and not just hit since if the hit
	 * position isn't available, it's hard to see how the distance
	 * could be meaningful.
	 */
	if (hit.m_bHitPosition && hit.m_pBody) {
		return hit.m_fDistance;
	}
#if 0
	btVector3 to;
	to.setX(from.x()+newaxis.x*m_fRange);
	to.setY(from.y()+newaxis.y*m_fRange);
	to.setZ(from.z()+newaxis.z*m_fRange);

	btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

	static_cast<palBulletPhysics*>(GetParent())->m_dynamicsWorld->rayTest(from, to, rayCallback);
	if (rayCallback.hasHit())
	{
		btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
		if (body)
		{
			return m_fRange*rayCallback.m_closestHitFraction;
		}
	}
#endif
	return m_fRange;
}



palBulletMotor::palBulletMotor()
: m_updateFunc(0), m_revolute(0), m_6dof(0) {}

void palBulletMotor::Init(palLink *pLink, int axis) {
	m_axis = axis;
	m_revolute = dynamic_cast<palBulletRevoluteLink *> (pLink);
	if (m_revolute == 0)
	{
		m_6dof = dynamic_cast<palBulletGenericLink *> (pLink);
		if (m_6dof != 0)
		{
			axis = std::max(0, axis);
			axis = std::min(5, axis);
			m_updateFunc = &palBulletMotor::Update6DOF;
		}
	}
	else
	{
		m_axis = -1;
		m_updateFunc = &palBulletMotor::UpdateRevolute;
	}
}

void palBulletMotor::Update(Float targetVelocity, Float Max) {
	if (m_updateFunc != 0)
		(this->*m_updateFunc)(targetVelocity, Max);
}

void palBulletMotor::Update6DOF(Float targetVelocity, Float Max) {
#if BT_BULLET_VERSION < 283
	typedef btRotationalLimitMotor RotationalMotor;
	typedef btTranslationalLimitMotor TranslationalMotor;
#else
	typedef btRotationalLimitMotor2 RotationalMotor;
	typedef btTranslationalLimitMotor2 TranslationalMotor;
#endif
	if (m_axis > 2)
	{
		RotationalMotor* motor = m_6dof->BulletGetGenericConstraint()->getRotationalLimitMotor(m_axis - PAL_AXIS_COUNT);
		motor->m_enableMotor = true;
		motor->m_targetVelocity = btScalar(targetVelocity);
		motor->m_maxMotorForce = btScalar(Max);
	}
	else
	{
		TranslationalMotor* motor = m_6dof->BulletGetGenericConstraint()->getTranslationalLimitMotor();
		motor->m_enableMotor[m_axis] = true;
		motor->m_maxMotorForce[m_axis] = btScalar(Max);
		motor->m_targetVelocity[m_axis] = btScalar(targetVelocity);
	}
	m_6dof->BulletGetGenericConstraint()->getRigidBodyA().activate();
	m_6dof->BulletGetGenericConstraint()->getRigidBodyB().activate();
}

void palBulletMotor::UpdateRevolute(Float targetVelocity, Float Max) {
	m_revolute->m_btHinge->enableAngularMotor(true,targetVelocity,Max);
	m_revolute->m_btHinge->getRigidBodyA().activate();
	m_revolute->m_btHinge->getRigidBodyB().activate();
}

void palBulletMotor::DisableMotor() {
#if BT_BULLET_VERSION < 283
	typedef btRotationalLimitMotor RotationalMotor;
	typedef btTranslationalLimitMotor TranslationalMotor;
#else
	typedef btRotationalLimitMotor2 RotationalMotor;
	typedef btTranslationalLimitMotor2 TranslationalMotor;
#endif
	if (m_revolute != 0) {
		m_revolute->m_btHinge->enableAngularMotor(false, 0.0, 0.0);
	}
	else if (m_6dof != 0) {
		if (m_axis > 2)
		{
			RotationalMotor* motor = m_6dof->BulletGetGenericConstraint()->getRotationalLimitMotor(m_axis - PAL_AXIS_COUNT);
			if (!motor->isLimited()) {
				motor->m_enableMotor = false;
			}
			motor->m_maxMotorForce = 0.0f;
			motor->m_targetVelocity = 0.0f;
		}
		else
		{
			TranslationalMotor* motor = m_6dof->BulletGetGenericConstraint()->getTranslationalLimitMotor();
			if (!motor->isLimited(m_axis)) {
				motor->m_enableMotor[m_axis] = false;
			}
			motor->m_maxMotorForce[m_axis] = 0.0f;
			motor->m_targetVelocity[m_axis] = 0.0f;
		}
	}
}

palLink *palBulletMotor::GetLink() const {
	if (m_revolute != 0) return m_revolute;
	if (m_6dof != 0) return m_6dof;
	return 0;
}

void palBulletMotor::Apply(float dt) {

}

/*
  if (m_pbtBody) {
  m_pbtBody->getWorldTransform().getOpenGLMatrix(m_mLoc._mat);
  }
  return m_mLoc;
 */

palBulletSoftBody::palBulletSoftBody()
: m_pbtSBody(0) {}

void palBulletSoftBody::BulletInit(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices) {

	palBulletPhysics *pbf=dynamic_cast<palBulletPhysics *>(PF->GetActivePhysics());

	const btScalar* particleArray;
#if (defined(DOUBLE_PRECISION) && defined(BT_USE_DOUBLE_PRECISION) || !defined(DOUBLE_PRECISION) && !defined(BT_USE_DOUBLE_PRECISION))
	particleArray = pParticles;
#else
	btScalar* tempArray = new btScalar[nParticles];
	for (int i = 0; i < nParticles; i++) {
		tempArray[i] = pParticles[i];
	}
	particleArray = tempArray;
#endif
	m_pbtSBody = btSoftBodyHelpers::CreateFromTriMesh(pbf->m_softBodyWorldInfo, particleArray, pIndices, nIndices/3);
	m_pbtSBody->generateBendingConstraints(2);
	m_pbtSBody->m_cfg.piterations=2;
	m_pbtSBody->m_cfg.collisions|=btSoftBody::fCollision::VF_SS;
	m_pbtSBody->randomizeConstraints();

	m_pbtSBody->setTotalMass(50,true);

	btSoftRigidDynamicsWorld* softWorld =	(btSoftRigidDynamicsWorld*)pbf->m_dynamicsWorld;
	softWorld->addSoftBody(m_pbtSBody);
}

int palBulletSoftBody::GetNumParticles() const {
	return (int)m_pbtSBody->m_nodes.size();
}
palVector3* palBulletSoftBody::GetParticlePositions() {
	pos.resize(GetNumParticles());
	for (int i=0;i<GetNumParticles();i++) {
		pos[i].x = Float(m_pbtSBody->m_nodes[i].m_x.x());
		pos[i].y = Float(m_pbtSBody->m_nodes[i].m_x.y());
		pos[i].z = Float(m_pbtSBody->m_nodes[i].m_x.z());
	}
	return &pos[0];
}

palBulletPatchSoftBody::palBulletPatchSoftBody() {
}

void palBulletPatchSoftBody::Init(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices) {
	palBulletSoftBody::BulletInit(pParticles,pMass,nParticles,pIndices,nIndices);
};

palBulletTetrahedralSoftBody::palBulletTetrahedralSoftBody() {
}

void palBulletTetrahedralSoftBody::Init(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices) {
	int *tris = ConvertTetrahedronToTriangles(pIndices,nIndices);
	palBulletSoftBody::BulletInit(pParticles,pMass,nParticles,tris,(nIndices/4)*12);
};

std::ostream& operator<<(std::ostream& out, const btVector3& v) {
	out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
	return out;
}

std::ostream& operator<<(std::ostream& out, const btMatrix3x3& matrix) {
	out << matrix.getRow(0) << "; " << matrix.getRow(1) << "; " << matrix.getRow(2);
	return out;
}

std::ostream& operator<<(std::ostream& out, const btTransform& xform) {
	// btTransform returns references, so copy it
	btVector3 origin = xform.getOrigin();
	btQuaternion rot = xform.getRotation();
	out << "[basis=" << xform.getBasis() << ", orig=" << origin << ", rot=" << rot << "]";
	return out;
}

std::ostream& operator<<(std::ostream& out, const btQuaternion& quat) {
	double degrees = quat.getAngle() / M_PI * 180;
	out << "[angle=" << degrees << "\370, axis=" << quat.getAxis() << "]";
	return out;
}

#ifdef STATIC_CALLHACK
void pal_bullet_call_me_hack() {
	printf("%s I have been called!!\n", __FILE__);
};
#endif


/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
