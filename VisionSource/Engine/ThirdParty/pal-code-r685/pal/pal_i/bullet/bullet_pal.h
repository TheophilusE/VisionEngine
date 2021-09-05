#ifndef BULLET_PAL_H
#define BULLET_PAL_H

#define BULLET_PAL_SDK_VERSION_MAJOR 0
#define BULLET_PAL_SDK_VERSION_MINOR 2
#define BULLET_PAL_SDK_VERSION_BUGFIX 1

//(c) Adrian Boeing 2006, see liscence.txt (BSD liscence)
/*
	Abstract:
		PAL - Physics Abstraction Layer. Bullet implementation.
		This enables the use of Bullet via PAL.
	Author:
		Adrian Boeing
	Revision History:
	Version 0.2.01: 16/04/09 - Soft body tetrahedron
	Version 0.2.00: 15/04/09 - Soft body cloth
	Version 0.1.06: 18/02/09 - Public set/get for Bullet functionality & documentation
	Version 0.1.05: 14/11/08 - Bugfixed generic link to support static bodies
	Version 0.1.04: 29/10/08 - Bugfixed collision detection body
	Version 0.1.03: 10/10/08 - Fixed revolute and spherical link limits and deconstructors.
	Version 0.1.02: 07/10/08 - Multithreaded disable macro (BULLET_SINGLETHREAD)
	Version 0.1.01: 30/09/08 - PAL Version
	Version 0.1.00: 24/09/08 - Static convex body
	Version 0.0.99: 05/09/08 - Updated for Bullet 2.70, multithreaded solver
	Version 0.0.98: 14/07/08 - Compound body finalize mass & inertia method
	Version 0.0.97: 06/07/08 - Collision detection raycast
	Version 0.0.96: 05/07/08 - Collision Detection initial
	Version 0.0.95: 26/05/08 - Collision group support
	Version 0.0.94: 03/05/08 - Static compound body
	Version 0.0.93: 09/04/08 - Angular Motor
	Version 0.0.92: 13/02/08 - Static box&sphere orientation fix
	Version 0.0.91: 26/12/07 - Static sphere, capsule
	Version 0.0.9 : 17/12/07 - Base body, compound body position fix, static box, base link support
	Version 0.0.87: 15/12/07 - Body deletion.
	Version 0.0.86: 20/11/07 - PSD fix.
	Version 0.0.85: 10/11/07 - Fixed orientated plane bug
	Version 0.0.84: 09/11/07 - Fixed geometery and body location bugs
	Version 0.0.83: 07/11/07 - Added compound body
	Version 0.0.82: 28/10/07 - Updated for Bullet 2.62RC2
	Version 0.0.81: 19/10/07 - Version number request, new force system
	Version 0.0.8 : 17/10/07 - Added Generic Constraint
	Version 0.0.7 : 15/10/07 - Added PSD sensor
	Version 0.0.6 : 18/08/07 - Convex geom and body and vehicle
	Version 0.0.54: 01/08/07 - Updated for Bullet 2.55
	Version 0.0.53: 25/07/07 - Orientated plane
	Version 0.0.52: 15/07/07 - body sleep
	Version 0.0.51: 22/06/07 - body set velocity linear & angular
	Version 0.0.5 : 10/05/06 - Update for Bullet 2.50
	Version 0.0.4 : 17/11/06 - materials, terrain heightmap
	Version 0.0.3 : 16/11/06 - terrain mesh, spherical, revolute and prismatic link.
	Version 0.0.2 : 14/11/06 - boxgeom fix, sphere geom, cylinder geom, sphere, cylinder, terrainplane
	Version 0.0.1 : 13/11/06 - physics, body, boxgeom, box
	TODO:
		- raycasts
		- fix prismatic link config
		- link limits
		- collision accuracy
		- sawp terrainplane to use btStaticPlaneShape
	notes:
 */

#include <pal/palFactory.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <pal/palActivation.h>
#include <pal/palCollision.h>
#include <pal/palSolver.h>
#include <pal/palSoftBody.h>
#include <iosfwd>

#if defined(_MSC_VER)
#pragma warning(disable : 4250)
#endif

#ifdef DOUBLE_PRECISION
#ifdef BT_USE_DOUBLE_PRECISION
#define BT_SCALAR_IS_PAL_FLOAT 1
#else
#define BT_SCALAR_IS_PAL_FLOAT 0
#endif
#else
#ifdef BT_USE_DOUBLE_PRECISION
#define BT_SCALAR_IS_PAL_FLOAT 0
#else
#define BT_SCALAR_IS_PAL_FLOAT 1
#endif
#endif

class palBulletBodyBase;
class palBulletDebugDraw;

/** Bullet Physics Class
	Additionally Supports:
		- Collision Detection
		- Solver System
 */
class palBulletPhysics: public palPhysics, public palCollisionDetectionExtended, public palSolver {
	friend class palBulletSoftBody;
public:
	palBulletPhysics();


	/*override*/ void GetPropertyDocumentation(PAL_MAP<PAL_STRING, PAL_STRING>& docOut) const;
	/*override*/ void Init(const palPhysicsDesc& desc);
	/*override*/ void Cleanup();
	/*override*/ const char* GetPALVersion() const;
	/*override*/ const char* GetVersion() const;
	/*override*/ palCollisionDetection* asCollisionDetection() { return this; }
	/*override*/ palSolver* asSolver() { return this; }
	//extra methods provided by Bullet abilities:
	/** Returns the current Bullet World in use by PAL
		\return A pointer to the current btDynamicsWorld
	 */
	btDynamicsWorld* BulletGetDynamicsWorld() {return m_dynamicsWorld;}
	/** Returns the current Bullet Collision Dispatcher in use by PAL
		\return A pointer to the current btCollisionDispatcher
	 */
	btCollisionDispatcher* BulletGetCollsionDispatcher() {return m_dispatcher;}

	//colision detection functionality
	virtual void SetCollisionAccuracy(Float fAccuracy);
	virtual void SetGroupCollision(palGroup a, palGroup b, bool enabled);
	virtual void RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range, palRayHit& hit) const;
	virtual void RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range,
			palRayHitCallback& callback, palGroupFlags groupFilter = ~0) const;
	virtual void NotifyCollision(palBodyBase *a, palBodyBase *b, bool enabled);
	virtual void NotifyCollision(palBodyBase *pBody, bool enabled);
	void CleanupNotifications(palBodyBase *pBody);

	//solver functionality
	virtual void SetSolverAccuracy(Float fAccuracy) ;//0 - fast, higher - accurate
	virtual float GetSolverAccuracy() const;
	virtual void StartIterate(Float timestep);
	virtual bool QueryIterationComplete() const;
	virtual void WaitForIteration();
	virtual void SetFixedTimeStep(Float fixedStep);
	virtual void SetPE(int n);
	virtual void SetSubsteps(int n);
	virtual void SetHardware(bool status);
	virtual bool GetHardware(void) const;

	void AddRigidBody(palBulletBodyBase* body);
	void RemoveRigidBody(palBulletBodyBase* body);
	void ClearBroadPhaseCachePairs(palBulletBodyBase* body);

	// This is a helper function to keep track of constraints being removed
	void AddBulletConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies);
	// This is a helper function to keep track of constraints being removed
	void RemoveBulletConstraint(btTypedConstraint* constraint);

	virtual void AddAction(palAction *action);
	virtual void RemoveAction(palAction *action);

	PAL_VECTOR<short> m_CollisionMasks;

protected:

	virtual void Iterate(Float timestep);
	virtual void CallActions(Float timestep);

	Float m_fFixedTimeStep;
	int set_substeps;
	int set_pe;

	btDiscreteDynamicsWorld*	m_dynamicsWorld;
	btSoftBodyWorldInfo		m_softBodyWorldInfo;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver* m_solver;
	btCollisionConfiguration* m_collisionConfiguration;
	btOverlapFilterCallback* m_overlapCallback;
	btOverlappingPairCallback* m_ghostPairCallback;
	palBulletDebugDraw*	m_pbtDebugDraw;

	// map of pal actions to bullet actions so they can be cleaned up.
	PAL_MAP<palAction*, btActionInterface*> m_BulletActions;

	FACTORY_CLASS(palBulletPhysics,palPhysics,Bullet,1)
};

/** Bullet Body Base Class
 */
class palBulletBodyBase : virtual public palBodyBase {
	friend class palBulletPhysics;
	friend class palBulletRevoluteSphericalLink;
	friend class palBulletSphericalLink;
	friend class palBulletPrismaticLink;
	friend class palBulletGenericLink;
	friend class palBulletRigidLink;
public:
	palBulletBodyBase();
	virtual ~palBulletBodyBase();
	virtual const palMatrix4x4& GetLocationMatrix() const;
	virtual const palMatrix4x4& GetLocationMatrixInterpolated() const;
	virtual void SetPosition(const palMatrix4x4& location);
	virtual void SetMaterial(palMaterial *material);
	virtual palGroup GetGroup() const;
	virtual void SetGroup(palGroup group);

	virtual Float GetSkinWidth() const;
	virtual bool SetSkinWidth(Float skinWidth);

	//Bullet specific:
	/** Returns the Bullet Body associated with the PAL body
		\return A pointer to the btRigidBody
	 */
	btRigidBody *BulletGetRigidBody() const {return m_pbtBody;}

protected:
	btRigidBody *m_pbtBody;
	void BuildBody(const palMatrix4x4& pos, Float mass,
			palDynamicsType dynType = PALBODY_DYNAMIC,
			btCollisionShape *btShape = NULL,
			const palVector3& inertia = palVector3(1.0f, 1.0f, 1.0f));
	void AssignDynamicsType(palDynamicsType dynType, Float mass,
			const btVector3& inertia);
	/** Internally, sometimes we want a Bullet-style transform, and it's faster to
	 * get it directly than convert from the PAL representation. */
	virtual const btTransform GetWorldTransform() const;

	Float m_fSkinWidth;
};

class palBulletBody : virtual public palBulletBodyBase, virtual public palBody,
virtual public palActivationSettings {
public:
	palBulletBody();
	virtual ~palBulletBody();
	//	virtual void SetPosition(palMatrix4x4& location);
	//	virtual palMatrix4x4& GetLocationMatrix();
	//	virtual void SetMaterial(palMaterial *material);

	//	virtual void SetForce(Float fx, Float fy, Float fz);
	//	virtual void GetForce(palVector3& force);
	//	virtual void SetTorque(Float tx, Float ty, Float tz);
	//	virtual void GetTorque(palVector3& torque);

	virtual void ApplyForce(Float fx, Float fy, Float fz);
	virtual void ApplyTorque(Float tx, Float ty, Float tz);

	virtual void ApplyImpulse(Float fx, Float fy, Float fz);
	virtual void ApplyAngularImpulse(Float fx, Float fy, Float fz);

	virtual void GetLinearVelocity(palVector3& velocity) const;
	virtual void GetAngularVelocity(palVector3& velocity_rad) const;

	virtual void SetLinearVelocity(const palVector3& velocity);
	virtual void SetAngularVelocity(const palVector3& velocity_rad);

	//@return if the body is active or sleeping
	virtual bool IsActive() const;

	virtual void SetActive(bool active);

	virtual palActivationSettings* asActivationSettings() { return this; }
	// palActivation implementation
	virtual Float GetActivationLinearVelocityThreshold() const;
	virtual void SetActivationLinearVelocityThreshold(Float);

	virtual Float GetActivationAngularVelocityThreshold() const;
	virtual void SetActivationAngularVelocityThreshold(Float);

	virtual Float GetActivationTimeThreshold() const;
	virtual void SetActivationTimeThreshold(Float);

	virtual const std::bitset <DUMMY_ACTIVATION_SETTING_TYPE>& GetSupportedActivationSettings() const;
protected:
	//	void BuildBody(Float fx, Float fy, Float fz, Float mass);
private:
	static const std::bitset<DUMMY_ACTIVATION_SETTING_TYPE> SUPPORTED_SETTINGS;
};

class palBulletGenericBody : virtual public palBulletBody, public palGenericBody {
public:
	palBulletGenericBody();
	virtual ~palBulletGenericBody();
	virtual void Init(const palMatrix4x4 &pos);
	virtual void SetDynamicsType(palDynamicsType dynType);
	virtual void SetGravityEnabled(bool enabled);
	virtual bool IsGravityEnabled() const;

	virtual void SetCollisionResponseEnabled(bool enabled);
	virtual bool IsCollisionResponseEnabled() const;

	virtual void SetMass(Float mass);
	virtual void SetInertia(Float Ixx, Float Iyy, Float Izz);

	virtual void SetLinearDamping(Float);
	virtual Float GetLinearDamping() const;

	virtual void SetAngularDamping(Float);
	virtual Float GetAngularDamping() const;

	virtual void SetMaxAngularVelocity(Float maxAngVel);
	virtual Float GetMaxAngularVelocity() const;

	virtual void ConnectGeometry(palGeometry* pGeom);
	virtual void RemoveGeometry(palGeometry* pGeom);
	virtual bool IsDynamic() const;
	virtual bool IsKinematic() const;
	virtual bool IsStatic() const;

	//using palBulletBody::GetActive;
	using palBulletBody::SetActive;
	using palBulletBody::IsActive;
	using palBulletBody::SetAngularVelocity;
	using palBulletBody::GetAngularVelocity;
	using palBulletBody::GetLinearVelocity;
	using palBulletBody::SetLinearVelocity;
	using palBulletBody::GetLocationMatrix;
protected:
	FACTORY_CLASS(palBulletGenericBody, palGenericBody, Bullet, 1);
	void AddShapeToCompound(palGeometry* pGeom);
	void RemoveShapeFromCompound(palGeometry* pGeom);
	void RebuildConcaveShapeFromGeometry();
	bool IsUsingConcaveShape() const;
	bool IsUsingOneCenteredGeometry() const;
	void InitCompoundIfNull();
private:
	bool m_bGravityEnabled;
	btCompoundShape* m_pCompound;
	btBvhTriangleMeshShape* m_pConcave;
};

/** Bullet Geometry Class
 */
class palBulletGeometry : virtual public palGeometry {
	friend class palBulletBodyBase;
public:
	palBulletGeometry();
	virtual ~palBulletGeometry();
	//Bullet specific:
	/** Returns the Bullet Collision Shape used by PAL geometry
		\return A pointer to the btCollisionShape
	 */
	btCollisionShape* BulletGetCollisionShape() {return m_pbtShape;}
	virtual Float GetMargin() const;
	virtual bool SetMargin(Float margin);
protected:
	btCollisionShape* m_pbtShape;
};

class palBulletBoxGeometry : public palBulletGeometry, public palBoxGeometry  {
public:
	palBulletBoxGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float width, Float height, Float depth, Float mass);
	using palBoxGeometry::CalculateInertia;
	btBoxShape *m_pbtBoxShape; // freed by our superclass
protected:
	FACTORY_CLASS(palBulletBoxGeometry,palBoxGeometry,Bullet,1)
};

class palBulletSphereGeometry : public palSphereGeometry, public palBulletGeometry {
public:
	palBulletSphereGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float mass);
	using palSphereGeometry::CalculateInertia;
	btSphereShape *m_btSphereShape;
protected:
	FACTORY_CLASS(palBulletSphereGeometry,palSphereGeometry,Bullet,1)
};

class palBulletCapsuleGeometry : public palCapsuleGeometry, public palBulletGeometry {
public:
	palBulletCapsuleGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass);
	using palCapsuleGeometry::CalculateInertia;
	btCapsuleShape *m_btCapsuleShape;
protected:
	FACTORY_CLASS(palBulletCapsuleGeometry,palCapsuleGeometry,Bullet,1)
};

class palBulletCylinderGeometry : public palCylinderGeometry, public palBulletGeometry {
public:
	palBulletCylinderGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass);
	using palCylinderGeometry::CalculateInertia;
	btCylinderShape *m_btCylinderShape;
protected:
	FACTORY_CLASS(palBulletCylinderGeometry,palCylinderGeometry,Bullet,1)
};

class palBulletTerrainPlane : public palTerrainPlane, virtual public palBulletBodyBase  {
public:
	palBulletTerrainPlane();
	virtual void Init(Float x, Float y, Float z, Float min_size);
	using palBulletBodyBase::GetLocationMatrix;
protected:
	btBoxShape *m_pbtBoxShape;
	FACTORY_CLASS(palBulletTerrainPlane,palTerrainPlane,Bullet,1)
};


class palBulletOrientatedTerrainPlane : public palOrientatedTerrainPlane, virtual public palBulletBodyBase  {
public:
	palBulletOrientatedTerrainPlane();
	virtual ~palBulletOrientatedTerrainPlane();
	virtual void Init(Float x, Float y, Float z, Float nx, Float ny, Float nz, Float min_size);
	// "using" keyword for this gave a compiler error
	virtual const palMatrix4x4& GetLocationMatrix() const {
		return palOrientatedTerrainPlane::GetLocationMatrix();
	}
public:
	btStaticPlaneShape *m_pbtPlaneShape;
	FACTORY_CLASS(palBulletOrientatedTerrainPlane,palOrientatedTerrainPlane,Bullet,1)
};

class palBulletTerrainMesh : public palTerrainMesh, virtual public palBulletBodyBase  {
public:
	palBulletTerrainMesh();
	virtual ~palBulletTerrainMesh();
	virtual void Init(Float x, Float y, Float z, const Float *pVertices, int nVertices, const int *pIndices, int nIndices);
	using palBulletBodyBase::GetLocationMatrix;
protected:
	btBvhTriangleMeshShape *m_pbtTriMeshShape;
	PAL_VECTOR<int> m_Indices;
	PAL_VECTOR<Float> m_Vertices;
	FACTORY_CLASS(palBulletTerrainMesh,palTerrainMesh,Bullet,1)
};

class palBulletTerrainHeightmap : public palTerrainHeightmap, private palBulletTerrainMesh {
public:
	palBulletTerrainHeightmap();
	virtual void Init(Float x, Float y, Float z, Float width, Float depth, int terrain_data_width, int terrain_data_depth, const Float *pHeightmap);
	using palBulletBodyBase::GetLocationMatrix;
protected:
	FACTORY_CLASS(palBulletTerrainHeightmap,palTerrainHeightmap,Bullet,1)
};


class palBulletConvexGeometry : public palBulletGeometry, public palConvexGeometry  {
public:
	palBulletConvexGeometry();
	virtual ~palBulletConvexGeometry() {};
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, Float mass);
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass);
protected:
	using palConvexGeometry::CalculateInertia;
	btConvexHullShape *m_pbtConvexShape;
	void InternalInit(const Float *pVertices, unsigned int nVertices, const int *pIndices, int nIndices);
	FACTORY_CLASS(palBulletConvexGeometry,palConvexGeometry,Bullet,1)
};

class btTriangleInfoMap;

class palBulletConcaveGeometry : public palBulletGeometry, public palConcaveGeometry  {
public:
	palBulletConcaveGeometry();
	virtual ~palBulletConcaveGeometry();
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass);
protected:
	btBvhTriangleMeshShape *m_pbtTriMeshShape;
	using palConcaveGeometry::CalculateInertia;
	btTriangleInfoMap* m_pInternalEdgeInfo;
	FACTORY_CLASS(palBulletConcaveGeometry,palConcaveGeometry,Bullet,1)
};

class CustomBulletConcaveShape;

class palBulletCustomConcaveGeometry : public palBulletGeometry, public palCustomConcaveGeometry  {
public:
	palBulletCustomConcaveGeometry();
	virtual ~palBulletCustomConcaveGeometry();
	virtual void Init(const palMatrix4x4& pos, Float mass, palCustomGeometryCallback& callback);
protected:
	CustomBulletConcaveShape *m_pCustomShape;
	using palCustomConcaveGeometry::CalculateInertia;
	btTriangleInfoMap* m_pInternalEdgeInfo;
	FACTORY_CLASS(palBulletCustomConcaveGeometry,palCustomConcaveGeometry,Bullet,1)
};


class palBulletPSDSensor : public palPSDSensor {
public:
	palBulletPSDSensor();
	virtual void Init(palBody *body, Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range); //position, direction
	virtual Float GetDistance() const;
protected:
	Float m_fRelativePosX;
	Float m_fRelativePosY;
	Float m_fRelativePosZ;
	FACTORY_CLASS(palBulletPSDSensor,palPSDSensor,Bullet,1)
};

class palBulletRevoluteLink;
class palBulletGenericLink;

/**
 * This works with both revolute and 6dof links.  The axis only does anything with the 6dof.
 */
class palBulletMotor : public palMotor {
public:
	palBulletMotor();
	virtual ~palBulletMotor() { DisableMotor(); }
	virtual void Init(palLink *pLink, int axis = -1);
	virtual void Update(Float targetVelocity, Float Max);
	virtual void DisableMotor();
	virtual void Apply(float dt);
	virtual palLink *GetLink() const;
protected:
	void Update6DOF(Float targetVelocity, Float Max);
	void UpdateRevolute(Float targetVelocity, Float Max);
private:
	void (palBulletMotor::* m_updateFunc) (Float, Float);
	palBulletRevoluteLink *m_revolute;
	palBulletGenericLink *m_6dof;
	int m_axis; // for 6dof.
	FACTORY_CLASS(palBulletMotor,palMotor,Bullet,1)
};

class palBulletSoftBody : virtual public palSoftBody {
public:
	palBulletSoftBody();

	virtual const palMatrix4x4& GetLocationMatrix() const {return m_mLoc;};
	virtual void GetLinearVelocity(palVector3& velocity) const {};

	virtual void GetAngularVelocity(palVector3& velocity_rad) const {};

	virtual void SetLinearVelocity(const palVector3& velocity) {};

	virtual void SetAngularVelocity(const palVector3& velocity_rad) {};

	virtual bool IsActive() const {return true;}

	virtual void SetActive(bool active) {};

	virtual int GetNumParticles() const;
	virtual palVector3* GetParticlePositions();

	btSoftBody* m_pbtSBody;
protected:
	void BulletInit(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices);
private:
	// cache used by GetParticlePositions
	mutable PAL_VECTOR<palVector3> pos;
};

class palBulletPatchSoftBody: public palPatchSoftBody, public palBulletSoftBody  {
public:
	palBulletPatchSoftBody();
	virtual void Init(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices);
	virtual void SetIterations(const int nIterations) {};
	void SetAngularVelocity();
	using palBulletSoftBody::GetAngularVelocity;
	using palBulletSoftBody::SetAngularVelocity;
	using palBulletSoftBody::GetParticlePositions;
	using palBulletSoftBody::GetLinearVelocity;
	using palBulletSoftBody::SetLinearVelocity;
	using palBulletSoftBody::GetLocationMatrix;
	using palBulletSoftBody::GetNumParticles;
	using palBulletSoftBody::IsActive;
	using palBulletSoftBody::SetActive;
protected:
	FACTORY_CLASS(palBulletPatchSoftBody,palPatchSoftBody,Bullet,1)
};


class palBulletTetrahedralSoftBody : public palTetrahedralSoftBody, public palBulletSoftBody  {
public:
	palBulletTetrahedralSoftBody();
	virtual void Init(const Float *pParticles, const Float *pMass, const int nParticles, const int *pIndices, const int nIndices);
	using palBulletSoftBody::GetAngularVelocity;
	using palBulletSoftBody::SetAngularVelocity;
	using palBulletSoftBody::GetParticlePositions;
	using palBulletSoftBody::GetLinearVelocity;
	using palBulletSoftBody::SetLinearVelocity;
	using palBulletSoftBody::GetLocationMatrix;
	using palBulletSoftBody::GetNumParticles;
	using palBulletSoftBody::IsActive;
	using palBulletSoftBody::SetActive;
protected:
	FACTORY_CLASS(palBulletTetrahedralSoftBody,palTetrahedralSoftBody,Bullet,1)
};

inline short int convert_group(palGroup group) {
	short int btgroup = 1 << group;
	//We don't need to use the built in group set, it's optional, and not exposed in pal.
	//btgroup = btgroup << 5;

	if (btgroup == 0)
	{
		return 1;
	}
	return btgroup;
}

inline palGroup convert_to_pal_group(short int v)
{
	static const unsigned int b[] = {0xAAAAAAAA, 0xCCCCCCCC, 0xF0F0F0F0,
			0xFF00FF00, 0xFFFF0000};
	palGroup r = (v & b[0]) != 0;
	for (unsigned i = 3; i > 0; i--)
	{
		r |= ((v & b[i]) != 0) << i;
	}
	return r;
}

// convenient for debugging
extern std::ostream& operator<<(std::ostream& out, const btVector3& v);
extern std::ostream& operator<<(std::ostream& out, const btTransform& xform);
extern std::ostream& operator<<(std::ostream& out, const btQuaternion& quat);

inline void convertManifoldPtToContactPoint(btManifoldPoint& pt, palContactPoint& cp)
{
	btVector3 pos = pt.getPositionWorldOnB();
	cp.m_vContactPosition.x = pos.x();
	cp.m_vContactPosition.y = pos.y();
	cp.m_vContactPosition.z = pos.z();

	btVector3 norm = pt.m_normalWorldOnB;
	cp.m_vContactNormal.x = norm.x();
	cp.m_vContactNormal.y = norm.y();
	cp.m_vContactNormal.z = norm.z();

	cp.m_fDistance= pt.getDistance();
	cp.m_fImpulse= pt.getAppliedImpulse();

	if (pt.m_lateralFrictionInitialized)
	{
		for (unsigned i = 0; i < 3; ++i)
		{
			cp.m_vImpulseLateral1[i] = pt.m_lateralFrictionDir1[i] * pt.m_appliedImpulseLateral1;
			cp.m_vImpulseLateral2[i] = pt.m_lateralFrictionDir2[i] * pt.m_appliedImpulseLateral2;
		}
	}
}

// This is only a partial conversion.  It assumes you are only updating a manfold point.
inline void convertContactPointToManifoldPt(palContactPoint& cp, btManifoldPoint& pt)
{
	palVector3 lat1 = cp.m_vImpulseLateral1;
	palVector3 lat2 = cp.m_vImpulseLateral2;
	Float latMag1 = vec_norm(&lat1);
	Float latMag2 = vec_norm(&lat2);

	for (unsigned i = 0; i < 3; ++i)
	{
		pt.m_positionWorldOnB[i] = cp.m_vContactPosition[i];
		pt.m_normalWorldOnB[i] = cp.m_vContactNormal[i];
		pt.m_lateralFrictionDir1[i] = lat1[i];
		pt.m_lateralFrictionDir2[i] = lat2[i];
	}

	pt.m_appliedImpulseLateral1 = latMag1;
	pt.m_appliedImpulseLateral2 = latMag2;

	pt.m_distance1 = cp.m_fDistance;
	pt.m_appliedImpulse = cp.m_fImpulse;

}

inline void convertPalMatToBtTransform(btTransform& xform, const palMatrix4x4& palMat)
{
	const btScalar* openGlMatrix;
#if BT_SCALAR_IS_PAL_FLOAT
	openGlMatrix = palMat._mat;
#else
	btScalar mat[4*4];
	for (unsigned i = 0; i < 16; ++i) {
		mat[i] = palMat._mat[i];
	}
	openGlMatrix = mat;
#endif
	xform.setFromOpenGLMatrix(openGlMatrix);
}

inline void convertBtTransformToPalMat(palMatrix4x4& palMat, const btTransform& xform)
{
//#if BT_SCALAR_IS_PAL_FLOAT
//	xform.getOpenGLMatrix(palMat._mat);
//#else
	btScalar mat[4*4];
	xform.getOpenGLMatrix(mat);
	for (unsigned i = 0; i < 16; ++i) {
		palMat._mat[i] = Float(mat[i]);
	}
//#endif
}

#ifdef STATIC_CALLHACK
#include "bullet_pal_static_include.h"
#endif

#endif
