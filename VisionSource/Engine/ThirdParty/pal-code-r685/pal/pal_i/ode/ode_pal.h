#ifndef ODE_PAL_H
#define ODE_PAL_H

#define ODE_PAL_SDK_VERSION_MAJOR 0
#define ODE_PAL_SDK_VERSION_MINOR 1
#define ODE_PAL_SDK_VERSION_BUGFIX 9

//(c) Adrian Boeing 2004, see liscence.txt (BSD liscence)
/*
	Abstract:
		PAL - Physics Abstraction Layer. ODE implementation.
		This enables the use of ODE via PAL.
	Author:
		Adrian Boeing
	Revision History:
		Version 0.1.11: 06/26/14 - DG - deleted the subclass of materials and added support for custom material callbacks.
		Version 0.1.10: 16/09/09 - AB: Fixed some bugs, introduced a new bug to the compound body (4x3 vs 4x4)
		Version 0.1.09: 18/02/09 - Public set/get for ODE functionality & documentation
		Version 0.1.08: 30/09/08 - PAL Version
		Version 0.1.07: 23/07/08 - Collision detection subsytem
		Version 0.1.06: 15/07/08 - Update for ODE 0.10.0 & dInitODE2 bugfix, staticbox deconstructor
		Version 0.1.05: 13/07/08 - Compound body finalize mass & inertia method
		Version 0.1.04: 26/05/08 - Collision group support
		Version 0.1.03: 04/05/08 - Static box, compound body, joints attach fix
		Version 0.1.02: 10/04/08 - ODE Joint, Angular Motor
		Version 0.1.01: 17/01/08 - Plane fix, TriMesh compatibility
		Version 0.1.0 : 13/01/08 - Capsule fix.
		Version 0.0.99: 19/11/07 - Include compound body.
		Version 0.0.98: 18/11/07 - Fix of static terrain, include convex bodys
		Version 0.0.97: 19/10/07 - Version number request
		Version 0.0.96: 25/07/07 - Orientated plane
		Versoin 0.0.95: 15/07/07 - Body sleep
		Versoin 0.0.94: 22/06/07 - Set angular & linear velocities
		Version 0.0.93: 22/12/04 - SetPosition update
		Version 0.0.92: 05/09/04 - Impulse
		Version 0.0.91: 12/08/04 - Revolute link add torque & get angle
		Version 0.0.8 : 12/07/04 - Geometries (sphere,cylinder,box) & body updates, bug fixes
		Version 0.0.72: 21/06/04 - Revolute joint limits & spherical joint limits
		Version 0.0.71: 15/06/04 - started prismatic link (anchor fix?)
		Version 0.0.7 : 12/06/04 - added 'cleanup' code, no more memory leaks, terrain plane, started terrain heightmap
		Version 0.0.6 : 09/06/04 - sphere, cylinder, link, spherical link, revolute link
		Version 0.0.5 : 04/06/04 - physics, box
	TODO:
		-collision ray body and distance
		-collision notify between two specific bodies
		-fix cylinder inertia matrix calc => build from pal inertias?
	notes:
 */

#include <pal/pal.h>
#include <pal/palFactory.h>
#include <pal/palCollision.h>
#include <pal/palActivation.h>

#include <ode/ode.h>

#if defined(_MSC_VER)
#pragma warning(disable : 4250)

//#ifndef NDEBUG
//#if defined(dSINGLE)
//#pragma comment( lib, "ode_singled.lib" )
//#else
//#pragma comment( lib, "ode_doubled.lib" )
//#endif
//#else
//#if defined(dSINGLE)
//#pragma comment( lib, "ode_single.lib" )
//#else
//#pragma comment( lib, "ode_double.lib" )
//#endif
//#endif

#endif //_MSC_VER

#define ODE_MATINDEXLOOKUP int

/** ODE Physics Class
	Additionally Supports:
		- Collision Detection
 */
class palODEPhysics: public palPhysics, public palCollisionDetectionExtended {
public:
	palODEPhysics();
	virtual void Init(const palPhysicsDesc& desc);
	void SetGravity(Float gravity_x, Float gravity_y, Float gravity_z);

	/*override*/ void GetPropertyDocumentation(PAL_MAP<PAL_STRING, PAL_STRING>& docOut) const;

	//colision detection functionality
	virtual void SetCollisionAccuracy(Float fAccuracy);

	virtual void SetGroupCollision(palGroup a, palGroup b, bool enabled);
	void SetGroupCollisionOnGeom(unsigned long bits, unsigned long otherBits, dGeomID geom, bool collide);

	virtual void RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range, palRayHit& hit) const;
	virtual void RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz,
			Float range, palRayHitCallback& callback, palGroupFlags groupFilter = ~0) const;
	virtual void NotifyCollision(palBodyBase *a, palBodyBase *b, bool enabled);
	virtual void NotifyCollision(palBodyBase *pBody, bool enabled);
	void CleanupNotifications(palBodyBase* geom);

	//	void SetDefaultMaterial(palMaterial *pmat);
	//	void SetGroundPlane(bool enabled, Float size);
	const char* GetPALVersion() const;
	virtual const char* GetVersion() const;
	virtual palCollisionDetection* asCollisionDetection() { return this; }

	//ODE specific:
	/** Returns the current ODE World in use by PAL
		\return A pointer to the current ODE dWorldID
	 */
	dWorldID ODEGetWorld() const;
	/** Returns the current ODE Space in use by PAL
		\return A pointer to the current ODE dSpaceID
	 */
	dSpaceID ODEGetSpace() const;

	virtual void Cleanup();

	PAL_VECTOR<unsigned long> m_CollisionMasks;

protected:
	void Iterate(Float timestep);

	FACTORY_CLASS(palODEPhysics,palPhysics,ODE,1)
	bool m_initialized;
};

/** The ODE Body class
 */
class palODEBody : virtual public palBody, virtual public palActivationSettings
{
	friend class palODERigidLink;
	friend class palODERevoluteLink;
	friend class palODESphericalLink;
	friend class palODEPrismaticLink;
	friend class palODEBoxGeometry;
	friend class palODESphereGeometry;
	friend class palODECapsuleGeometry;
	friend class palODECylinderGeometry;
	friend class palODEConvexGeometry;
	friend class palODEConcaveGeometry;
public:
	palODEBody();
	virtual ~palODEBody();
	virtual void SetPosition(Float x, Float y, Float z);
	virtual void SetPosition(const palMatrix4x4& location);
#if 0
	virtual void SetForce(Float fx, Float fy, Float fz);
	virtual void GetForce(palVector3& force) const;

	virtual void AddForce(Float fx, Float fy, Float fz);
	virtual void AddTorque(Float tx, Float ty, Float tz);

	virtual void SetTorque(Float tx, Float ty, Float tz);
	virtual void GetTorque(palVector3& torque);

	virtual void ApplyImpulse(Float fx, Float fy, Float fz);
	virtual void ApplyAngularImpulse(Float ix, Float iy, Float iz);
#endif

	virtual void ApplyForce(Float fx, Float fy, Float fz);
	virtual void ApplyTorque(Float tx, Float ty, Float tz);

	virtual void GetLinearVelocity(palVector3& velocity) const;
	virtual void GetAngularVelocity(palVector3& velocity_rad) const;

	virtual void SetLinearVelocity(const palVector3& velocity);
	virtual void SetAngularVelocity(const palVector3& velocity_rad);

	//@return if the body is active or sleeping
	virtual bool IsActive() const;

	virtual void SetActive(bool active);

	virtual void SetGroup(palGroup group);

	//virtual void a() {};
	virtual const palMatrix4x4& GetLocationMatrix() const;

	virtual palActivationSettings* asActivationSettings() { return this; }

	/***** Pal Activation ****/
	virtual Float GetActivationLinearVelocityThreshold() const;
	virtual void SetActivationLinearVelocityThreshold(Float);

	virtual Float GetActivationAngularVelocityThreshold() const;
	virtual void SetActivationAngularVelocityThreshold(Float);

	virtual Float GetActivationTimeThreshold() const;
	virtual void SetActivationTimeThreshold(Float);

	virtual const std::bitset<DUMMY_ACTIVATION_SETTING_TYPE>& GetSupportedActivationSettings() const;
	/***** Pal Activation ****/

	//ODE specific:
	/** Returns the ODE body associated with the PAL body
		\return The ODE dBodyID
	 */
	dBodyID ODEGetBody() const {return odeBody;}
	/**
	 *  Same as IsCollisionResponseEnabled.  Added a fast inline so the internal code won't have to call a virtual method
	 *  for every potential collision
	 */
	inline bool ODEGetCollisionResponseEnabled() const { return m_bCollisionResponseEnabled; }
protected:
	dBodyID odeBody; // the ODE body
	bool m_bCollisionResponseEnabled;
protected:
	void BodyInit(Float x, Float y, Float z);
	virtual void SetGeometryBody(palGeometry *pgeom);
	void RecalcMassAndInertia();
	virtual void CreateODEBody();
private:
	static const std::bitset<DUMMY_ACTIVATION_SETTING_TYPE> SUPPORTED_SETTINGS;
};

/** The ODE Geometry class
 */
class palODEGeometry : virtual public palGeometry {
	friend class palODEPhysics;
	friend class palODEBody;
public:
	palODEGeometry();
	virtual ~palODEGeometry();
	virtual const palMatrix4x4& GetLocationMatrix() const; //unfinished!
	//ode abilites:
	virtual void SetPosition(const palMatrix4x4 &pos);
	//ODE specific:
	/** Returns the ODE geometry associated with the PAL geometry
		\return The ODE dGeomID
	 */
	dGeomID ODEGetGeom() const {return odeGeom;}

	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const = 0;

protected:
	virtual void ReCalculateOffset();
	dGeomID odeGeom; // the ODE geometries representing this body
};

class palODEBoxGeometry : virtual public palBoxGeometry, virtual public palODEGeometry {
public:
	palODEBoxGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float width, Float height, Float depth, Float mass);
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	FACTORY_CLASS(palODEBoxGeometry,palBoxGeometry,ODE,1)
};

class palODESphereGeometry : virtual public palSphereGeometry, virtual public palODEGeometry {
public:
	palODESphereGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float mass);
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	FACTORY_CLASS(palODESphereGeometry,palSphereGeometry,ODE,1)
};

class palODECapsuleGeometry : virtual public palCapsuleGeometry, virtual public palODEGeometry {
public:
	palODECapsuleGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass);
	virtual const palMatrix4x4& GetLocationMatrix() const;
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	virtual void ReCalculateOffset();
	FACTORY_CLASS(palODECapsuleGeometry,palCapsuleGeometry,ODE,1)
private:
	unsigned int m_upAxis;
};

class palODECylinderGeometry : virtual public palCylinderGeometry, virtual public palODEGeometry {
public:
	palODECylinderGeometry();
	virtual void Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass);
	virtual const palMatrix4x4& GetLocationMatrix() const;
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	virtual void ReCalculateOffset();
	FACTORY_CLASS(palODECylinderGeometry,palCylinderGeometry,ODE,1)
private:
	unsigned int m_upAxis;
};

class palODEConvexGeometry : virtual public palConvexGeometry, virtual public palODEGeometry  {
public:
	palODEConvexGeometry();
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, Float mass);
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass);
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	FACTORY_CLASS(palODEConvexGeometry,palConvexGeometry,ODE,1)
};

class palODEConcaveGeometry : virtual public palConcaveGeometry, virtual public palODEGeometry  {
public:
	palODEConcaveGeometry();
	virtual void Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass);
	virtual void CalculateMassParams(dMass& odeMass, Float massScalar) const;
protected:
	FACTORY_CLASS(palODEConcaveGeometry,palConcaveGeometry,ODE,1)
};


class palODEGenericBody : virtual public palODEBody, virtual public palGenericBody {
public:
	palODEGenericBody();

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

protected:
	FACTORY_CLASS(palODEGenericBody, palGenericBody, ODE, 1);
};

/** The ODE Link class
 */
class palODELinkData {
public:
	palODELinkData();
	virtual ~palODELinkData();

	//ODE specific:
	/** Returns the ODE joint associated with the PAL link
		\return The ODE dJointID
	 */
	dJointID ODEGetJointID() const {
		return odeJoint;
	}

protected:
	dJointID odeJoint; //the ODE joint
	dJointID odeMotorJoint; //the ODE motorised joint
};

class palODESphericalLink : public palSphericalLink, public palODELinkData {
public:
	palODESphericalLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const;
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const;

	void GetAnchor(palVector3& anchor) const;
	void SetAnchor(const palVector3& anchor);

	/*override*/ bool SetParam(int parameterCode, Float value, int axis = -1);
	/*override*/ Float GetParam(int parameterCode, int axis = -1);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;
protected:
	void InitMotor();
	FACTORY_CLASS(palODESphericalLink,palSphericalLink,ODE,1)
private:
	// Have to store these because it's a big mess to compute them.
	palMatrix4x4 m_FrameA, m_FrameB;
};

class palODERigidLink: public palRigidLink, public palODELinkData {
public:
	palODERigidLink();

	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const;
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const;

	/*override*/ bool SetParam(int parameterCode, Float value, int axis = -1);
	/*override*/ Float GetParam(int parameterCode, int axis = -1);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;
protected:
	FACTORY_CLASS(palODERigidLink,palRigidLink,ODE,1)
private:
	// Have to store these because it's a big mess to compute them.
	palMatrix4x4 m_FrameA, m_FrameB;
};

class odeRevoluteLinkFeedback : public palLinkFeedback {
public:
	odeRevoluteLinkFeedback(dJointID odeJoint);
	virtual ~odeRevoluteLinkFeedback();
	virtual bool IsEnabled() const;
	virtual bool SetEnabled(bool enable);
	virtual Float GetValue() const;
protected:
	dJointID m_odeJoint;
	dJointFeedback* m_odeFeedback;
};

class palODERevoluteLink: public palRevoluteLink, public palODELinkData {
public:
	palODERevoluteLink();
	virtual ~palODERevoluteLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const;
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const;

	virtual void AddTorque(Float torque);
	//extra methods provided by ODE abilities:
	virtual void SetAnchorAxis(const palVector3& anchor, const palVector3& axis);
	virtual void GetAnchorAxis(palVector3& anchor, palVector3& axis) const;
	virtual palLinkFeedback* GetFeedback() const throw(palIllegalStateException);
	/*override*/ bool SetParam(int parameterCode, Float value, int axis = -1);
	/*override*/ Float GetParam(int parameterCode, int axis = -1);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;
protected:
	odeRevoluteLinkFeedback* m_feedback;
	FACTORY_CLASS(palODERevoluteLink,palRevoluteLink,ODE,1)
};

class palODEPrismaticLink: public palPrismaticLink, public palODELinkData {
public:
	palODEPrismaticLink();

	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies);

	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const;
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const;

	//extra methods provided by ODE abilities:
	void SetAxis(const palVector3& axis);
	/*override*/ bool SetParam(int parameterCode, Float value, int axis = -1);
	/*override*/ Float GetParam(int parameterCode, int axis = -1);
	/*override*/ bool SupportsParameters() const;
	/*override*/ bool SupportsParametersPerAxis() const;
protected:
	FACTORY_CLASS(palODEPrismaticLink,palPrismaticLink,ODE,1)
private:
	// Have to store these because it's a big mess to compute them.
	palMatrix4x4 m_FrameA, m_FrameB;
};

class palODETerrain : virtual public palTerrain {
public:
	palODETerrain();
	virtual ~palODETerrain();
	virtual const palMatrix4x4& GetLocationMatrix() const;
	//protected:
	dGeomID odeGeom; // the ODE geometries representing this body
};

class palODETerrainPlane : virtual public palTerrainPlane, virtual public palODETerrain {
public:
	palODETerrainPlane();
	virtual void Init(Float x, Float y, Float z, Float min_size);
	virtual const palMatrix4x4& GetLocationMatrix() const;
protected:
	FACTORY_CLASS(palODETerrainPlane,palTerrainPlane,ODE,1)
};

class palODEOrientatedTerrainPlane : virtual  public palOrientatedTerrainPlane, virtual public palODETerrain  {
public:
	palODEOrientatedTerrainPlane();
	virtual void Init(Float x, Float y, Float z, Float nx, Float ny, Float nz, Float min_size);
	virtual const palMatrix4x4& GetLocationMatrix() const {return palOrientatedTerrainPlane::GetLocationMatrix();}
protected:
	FACTORY_CLASS(palODEOrientatedTerrainPlane,palOrientatedTerrainPlane,ODE,1)
};

class palODETerrainMesh : virtual public palTerrainMesh, virtual public palODETerrain {
public:
	palODETerrainMesh();
	virtual void Init(Float x, Float y, Float z, const Float *pVertices, int nVertices, const int *pIndices, int nIndices);
	//	palMatrix4x4& GetLocationMatrix() const;
protected:
	FACTORY_CLASS(palODETerrainMesh,palTerrainMesh,ODE,1)
};

class palODETerrainHeightmap : virtual public palTerrainHeightmap, virtual private palODETerrainMesh {
public:
	palODETerrainHeightmap();
	virtual void Init(Float x, Float y, Float z, Float width, Float depth, int terrain_data_width, int terrain_data_depth, const Float *pHeightmap);
	//	palMatrix4x4& GetLocationMatrix() const;
protected:
	FACTORY_CLASS(palODETerrainHeightmap,palTerrainHeightmap,ODE,1)
};

class palODEPSDSensor : public palPSDSensor {
public:
	palODEPSDSensor();
	virtual void Init(palBody *body, Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range); //position, direction
	Float GetDistance() const;
protected:
	Float m_fRelativePosX;
	Float m_fRelativePosY;
	Float m_fRelativePosZ;
	dGeomID odeRayId;
	FACTORY_CLASS(palODEPSDSensor,palPSDSensor,ODE,1)
};

class palODEMotor : public palMotor {
public:
	palODEMotor();
	virtual ~palODEMotor();
	virtual void Init(palLink *pLink, int axis = -1);
	virtual void Update(Float targetVelocity, Float Max);
	virtual void DisableMotor();
	virtual palLink *GetLink() const;
	virtual void Apply(float dt);
protected:
	palLink* m_Link;
	dJointID odeJoint; //the ODE joint
	FACTORY_CLASS(palODEMotor,palMotor,ODE,1)
};
#endif
