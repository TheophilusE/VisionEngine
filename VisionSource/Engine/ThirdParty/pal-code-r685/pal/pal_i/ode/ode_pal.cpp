#ifdef MICROSOFT_VC
#pragma warning( disable : 4786 ) // ident trunc to '255' chars in debug info
#endif
//(c) Adrian Boeing 2004, see liscence.txt (BSD liscence)
#include "ode_pal.h"
/*
 Abstract:
 PAL - Physics Abstraction Layer. ODE implementation.
 This enables the use of ODE via PAL.

 Implementaiton
 Author:
 Adrian Boeing
 Revision History:
 Version 0.5 : 04/06/04 -
 TODO:
 -get to 1.0 (ie: same as pal.h)
 */
#include <pal/pal.inl>

#ifndef NDEBUG
#ifdef MICROSOFT_VC
#ifdef MEMDEBUG
#include <crtdbg.h>
#define new new(_NORMAL_BLOCK,__FILE__, __LINE__)
#endif
#endif
#endif

#include <cassert>

FACTORY_CLASS_IMPLEMENTATION_BEGIN_GROUP
;	//FACTORY_CLASS_IMPLEMENTATION(palODEMaterial);
FACTORY_CLASS_IMPLEMENTATION(palODEPhysics);

FACTORY_CLASS_IMPLEMENTATION(palODEBoxGeometry);
FACTORY_CLASS_IMPLEMENTATION(palODESphereGeometry);
FACTORY_CLASS_IMPLEMENTATION(palODECapsuleGeometry);
FACTORY_CLASS_IMPLEMENTATION(palODECylinderGeometry);
FACTORY_CLASS_IMPLEMENTATION(palODEConvexGeometry);
FACTORY_CLASS_IMPLEMENTATION(palODEConcaveGeometry);

FACTORY_CLASS_IMPLEMENTATION(palODEGenericBody);

FACTORY_CLASS_IMPLEMENTATION(palODERigidLink);
FACTORY_CLASS_IMPLEMENTATION(palODESphericalLink);
FACTORY_CLASS_IMPLEMENTATION(palODERevoluteLink);
FACTORY_CLASS_IMPLEMENTATION(palODEPrismaticLink);

FACTORY_CLASS_IMPLEMENTATION(palODEOrientatedTerrainPlane);
FACTORY_CLASS_IMPLEMENTATION(palODETerrainPlane);
FACTORY_CLASS_IMPLEMENTATION(palODETerrainMesh);
FACTORY_CLASS_IMPLEMENTATION(palODETerrainHeightmap);

FACTORY_CLASS_IMPLEMENTATION(palODEMotor);

FACTORY_CLASS_IMPLEMENTATION_END_GROUP;
//PAL_MAP<dGeomID, ODE_MATINDEXLOOKUP> palODEMaterials::g_IndexMap;
//std_matrix<palMaterial *> palODEMaterials::g_Materials;
//PAL_VECTOR<PAL_STRING> palODEMaterials::g_MaterialNames;

static dWorldID g_world;
static dSpaceID g_space;
static dJointGroupID g_contactgroup;

/*
 palODEMaterial::palODEMaterial() {
 };

 void palODEMaterial::Init(Float static_friction, Float kinetic_friction, Float restitution) {
 palMaterial::Init(static_friction,kinetic_friction,restitution);
 }
 */

static dGeomID CreateTriMesh(const Float *pVertices, int nVertices, const int *pIndices, int nIndices) {
	dGeomID odeGeom;
	int i;
	dVector3 *spacedvert = new dVector3[nVertices];
#if 0
	dTriIndex *dIndices = new dTriIndex[nIndices];
#else
	int *dIndices = new int[nIndices];
#endif

	for (i = 0; i < nVertices; i++) {
		spacedvert[i][0] = pVertices[i * 3 + 0];
		spacedvert[i][1] = pVertices[i * 3 + 1];
		spacedvert[i][2] = pVertices[i * 3 + 2];
	}

	for (i = 0; i < nIndices; i++) {
		dIndices[i] = pIndices[i];
	}

	// build the trimesh data
	dTriMeshDataID data = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSimple(data, (dReal*)spacedvert, nVertices, (const dTriIndex*)dIndices,
			nIndices);
	// build the trimesh geom
	odeGeom = dCreateTriMesh(g_space, data, 0, 0, 0);
	return odeGeom;
}

palODEPhysics::palODEPhysics() : m_initialized(false) {
}

const char* palODEPhysics::GetVersion() const {
	static char verbuf[256];
	sprintf(verbuf, "ODE V.UNKOWN");
	return verbuf;
}

const char* palODEPhysics::GetPALVersion() const {
	static char verbuf[512];
	sprintf(verbuf, "PAL SDK V%d.%d.%d\nPAL ODE V:%d.%d.%d\nFile: %s\nCompiled: %s %s\nModified:%s",
			PAL_SDK_VERSION_MAJOR, PAL_SDK_VERSION_MINOR, PAL_SDK_VERSION_BUGFIX,
			ODE_PAL_SDK_VERSION_MAJOR, ODE_PAL_SDK_VERSION_MINOR, ODE_PAL_SDK_VERSION_BUGFIX,
			__FILE__, __TIME__, __DATE__, __TIMESTAMP__);
	return verbuf;
}

void palODEPhysics::GetPropertyDocumentation(PAL_MAP<PAL_STRING, PAL_STRING>& descriptions) const
{
	palPhysics::GetPropertyDocumentation(descriptions);
	descriptions["ODE_NoInitOrShutdown"] = "Defaults to FALSE.  If set to true, the global ode init won't be called, nor the global shutdown.  This is so you can manage this yourself.";
	descriptions["WorldERP"] = "The Global value of the Error Reduction Parameter. Default is 0.2. See http://www.ode.org/ode-latest-userguide.html#sec_3_8_2";
	descriptions["WorldCFM"] = "The Global value of the Constraint Force Mixing Parameter. Default is 10^-5 (single) or 10^-10 (double).  See http://www.ode.org/ode-latest-userguide.html#sec_3_8_2";
}

void palODEPhysics::Init(const palPhysicsDesc& desc) {
	palPhysics::Init(desc);
	if (GetInitProperty("ODE_NoInitOrShutdown") != "true") {
		dInitODE2(0);
	}

	g_world = dWorldCreate();
	g_space = dHashSpaceCreate(0);
	g_contactgroup = dJointGroupCreate(0); //0 apparently
	SetGravity(m_fGravityX, m_fGravityY, m_fGravityZ);
	// enable auto disable because pal has support for it on bodies, and it generally helps performance.
	dWorldSetAutoDisableFlag(g_world, 1);

	dReal erp = GetInitProperty("WorldERP", dWorldGetERP(g_world), dReal(PAL_FLOAT_EPSILON), dReal(1.0));
	dWorldSetERP (g_world, erp);

	dReal cfm = GetInitProperty("WorldCFM", dWorldGetCFM(g_world), dReal(0.0), dReal(1.0));
	dWorldSetCFM (g_world, cfm);

	m_initialized = true;
}
;

//colision detection functionality
void palODEPhysics::SetCollisionAccuracy(Float fAccuracy) {

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef PAL_MULTIMAP <palBodyBase*, palBodyBase*> ListenMap;
typedef ListenMap::iterator ListenIterator;
typedef ListenMap::const_iterator ListenConstIterator;
ListenMap pallisten;
#define MAX_CONTACTS 8 // maximum number of contact points per body
dContact g_contactArray[MAX_CONTACTS];


static bool listenCollision(palBodyBase* body1, palBodyBase* body2) {
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

bool IsCollisionResponseEnabled(dBodyID dbody) {
	palBodyBase *body = NULL;
	body = static_cast<palBodyBase *> (dBodyGetData(dbody));
	// TODO get rid of the dynamic cast by storing the palODEBody in the user data, and putting the collision response
	// var there.
	palODEGenericBody* genericBody = dynamic_cast<palODEGenericBody*>(body);
	if (genericBody != NULL && !genericBody->ODEGetCollisionResponseEnabled())
	{
		return false;
	}
	return true;
}

/* this is called by dSpaceCollide when two objects in space are
 * potentially colliding.
 */
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {

	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		// Colliding a space with either a geom or another space.
		dSpaceCollide2(o1, o2, data, &nearCallback);

		if (dGeomIsSpace(o1)) {
			// Colliding all geoms internal to the space.
			dSpaceCollide((dSpaceID)o1, data, &nearCallback);
		}

		if (dGeomIsSpace(o2)) {
			// Colliding all geoms internal to the space.
			dSpaceCollide((dSpaceID)o2, data, &nearCallback);
		}
		return;
	}

	int i = 0;
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	palBodyBase* pb1 = NULL,* pb2 = NULL;
	if (b1 != 0)
		pb1 = static_cast<palBodyBase *> (dBodyGetData(b1));
	if (b2 != 0)
		pb2 = static_cast<palBodyBase *> (dBodyGetData(b2));

	if (b1 != 0 && b2 != 0 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
		return;

	bool response = true;
	if (b1)
		response = response && IsCollisionResponseEnabled(b1);
	if (b2)
		response = response && IsCollisionResponseEnabled(b2);




	palMaterialDesc finalMaterial;
	palMaterial * pm1 = pb1->GetMaterial();
	palMaterial * pm2 = pb2->GetMaterial();

	palPhysics* curPhysics = static_cast<palPhysics*>(pb1->GetParent());
	palCollisionDetection* curCollision = curPhysics->asCollisionDetection();
	if (curCollision == nullptr) {
		static bool printed = false;
		if (!printed)
		{
			perror("Internal Error: The Collision detection interface in the ODE plugin returned NULL.  Collision detection won't work.");
			printed = true;
		}
		return;
	}

	palMaterials* materials = curPhysics->GetMaterials();

	int numc = dCollide(o1, o2, MAX_CONTACTS, &g_contactArray[0].geom, sizeof(dContact));

	if (numc > 0) {
		for (i = 0; i < numc; i++) {
			palContactPoint cp;

			for (unsigned vidx = 0; vidx < 3; ++vidx)
			{
				cp.m_vContactPosition[vidx] = Float(g_contactArray[i].geom.pos[vidx]);
				cp.m_vContactNormal[vidx] = Float(g_contactArray[i].geom.normal[vidx]);
			}

			cp.m_fDistance = Float(g_contactArray[i].geom.depth);

			cp.m_pBody1 = pb1;
			cp.m_pBody2 = pb2;

			if (!materials->HandleCustomInteraction(pm1, pm2, finalMaterial, cp, true))
			{
				finalMaterial.m_fStatic = Float(dInfinity);
				finalMaterial.m_fRestitution = Float(0.1);
				finalMaterial.m_bEnableAnisotropicFriction = false;
			}
			else
			{
				for (unsigned vidx = 0; vidx < 3; ++vidx)
				{
					g_contactArray[i].geom.pos[vidx] = dReal(cp.m_vContactPosition[vidx]);
					g_contactArray[i].geom.normal[vidx] = dReal(cp.m_vContactNormal[vidx]);
				}
				g_contactArray[i].geom.depth = dReal(cp.m_fDistance);
			}

			g_contactArray[i].surface.mode = dContactBounce //| dContactSoftERP | dContactSoftCFM
					| dContactApprox1;
			//remove dContactSoftCFM | dContactApprox1 for bounce..
			g_contactArray[i].surface.mu = finalMaterial.m_fStatic;
			g_contactArray[i].surface.bounce = finalMaterial.m_fRestitution;
			if (finalMaterial.m_bEnableAnisotropicFriction)
			{
				g_contactArray[i].surface.mu = finalMaterial.m_fStatic * finalMaterial.m_vStaticAnisotropic[0];
				g_contactArray[i].surface.mode |= dContactMu2;
				g_contactArray[i].surface.mu2 = finalMaterial.m_fStatic * finalMaterial.m_vStaticAnisotropic[1];
			}
			//			g_contactArray[i].surface.slip1 = 0.1; // friction
			//			g_contactArray[i].surface.slip2 = 0.1;
			//			g_contactArray[i].surface.bounce_vel = 1;
			//			g_contactArray[i].surface.soft_erp = 0.5f;
			//			g_contactArray[i].surface.soft_cfm = 0.01f;
			if (response)
			{
				dJointID c = dJointCreateContact(g_world, g_contactgroup, &g_contactArray[i]);
				dJointAttach(c, b1, b2);
			}

			bool dolisten = false;
			if (pb1 != NULL)
			{
				dolisten = listenCollision(pb1, pb2);
			}
			else if (pb2 != NULL)
			{
				dolisten = listenCollision(pb2, pb1);
			}

			if (!dolisten) continue;

			curPhysics->asCollisionDetection()->EmitContact(cp);
		}
	}

}
static void OdeRayCallback(void* data, dGeomID o1, dGeomID o2) {
	//o2 == ray
	// handle sub-space
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, data, &OdeRayCallback);
		return;
	} else {
		if (o1 == o2) {
			return;
		}
		dContactGeom contactArray[MAX_CONTACTS];
		int numColls = dCollide(o1, o2, MAX_CONTACTS, contactArray, sizeof(dContactGeom));
		if (numColls == 0) {
			return;
		}

		//now find the closest
		int closest = 0;
		for (int i = 0; i < numColls; i++) {
			if (contactArray[i].depth < contactArray[closest].depth) {
				closest = i;
			}
		}

		dContactGeom &c = contactArray[closest];
		palRayHit *phit = static_cast<palRayHit *> (data);
		phit->Clear();
		phit->SetHitPosition(c.pos[0], c.pos[1], c.pos[2]);
		phit->SetHitNormal(c.normal[0], c.normal[1], c.normal[2]);
		phit->m_bHit = true;

		phit->m_fDistance = c.depth;
		phit->m_pBody = reinterpret_cast<palBodyBase*> (dGeomGetData(c.g1));
	}

}

struct OdeCallbackData {
	float m_range;
	palRayHitCallback* m_callback;
	palGroupFlags m_filter;
};

static void OdeRayCallbackCallback(void* data, dGeomID o1, dGeomID o2) {
	//o2 == ray
	// handle sub-space
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, data, &OdeRayCallback);
		return;
	} else {
		if (o1 == o2) {
			return;
		}
		dContactGeom contactArray[MAX_CONTACTS];
		int numColls = dCollide(o1, o2, MAX_CONTACTS, contactArray, sizeof(dContactGeom));
		if (numColls == 0) {
			return;
		}

		OdeCallbackData* callbackData = static_cast<OdeCallbackData*> (data);

		palRayHitCallback& callback = *callbackData->m_callback;

		//now find the closest
		float distance = callbackData->m_range;
		for (int i = 0; i < numColls; i++) {
			dContactGeom &c = contactArray[i];

			unsigned long categoryBits = dGeomGetCategoryBits(c.g1);

			if ((categoryBits & callbackData->m_filter) == 0) {
				continue;
			}

			float newDistance = c.depth;
			if (newDistance >= distance) {
				continue;
			}

			palRayHit hit;
			hit.Clear();
			hit.SetHitPosition(c.pos[0], c.pos[1], c.pos[2]);
			hit.SetHitNormal(c.normal[0], c.normal[1], c.normal[2]);
			hit.m_bHit = true;

			hit.m_fDistance = c.depth;
			hit.m_pBody = reinterpret_cast<palBodyBase*> (dGeomGetData(c.g1));

			distance = callback.AddHit(hit);
		}
	}

}

void palODEPhysics::RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range,
		palRayHit& hit) const {
	dGeomID odeRayId = dCreateRay(0, range);
	dGeomRaySet(odeRayId, x, y, z, dx, dy, dz);
	dSpaceCollide2((dGeomID)ODEGetSpace(), odeRayId, &hit, &OdeRayCallback);

}

void palODEPhysics::RayCast(Float x, Float y, Float z, Float dx, Float dy, Float dz, Float range,
		palRayHitCallback& callback, palGroupFlags groupFilter) const {
	dGeomID odeRayId = dCreateRay(0, range);
	dGeomRaySet(odeRayId, x, y, z, dx, dy, dz);
	OdeCallbackData data;
	data.m_range = range;
	data.m_callback = &callback;
	data.m_filter = groupFilter;
	dSpaceCollide2((dGeomID)ODEGetSpace(), odeRayId, &data, &OdeRayCallbackCallback);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void palODEPhysics::NotifyCollision(palBodyBase *body1, palBodyBase *body2, bool enabled) {
	bool found = false;
	std::pair<ListenIterator, ListenIterator> range;

	// The greater one is the key, which also works for NULL.
	palBodyBase* b0 = body1 > body2 ? body1: body2;
	palBodyBase* b1 = body1 < body2 ? body1: body2;

	if (b0 != NULL)
	{
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

		if (!found && enabled)
		{
			pallisten.insert(range.second, std::make_pair(b0, b1));
		}
	}
}

void palODEPhysics::NotifyCollision(palBodyBase *pBody, bool enabled) {
	NotifyCollision(pBody, NULL, enabled);
}

void palODEPhysics::CleanupNotifications(palBodyBase *pBody) {
	std::pair<ListenIterator, ListenIterator> range;

	if (pBody != NULL)
	{
		range = pallisten.equal_range(pBody);
		// erase the forward list for the one passed in.
		pallisten.erase(range.first, range.second);

		// since only GREATER keys will have this one as a value, just search starting at range.second.
		// plus range.second is not invalidated by the erase.
		ListenIterator i = range.second;
		while (i != pallisten.end())
		{
			if (i->second == pBody)
			{
				ListenIterator oldI = i;
				++i;
				pallisten.erase(oldI);
			}
			else
			{
				++i;
			}
		}
	}
}

dWorldID palODEPhysics::ODEGetWorld() const {
	return g_world;
}

dSpaceID palODEPhysics::ODEGetSpace() const {
	return g_space;
}

void palODEPhysics::SetGravity(Float gravity_x, Float gravity_y, Float gravity_z) {
	dWorldSetGravity(g_world, gravity_x, gravity_y, gravity_z);
}
/*
 void palODEPhysics::SetGroundPlane(bool enabled, Float size) {
 if (enabled)
 //		dCreatePlane (g_space,0,0,1,0);
 dCreatePlane (g_space,0,1,0,0);
 };
 */

void palODEPhysics::Iterate(Float timestep) {
	ClearContacts();
	dSpaceCollide(g_space, 0, &nearCallback);//evvvil
	dWorldStep(g_world, timestep);

	dJointGroupEmpty(g_contactgroup);
}

void palODEPhysics::Cleanup() {
	if (m_initialized) {
		dJointGroupDestroy(g_contactgroup);
		dSpaceDestroy(g_space);
		dWorldDestroy(g_world);
		if (GetInitProperty("ODE_NoInitOrShutdown") != "true") {
			dCloseODE();
		}
	}
}

void palODEPhysics::SetGroupCollisionOnGeom(unsigned long bits, unsigned long otherBits,
		dGeomID geom, bool collide) {
	unsigned long coll = dGeomGetCollideBits(geom);

	if (dGeomGetCategoryBits(geom) & bits) {
		if (collide)
			dGeomSetCollideBits(geom, coll | otherBits);
		else
			dGeomSetCollideBits(geom, coll & ~otherBits);
	} else if (dGeomGetCategoryBits(geom) & otherBits) {
		if (collide)
			dGeomSetCollideBits(geom, coll | bits);
		else
			dGeomSetCollideBits(geom, coll & ~bits);
	}
}

void palODEPhysics::SetGroupCollision(palGroup a, palGroup b, bool collide) {
	unsigned long bits = 1L << ((unsigned long)a);
	unsigned long otherBits = 1L << ((unsigned long)b);

	if (m_CollisionMasks.size() < size_t(std::max(a, b))) {
		m_CollisionMasks.resize(std::max(a, b)+1, ~0);
	}

	//Save off the collision mask so that new bodies can pick it up.
	if (collide) {
		m_CollisionMasks[a] = m_CollisionMasks[a] | otherBits;
		m_CollisionMasks[b] = m_CollisionMasks[b] | bits;
	} else {
		m_CollisionMasks[a] = m_CollisionMasks[a] & ~otherBits;
		m_CollisionMasks[b] = m_CollisionMasks[b] & ~bits;
	}

	int t = dSpaceGetNumGeoms(g_space);

	for (int i = 0; i < t; ++i) {
		dGeomID geom = dSpaceGetGeom(g_space, i);

		SetGroupCollisionOnGeom(bits, otherBits, geom, collide);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODEBody::palODEBody()
: odeBody(0)
, m_bCollisionResponseEnabled(true)
{
}

palODEBody::~palODEBody() {
	//printf("now deleteing %d, with %d,%d\n",this,odeBody,odeGeom);
	Cleanup();
	if (odeBody) {
		dBodyDestroy(odeBody);
		odeBody = 0;
	}
	palODEPhysics* odePhysics = dynamic_cast<palODEPhysics*>(palFactory::GetInstance()->GetActivePhysics());
	odePhysics->CleanupNotifications(this);
}

void palODEBody::BodyInit(Float x, Float y, Float z) {
	SetPosition(x, y, z);
	//The group is stored before init, so it has be set when init happens.
	SetGroup(GetGroup());
}

void palODEBody::CreateODEBody() {
	odeBody = dBodyCreate(g_world);
	dBodySetData(odeBody, dynamic_cast<palBodyBase *> (this));
}

void palODEBody::SetPosition(Float x, Float y, Float z) {
	if (odeBody) {
		dBodySetPosition(odeBody, x, y, z);
	} else {
		palBody::SetPosition(x,y,z);
	}
}

void palODEBody::SetGeometryBody(palGeometry *pgeom) {
	palODEGeometry* pODEGeom = dynamic_cast<palODEGeometry*> (pgeom);
	// !! Warning !!
	//		[Sukender] The SF.net user "christopheralme" reported an error, and I was able to reproduce the issue.
	//		I thus changed the following test (pODEGeom != NULL), but it seems to be a "workaround" rather than a fix.
	//		If you know how to really fix the pODEGeom->odeGeom initialization, please do it and uncomment the assertion.
	//		Else, if you are *SURE* pODEGeom->odeGeom can be NULL, then just remove my comment. :)
	//if (pODEGeom != NULL) {
	//assert(pODEGeom->odeGeom);
	if (pODEGeom != NULL && pODEGeom->odeGeom != NULL) {
		dGeomSetData(pODEGeom->odeGeom, static_cast<palBodyBase*>(this));
		dGeomSetBody(pODEGeom->odeGeom, odeBody);
	}
	palBodyBase::SetGeometryBody(pgeom);
}

static void convODEFromPAL(dReal pos[3], dReal R[12], const palMatrix4x4& location) {
	R[3] = 0.0;
	R[7] = 0.0;
	R[11] = 0.0;

	R[0] = location._mat[0];
	R[4] = location._mat[1];
	R[8] = location._mat[2];

	R[1] = location._mat[4];
	R[5] = location._mat[5];
	R[9] = location._mat[6];

	R[2] = location._mat[8];
	R[6] = location._mat[9];
	R[10] = location._mat[10];

	pos[0] = location._mat[12];
	pos[1] = location._mat[13];
	pos[2] = location._mat[14];
}

static void convODEToPAL(const dReal *pos, const dReal *R, palMatrix4x4& m_mLoc) {
	mat_identity(&m_mLoc);
	//this code is correct!
	//it just looks wrong, because R is a padded SSE structure!
	m_mLoc._mat[0] = R[0];
	m_mLoc._mat[1] = R[4];
	m_mLoc._mat[2] = R[8];
	m_mLoc._mat[3] = 0;
	m_mLoc._mat[4] = R[1];
	m_mLoc._mat[5] = R[5];
	m_mLoc._mat[6] = R[9];
	m_mLoc._mat[7] = 0;
	m_mLoc._mat[8] = R[2];
	m_mLoc._mat[9] = R[6];
	m_mLoc._mat[10] = R[10];
	m_mLoc._mat[11] = 0;
	m_mLoc._mat[12] = pos[0];
	m_mLoc._mat[13] = pos[1];
	m_mLoc._mat[14] = pos[2];
	m_mLoc._mat[15] = 1;
}

void palODEBody::RecalcMassAndInertia() {
	dMass m;
	dMassSetZero(&m);

	unsigned validGeoms = 0;

	for (unsigned int i = 0; i < m_Geometries.size(); i++) {
		palODEGeometry *pog = dynamic_cast<palODEGeometry *> (m_Geometries[i]);

		dReal pos[3];
		dReal R[12];

		convODEFromPAL(pos, R, pog->GetOffsetMatrix());
		if (pog->odeGeom != 0 && odeBody != 0 && pog->GetMass() > 0.0) {
			dMass massGeom;
			dMassSetZero(&m);
			pog->CalculateMassParams(massGeom, pog->GetMass());
			if (dMassCheck(&massGeom)) {
				dMassTranslate(&massGeom, pos[0], pos[1], pos[2]);
				dMassRotate(&massGeom, R);
				dMassAdd(&m, &massGeom);
				validGeoms++;
			}
		}
	}

	if (validGeoms == 0) {
		dMassSetSphereTotal(&m, m_fMass, 1.0);
	}

	dReal newMass = m_fMass;
	if (newMass == dReal(0.0))
	{
		newMass = dReal(1.0);
	}

	dMassAdjust(&m, newMass);
	m.c[0] = 0.0;
	m.c[1] = 0.0;
	m.c[2] = 0.0;

	if (dMassCheck(&m)) {
		dBodySetMass(odeBody, &m);
	}
	else {
		dMassSetZero(&m);
		dMassSetSphereTotal(&m, newMass, 1.0);
		dBodySetMass(odeBody, &m);
	}
}

void palODEBody::SetPosition(const palMatrix4x4& location) {
	if (odeBody) {
		dReal pos[3];
		dReal R[12];
		convODEFromPAL(pos, R, location);

		dBodySetPosition(odeBody, pos[0], pos[1], pos[2]);
		dBodySetRotation(odeBody, R);
	}
	palBody::SetPosition(location);
}

const palMatrix4x4& palODEBody::GetLocationMatrix() const {
	if (odeBody) {
		const dReal *pos = dBodyGetPosition(odeBody);
		const dReal *R = dBodyGetRotation(odeBody);
		//memset(m_mLoc._mat,0,sizeof(palMatrix4x4));
		convODEToPAL(pos, R, m_mLoc);
	}
	return m_mLoc;
}

bool palODEBody::IsActive() const {
	return dBodyIsEnabled(odeBody) != 0;
}

void palODEBody::SetActive(bool active) {
	if (active)
		dBodyEnable(odeBody);
	else
		dBodyDisable(odeBody);
}

void palODEBody::SetGroup(palGroup group) {
	palBodyBase::SetGroup(group);

	palODEPhysics* physics =
			dynamic_cast<palODEPhysics*> (palFactory::GetInstance()->GetActivePhysics());

	unsigned long bits = 1L << (unsigned long)(group);
	for (unsigned int i = 0; i < m_Geometries.size(); i++) {
		palODEGeometry *pg = dynamic_cast<palODEGeometry *> (m_Geometries[i]);
		dGeomSetCategoryBits(pg->odeGeom, bits);
		if (physics->m_CollisionMasks.size() > (unsigned long)(group)) {
			dGeomSetCollideBits(pg->odeGeom, physics->m_CollisionMasks[group]);
		} else {
			// all bits on by default.
			dGeomSetCollideBits(pg->odeGeom, ~0);
		}
	}
}

#if 0
void palODEBody::SetForce(Float fx, Float fy, Float fz) {
	dBodySetForce (odeBody,fx,fy,fz);
}
void palODEBody::GetForce(palVector3& force) const {
	const dReal *pf=dBodyGetForce(odeBody);
	force.x=pf[0];
	force.y=pf[1];
	force.z=pf[2];
}

void palODEBody::SetTorque(Float tx, Float ty, Float tz) {
	dBodySetTorque(odeBody,tx,ty,tz);
}

void palODEBody::GetTorque(palVector3& torque) const {
	const dReal *pt=dBodyGetTorque(odeBody);
	torque.x=pt[0];
	torque.y=pt[1];
	torque.z=pt[2];
}
#endif

void palODEBody::ApplyForce(Float fx, Float fy, Float fz) {
	dBodyAddForce(odeBody, fx, fy, fz);
}

void palODEBody::ApplyTorque(Float tx, Float ty, Float tz) {
	dBodyAddTorque(odeBody, tx, ty, tz);
}
/*
 void palODEBody::ApplyImpulse(Float fx, Float fy, Float fz) {
 dReal *pv = (dReal *)dBodyGetLinearVel(odeBody);
 dBodySetLinearVel(odeBody,pv[0]+fx/m_fMass,pv[1]+fy/m_fMass,pv[2]+fz/m_fMass);
 //	m_kVelocity        += rkImpulse * m_fInvMass;
 }

 void palODEBody::ApplyAngularImpulse(Float fx, Float fy, Float fz) {
 dReal *pv = (dReal *)dBodyGetAngularVel(odeBody);
 dBodySetAngularVel(odeBody,pv[0]+fx/m_fMass,pv[1]+fy/m_fMass,pv[2]+fz/m_fMass);
 }
 */
void palODEBody::GetLinearVelocity(palVector3& velocity) const {
	const dReal *pv = dBodyGetLinearVel(odeBody);
	velocity.x = pv[0];
	velocity.y = pv[1];
	velocity.z = pv[2];
}

void palODEBody::GetAngularVelocity(palVector3& velocity) const {
	const dReal *pv = dBodyGetAngularVel(odeBody);
	velocity.x = pv[0];
	velocity.y = pv[1];
	velocity.z = pv[2];
}

void palODEBody::SetLinearVelocity(const palVector3& vel) {
	dBodySetLinearVel(odeBody, vel.x, vel.y, vel.z);
}
void palODEBody::SetAngularVelocity(const palVector3& vel) {
	dBodySetAngularVel(odeBody, vel.x, vel.y, vel.z);
}

const std::bitset<palODEBody::DUMMY_ACTIVATION_SETTING_TYPE>
palODEBody::SUPPORTED_SETTINGS = std::bitset<palODEBody::DUMMY_ACTIVATION_SETTING_TYPE>(int(~(0xFFFFFFFF << palODEBody::DUMMY_ACTIVATION_SETTING_TYPE)));


Float palODEBody::GetActivationLinearVelocityThreshold() const {
	Float velocity;
	if (odeBody != 0) {
		velocity = Float(dBodyGetAutoDisableLinearThreshold(odeBody));
	}
	else {
		velocity = Float(-1.0);
	}
	return velocity;

}

void palODEBody::SetActivationLinearVelocityThreshold(Float velocity) {
	if (odeBody != 0) {
		dBodySetAutoDisableLinearThreshold(odeBody, dReal(velocity));
	}
}

Float palODEBody::GetActivationAngularVelocityThreshold() const {
	Float omega;
	if (odeBody != 0) {
		omega = Float(dBodyGetAutoDisableAngularThreshold(odeBody));
	}
	else {
		omega = Float(-1.0);
	}
	return omega;
}

void palODEBody::SetActivationAngularVelocityThreshold(Float omega) {
	if (odeBody != 0) {
		dBodySetAutoDisableAngularThreshold(odeBody, dReal(omega));
	}
}

Float palODEBody::GetActivationTimeThreshold() const {
	Float time;
	if (odeBody != 0) {
		time = Float(dBodyGetAutoDisableTime(odeBody));
	}
	else {
		time = Float(-1.0);
	}
	return time;
}

void palODEBody::SetActivationTimeThreshold(Float time) {
	if (odeBody != 0) {
		dBodySetAutoDisableTime(odeBody, dReal(time));
	}
}

const std::bitset<palODEBody::DUMMY_ACTIVATION_SETTING_TYPE>& palODEBody::GetSupportedActivationSettings() const {
	return SUPPORTED_SETTINGS;
}

palODEGeometry::palODEGeometry() {
	m_pBody = 0;
	odeGeom = 0;
}

palODEGeometry::~palODEGeometry() {
	if (odeGeom) {
		dGeomDestroy(odeGeom);
		odeGeom = 0;
	}
}

const palMatrix4x4& palODEGeometry::GetLocationMatrix() const {
	if (odeGeom) {
		const dReal *pos = dGeomGetPosition(odeGeom);
		const dReal *R = dGeomGetRotation(odeGeom);
		convODEToPAL(pos, R, m_mLoc);
	}
	return m_mLoc;
}
void palODEGeometry::SetPosition(const palMatrix4x4 &loc) {
	palGeometry::SetPosition(loc);

	dReal pos[3];
	dReal R[12];

	convODEFromPAL(pos, R, loc);

	dGeomSetPosition(odeGeom, pos[0], pos[1], pos[2]);
	dGeomSetRotation(odeGeom, R);
}

void palODEGeometry::ReCalculateOffset() {
	palGeometry::ReCalculateOffset();
	if (m_pBody && odeGeom !=0 && dGeomGetBody(odeGeom) != 0)
	{
		dReal pos[3];
		dReal R[12];

		convODEFromPAL(pos, R, m_mOffset);
		if (odeGeom != 0) {
			dGeomSetOffsetPosition(odeGeom, pos[0], pos[1], pos[2]);
			dGeomSetOffsetRotation(odeGeom, R);
		}
	}
}

palODEBoxGeometry::palODEBoxGeometry() {
}

void palODEBoxGeometry::Init(const palMatrix4x4 &pos, Float width, Float height, Float depth, Float mass) {
	palBoxGeometry::Init(pos, width, height, depth, mass);
	memset(&odeGeom, 0, sizeof(odeGeom));
	palVector3 dim = GetXYZDimensions();
	odeGeom = dCreateBox(g_space, dim.x, dim.y, dim.z);

	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom,pob->odeBody);
				//				printf("made geom with b:%d\n",pob->odeBody);
			}
		}
	}

	SetPosition(pos);
}

void palODEBoxGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetBoxTotal(&odeMass, massScalar, m_fWidth, m_fHeight, m_fDepth);
}

palODESphereGeometry::palODESphereGeometry() {
}

void palODESphereGeometry::Init(const palMatrix4x4 &pos, Float radius, Float mass) {
	palSphereGeometry::Init(pos, radius, mass);
	memset(&odeGeom, 0, sizeof(odeGeom));
	odeGeom = dCreateSphere(g_space, m_fRadius);
	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom, pob->odeBody);
			}
		}
	}
	SetPosition(pos);
}

void palODESphereGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetSphereTotal(&odeMass, massScalar, m_fRadius);
}

palODECapsuleGeometry::palODECapsuleGeometry() : m_upAxis(1) {
}

void palODECapsuleGeometry::Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass) {
	palCapsuleGeometry::Init(pos,radius,length,mass);
	m_upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();
	memset(&odeGeom ,0,sizeof(odeGeom));
	odeGeom = dCreateCapsule(g_space, m_fRadius, m_fLength+m_fRadius);
	//odeGeom = dCreateCylinder(g_space, m_fRadius, m_fLength);

	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom, pob->odeBody);
			}
		}
	}
	SetPosition(pos);
}

void palODECapsuleGeometry::ReCalculateOffset() {
	palODEGeometry::ReCalculateOffset();
	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dMatrix3 R;
				if (m_upAxis == 1) {
					dRFromAxisAndAngle(R,1,0,0,M_PI/2);
				}
				else if (m_upAxis == 0) {
					dRFromAxisAndAngle(R,0,1,0,M_PI/2);
				}
				else {
					dRSetIdentity(R);
				}
				dReal pos[3];
				dReal offsetR[12];

				convODEFromPAL(pos, offsetR, m_mOffset);

				dMultiply0(R, offsetR, R, 3, 3, 3);
				dGeomSetOffsetRotation(odeGeom,R);
			}
		}
	}
}

const palMatrix4x4& palODECapsuleGeometry::GetLocationMatrix() const {
	if (odeGeom) {
		const dReal *pos = dGeomGetPosition(odeGeom);
		const dReal *R = dGeomGetRotation(odeGeom);
		convODEToPAL(pos, R, m_mLoc);
		if (m_upAxis == 1) {
			mat_rotate(&m_mLoc, -90, 1, 0, 0);
		} else if (m_upAxis == 0) {
			mat_rotate(&m_mLoc, -90, 0, 1, 0);
		}
	}
	return m_mLoc;
}

void palODECapsuleGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetCapsuleTotal(&odeMass, massScalar, m_upAxis, m_fRadius, m_fLength);
}

palODECylinderGeometry::palODECylinderGeometry() : m_upAxis(1) {
}

void palODECylinderGeometry::Init(const palMatrix4x4 &pos, Float radius, Float length, Float mass) {
	palCylinderGeometry::Init(pos,radius,length,mass);
	m_upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();
	memset(&odeGeom ,0,sizeof(odeGeom));
	odeGeom = dCreateCylinder(g_space, m_fRadius, m_fLength);

	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom, pob->odeBody);
			}
		}
	}
	SetPosition(pos);
}

void palODECylinderGeometry::ReCalculateOffset() {
	palODEGeometry::ReCalculateOffset();
	if (m_pBody) {
		palODEBody *pob = dynamic_cast<palODEBody *> (m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dMatrix3 R;
				if (m_upAxis == 1) {
					dRFromAxisAndAngle(R,1,0,0,M_PI/2);
				}
				else if (m_upAxis == 0) {
					dRFromAxisAndAngle(R,0,1,0,M_PI/2);
				}
				else {
					dRSetIdentity(R);
				}
				dReal pos[3];
				dReal offsetR[12];

				convODEFromPAL(pos, offsetR, m_mOffset);

				dMultiply0(R, offsetR, R, 3, 3, 3);
				dGeomSetOffsetRotation(odeGeom,R);
			}
		}
	}
}

const palMatrix4x4& palODECylinderGeometry::GetLocationMatrix() const {
	if (odeGeom) {
		const dReal *pos = dGeomGetPosition(odeGeom);
		const dReal *R = dGeomGetRotation(odeGeom);
		convODEToPAL(pos, R, m_mLoc);
		if (m_upAxis == 1) {
			mat_rotate(&m_mLoc, -90, 1, 0, 0);
		} else if (m_upAxis == 0) {
			mat_rotate(&m_mLoc, -90, 0, 1, 0);
		}
	}
	return m_mLoc;
}

void palODECylinderGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetCylinderTotal(&odeMass, massScalar, m_upAxis, m_fRadius, m_fLength);
}

palODEConvexGeometry::palODEConvexGeometry() {
}

#include <pal_i/hull.h>

void palODEConvexGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices,
		Float mass) {

	palConvexGeometry::Init(pos, pVertices, nVertices, mass);
	unsigned int i;

	HullDesc desc;
	desc.SetHullFlag(QF_TRIANGLES);
	desc.mVcount = nVertices;
	desc.mVertices = new double[desc.mVcount * 3];
	for (i = 0; i < desc.mVcount; i++) {
		desc.mVertices[i * 3 + 0] = pVertices[i * 3 + 0];
		desc.mVertices[i * 3 + 1] = pVertices[i * 3 + 1];
		desc.mVertices[i * 3 + 2] = pVertices[i * 3 + 2];
	}

	desc.mVertexStride = sizeof(double) * 3;

	HullResult dresult;
	HullLibrary hl;
	/*HullError ret =*/ hl.CreateConvexHull(desc, dresult);

	odeGeom = CreateTriMesh(pVertices, nVertices, (int*)dresult.mIndices, dresult.mNumFaces * 3);
	SetPosition(pos);

	hl.ReleaseResult(dresult);

	if (m_pBody) {
		palODEBody *pob=dynamic_cast<palODEBody *>(m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom,pob->odeBody);
			}
		}
	}
}

void palODEConvexGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass){
	palConvexGeometry::Init(pos,pVertices,nVertices,pIndices,nIndices,mass);

	odeGeom = CreateTriMesh(pVertices,nVertices,pIndices,nIndices);
	SetPosition(pos);

	if (m_pBody) {
		palODEBody *pob=dynamic_cast<palODEBody *>(m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom,pob->odeBody);
			}
		}
	}
}

void palODEConvexGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetTrimeshTotal(&odeMass, massScalar, odeGeom);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODEConcaveGeometry::palODEConcaveGeometry() {
}

void palODEConcaveGeometry::Init(const palMatrix4x4 &pos, const Float *pVertices, int nVertices, const int *pIndices, int nIndices, Float mass){
	palConcaveGeometry::Init(pos,pVertices,nVertices,pIndices,nIndices,mass);

	odeGeom = CreateTriMesh(pVertices,nVertices,pIndices,nIndices);


	if (m_pBody) {
		palODEBody *pob=dynamic_cast<palODEBody *>(m_pBody);
		if (pob) {
			if (pob->odeBody) {
				dGeomSetBody(odeGeom,pob->odeBody);
			}
		}
	}
}

void palODEConcaveGeometry::CalculateMassParams(dMass& odeMass, Float massScalar) const {
	dMassSetTrimeshTotal(&odeMass, massScalar, odeGeom);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODEGenericBody::palODEGenericBody()
: palODEBody()
, palGenericBody()
{
}

void palODEGenericBody::Init(const palMatrix4x4 &pos) {
	CreateODEBody();
	palGenericBody::Init(pos);
	// Must set the position after the Init because the init only stores the pos
	// and not on the body because most engines have to do some more setup before setting it.
	SetPosition(pos);
	SetGroup(GetGroup());
	SetDynamicsType(GetDynamicsType());
}

void palODEGenericBody::SetDynamicsType(palDynamicsType dynType) {
	palGenericBody::SetDynamicsType(dynType);
	if (odeBody != 0) {

		switch (dynType) {
		case PALBODY_DYNAMIC: {
			dBodySetDynamic(odeBody);
			//Reset the mass now that it's dynamic.
			RecalcMassAndInertia();
			break;
		}
		case PALBODY_STATIC: {
			// I know this is technically wrong, but ode bodies can't be static.  Geometry
			// with no body can be static, but that would be kind of a mess to have a state where the body has been
			// deleted and the geometry is all separate.  It could be done, but we'll wait on that.
			dBodySetKinematic(odeBody);
			break;
		}
		case PALBODY_KINEMATIC: {
			dBodySetKinematic(odeBody);
			break;
		}

		}
	}
}

void palODEGenericBody::SetGravityEnabled(bool enabled) {
	if (odeBody != 0) {
		dBodySetGravityMode(odeBody, int(enabled));
	}
}

bool palODEGenericBody::IsGravityEnabled() const {
	bool result = true;
	if (odeBody != 0) {
		result = (dBodyGetGravityMode(odeBody) > 0);
	}
	return result;
}

void palODEGenericBody::SetCollisionResponseEnabled(bool enabled) {
	m_bCollisionResponseEnabled = enabled;
}

bool palODEGenericBody::IsCollisionResponseEnabled() const {
	return ODEGetCollisionResponseEnabled();
}

void palODEGenericBody::SetMass(Float mass) {
	palGenericBody::SetMass(mass);
	if (odeBody != 0 && GetDynamicsType() == PALBODY_DYNAMIC) {
		RecalcMassAndInertia();
	}
}

void palODEGenericBody::SetInertia(Float Ixx, Float Iyy, Float Izz) {
	palGenericBody::SetInertia(Ixx, Iyy, Izz);
	if (odeBody != 0 && GetDynamicsType() == PALBODY_DYNAMIC) {
		RecalcMassAndInertia();
	}
}

void palODEGenericBody::SetLinearDamping(Float damping) {
	palGenericBody::SetLinearDamping(damping);
	if (odeBody != 0) {
		dBodySetLinearDamping(odeBody, dReal(damping));
	}
}

Float palODEGenericBody::GetLinearDamping() const {
	if (odeBody != 0) {
		return Float(dBodyGetLinearDamping(odeBody));
	}
	return palGenericBody::GetLinearDamping();
}

void palODEGenericBody::SetAngularDamping(Float damping) {
	palGenericBody::SetAngularDamping(damping);
	if (odeBody != 0) {
		dBodySetAngularDamping(odeBody, dReal(damping));
	}
}

Float palODEGenericBody::GetAngularDamping() const
{
	if (odeBody != 0) {
		return Float(dBodyGetAngularDamping(odeBody));
	}
	return palGenericBody::GetAngularDamping();
}

void palODEGenericBody::SetMaxAngularVelocity(Float maxAngVel)
{
	palGenericBody::SetMaxAngularVelocity(maxAngVel);
	// TODO this will have to be done at tick time.
}

Float palODEGenericBody::GetMaxAngularVelocity() const
{
	return palGenericBody::GetMaxAngularVelocity();
}

void palODEGenericBody::ConnectGeometry(palGeometry* pGeom) {
	palGenericBody::ConnectGeometry(pGeom);
	if (odeBody != 0 && GetDynamicsType() == PALBODY_DYNAMIC) {
		RecalcMassAndInertia();
	}
}

void palODEGenericBody::RemoveGeometry(palGeometry* pGeom) {
	palGenericBody::RemoveGeometry(pGeom);
	if (odeBody != 0 && GetDynamicsType() == PALBODY_DYNAMIC) {
		RecalcMassAndInertia();
	}
}

bool palODEGenericBody::IsDynamic() const {
	if (odeBody != 0) {
		return !dBodyIsKinematic(odeBody);
	}
	return palGenericBody::IsDynamic();

}
bool palODEGenericBody::IsKinematic() const {
	if (odeBody != 0) {
		return dBodyIsKinematic(odeBody) && palGenericBody::IsKinematic();
	}
	return palGenericBody::IsKinematic();
}

bool palODEGenericBody::IsStatic() const {
	if (odeBody != 0) {
		return dBodyIsKinematic(odeBody) && palGenericBody::IsStatic();
	}
	return palGenericBody::IsStatic();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODELinkData::palODELinkData() {
	odeJoint = 0;
	odeMotorJoint = 0;
}

palODELinkData::~palODELinkData() {
	if (odeJoint) {
		dJointDestroy(odeJoint);
		odeJoint = 0;
	}
	if (odeMotorJoint) {
		dJointDestroy(odeJoint);
		odeMotorJoint = 0;
	}
}

static bool MapLinkParameterToODEParam(int& parameterCode, int axis)
{
	bool result = true;
	if (axis < 0)
		axis = 0;
	else
		// Need to add one to the axis because ODE use 1 based, but we use 0 based.
		axis += 1;

	// IF the value is less than the link base, then accept the code as an ode specific parameter and just pass it.
	if (parameterCode > PAL_LINK_PARAM_BASE)
	{
		switch (parameterCode)
		{
		case PAL_LINK_PARAM_ERP:
			parameterCode = dParamERP + dParamGroup * axis;
			break;
		case PAL_LINK_PARAM_STOP_ERP:
			parameterCode = dParamStopERP + dParamGroup * axis;
			break;
		case PAL_LINK_PARAM_CFM:
			parameterCode = dParamCFM + dParamGroup * axis;
			break;
		case PAL_LINK_PARAM_STOP_CFM:
			parameterCode = dParamStopCFM + dParamGroup * axis;
			break;
		case PAL_LINK_PARAM_DOF_MIN:
			parameterCode = dParamLoStop + dParamGroup * axis;
			break;
		case PAL_LINK_PARAM_DOF_MAX:
			parameterCode = dParamHiStop + dParamGroup * axis;
			break;
		default:
			// false means it's a pal code, but it wasn't handled here.
			bool result = false;
			// leave it alone.
			break;
		}
	}

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODESphericalLink::palODESphericalLink() {
}

void palODESphericalLink::InitMotor() {
	if (odeMotorJoint == 0) {
		odeMotorJoint = dJointCreateAMotor(g_world, 0);
		palODEBody *body0 = dynamic_cast<palODEBody *> (GetParentBody());
		palODEBody *body1 = dynamic_cast<palODEBody *> (GetChildBody());
		dJointAttach(odeMotorJoint, body0->odeBody, body1->odeBody);
		dJointSetAMotorNumAxes(odeMotorJoint, 3);
		dJointSetAMotorAxis(odeMotorJoint, 0, 1, 0, 0, 1);
		dJointSetAMotorAxis(odeMotorJoint, 2, 2, 1, 0, 0); //i may need to check this?
		dJointSetAMotorMode(odeMotorJoint, dAMotorEuler);
	}
	if (odeMotorJoint == 0) {
		printf("Motor Failed! on line %d\n", __LINE__);
	}
}

void palODESphericalLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies) {
	SetBodies(parent, child);
	m_FrameA = parentFrame;
	m_FrameB = childFrame;
	// Ode defaults to using the x axis, but the pal definitions says Z, so this takes the Z from the frame.
	CallAnchorAxisInitWithFrames(parentFrame, childFrame, 2, disableCollisionsBetweenLinkedBodies);
}

void palODESphericalLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies) {
	if (GetParent() != parent)
	{
		SetBodies(parent, child);
		ComputeFramesFromPivot(m_FrameA, m_FrameB, pos, axis, palVector3(1.0, 0.0, 0.0));
	}
	palODEBody *body0 = dynamic_cast<palODEBody *> (parent);
	palODEBody *body1 = dynamic_cast<palODEBody *> (child);
	//	printf("%d and %d\n",body0,body1);

	odeJoint = dJointCreateBall(g_world, 0);
	dJointAttach(odeJoint, body0->odeBody, body1->odeBody);

	SetAnchor(pos);
}


void palODESphericalLink::ComputeFrameParent(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameA;
}

void palODESphericalLink::ComputeFrameChild(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameB;
}

void palODESphericalLink::GetAnchor(palVector3& anchor) const
{
	dVector3 anchorOde;
	dJointGetBallAnchor(odeJoint, anchorOde);
	anchor.Set(anchorOde[0], anchorOde[1], anchorOde[2]);
}

void palODESphericalLink::SetAnchor(const palVector3& anchor) {
	dJointSetBallAnchor(odeJoint, anchor.x, anchor.y, anchor.z);
}

bool palODESphericalLink::SetParam(int parameterCode, Float value, int axis) {
	if (axis != -1)
		return false;
	// TODO, convert the limit values based on the rotation of frame.
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		dJointSetBallParam(ODEGetJointID(), parameterCode, dReal(value));
		return dJointGetBallParam(ODEGetJointID(), parameterCode) != 0 || dReal(value) == dReal(0.0);
	}
	return false;
}

Float palODESphericalLink::GetParam(int parameterCode, int axis) {
	if (axis != -1)
		return Float(-1.0f);
	// TODO, convert the limit values based on the rotation of frame.
	if (MapLinkParameterToODEParam(parameterCode, axis))
		return Float(dJointGetBallParam(ODEGetJointID(), parameterCode));
	return Float(-1.0);
}

bool palODESphericalLink::SupportsParameters() const {
	return true;
}

bool palODESphericalLink::SupportsParametersPerAxis() const {
	return false;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODERigidLink::palODERigidLink() {}

void palODERigidLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	m_FrameA = parentFrame;
	m_FrameB = childFrame;
	SetBodies(parent, child);
	// Ode defaults to using the x axis, but the pal definitions says Z, so this takes the Z from the frame.
	CallAnchorAxisInitWithFrames(parentFrame, childFrame, 2, disableCollisionsBetweenLinkedBodies);
}

void palODERigidLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies) {
	if (GetParent() != parent)
	{
		SetBodies(parent, child);
		ComputeFramesFromPivot(m_FrameA, m_FrameB, pos, axis, palVector3(0.0, 0.0, 1.0));
	}
	palODEBody *body0 = dynamic_cast<palODEBody *> (parent);
	palODEBody *body1 = dynamic_cast<palODEBody *> (child);
	//	printf("%d and %d\n",body0,body1);

	if ((!body0) && (!body1)) {
		return; //can't attach two statics
	}

	odeJoint = dJointCreateFixed(g_world, 0);

	if ((body0) && (body1))
		dJointAttach(odeJoint, body0->odeBody, body1->odeBody);
	else {
		if (!body0) {
			dJointAttach(odeJoint, 0, body1->odeBody);
		}
		if (!body1) {
			dJointAttach(odeJoint, body0->odeBody, 0);
		}
	}
}

void palODERigidLink::ComputeFrameParent(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameA;
}

void palODERigidLink::ComputeFrameChild(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameB;
}

bool palODERigidLink::SetParam(int parameterCode, Float value, int axis) {
	if (axis != -1)
		return false;
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		dJointSetFixedParam(ODEGetJointID(), parameterCode, dReal(value));
		return dJointGetFixedParam(ODEGetJointID(), parameterCode) != 0 || dReal(value) == dReal(0.0);
	}
	return false;
}

Float palODERigidLink::GetParam(int parameterCode, int axis) {
	if (axis != -1)
		return Float(-1.0);
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		return Float(dJointGetFixedParam(ODEGetJointID(), parameterCode));
	}
	return Float(-1.0);
}

bool palODERigidLink::SupportsParameters() const {
	return true;
}

bool palODERigidLink::SupportsParametersPerAxis() const {
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODERevoluteLink::palODERevoluteLink()
: m_feedback(0)
{
}

palODERevoluteLink::~palODERevoluteLink() {
	delete m_feedback;
}

void palODERevoluteLink::AddTorque(Float torque) {
	dJointAddHingeTorque(odeJoint, torque);
}

void palODERevoluteLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	// Ode defaults to using the x axis, but the pal definitions says Z, so this takes the Z from the frame.
	CallAnchorAxisInitWithFrames(parentFrame, childFrame, 2, disableCollisionsBetweenLinkedBodies);
}

void palODERevoluteLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies) {
	SetBodies(parent, child);

	palODEBody *body0 = dynamic_cast<palODEBody *> (parent);
	palODEBody *body1 = dynamic_cast<palODEBody *> (child);
	//	printf("%d and %d\n",body0,body1);

	if ((!body0) && (!body1)) {
		return; //can't attach two statics
	}

	odeJoint = dJointCreateHinge(g_world, 0);

	if ((body0) && (body1))
		dJointAttach(odeJoint, body0->odeBody, body1->odeBody);
	else {
		if (!body0) {
			dJointAttach(odeJoint, 0, body1->odeBody);
		}
		if (!body1) {
			dJointAttach(odeJoint, body0->odeBody, 0);
		}
	}

	SetAnchorAxis(pos, axis);
}

void palODERevoluteLink::ComputeFrameParent(palMatrix4x4& frameOut) const
{
	palMatrix4x4 frameExtra;

	static palVector3 defaultAxis(0.0, 0.0, 1.0);
	palVector3 pivot, axis;
	GetAnchorAxis(pivot, axis);

	ComputeFramesFromPivot(frameOut, frameExtra, pivot, axis, defaultAxis);
}

void palODERevoluteLink::ComputeFrameChild(palMatrix4x4& frameOut) const
{
	palMatrix4x4 frameExtra;

	static palVector3 defaultAxis(0.0, 0.0, 1.0);
	palVector3 pivot, axis;
	GetAnchorAxis(pivot, axis);

	ComputeFramesFromPivot(frameExtra, frameOut, pivot, axis, defaultAxis);
}

void palODERevoluteLink::GetAnchorAxis(palVector3& anchor, palVector3& axis) const {
	dVector3 anchorOde, axisOde;
	dJointGetHingeAnchor(odeJoint, anchorOde);
	dJointGetHingeAxis(odeJoint, axisOde);
	anchor.Set(anchorOde[0], anchorOde[1], anchorOde[2]);
	axis.Set(axisOde[0], axisOde[1], axisOde[2]);
}

void palODERevoluteLink::SetAnchorAxis(const palVector3& anchor, const palVector3& axis) {
	dJointSetHingeAnchor(odeJoint, anchor.x, anchor.y, anchor.z);
	dJointSetHingeAxis(odeJoint, axis.x, axis.y, axis.z);
}

palLinkFeedback* palODERevoluteLink::GetFeedback() const throw(palIllegalStateException) {
	if (!odeJoint) {
		throw palIllegalStateException("Init must be called first");
	}
	if (!m_feedback) {
		const_cast<palODERevoluteLink*>(this)->m_feedback = new odeRevoluteLinkFeedback(odeJoint);
	}
	return m_feedback;
}

bool palODERevoluteLink::SetParam(int parameterCode, Float value, int axis) {
	if (axis != -1)
		return false;
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		dJointSetHingeParam(ODEGetJointID(), parameterCode, dReal(value));
		return dJointGetHingeParam(ODEGetJointID(), parameterCode) != 0 || dReal(value) == dReal(0.0);
	}
	return false;
}

Float palODERevoluteLink::GetParam(int parameterCode, int axis) {
	if (axis != -1)
		return -1.0f;

	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		return Float(dJointGetHingeParam(ODEGetJointID(), parameterCode));
	}
	else if (parameterCode == PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE)
	{
		return Float(dJointGetHingeAngle(ODEGetJointID()));
	}
	return Float(-1.0);
}

bool palODERevoluteLink::SupportsParameters() const {
	return true;
}

bool palODERevoluteLink::SupportsParametersPerAxis() const {
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

odeRevoluteLinkFeedback::odeRevoluteLinkFeedback(dJointID odeJoint)
: m_odeJoint(odeJoint),
  m_odeFeedback(0)
{
}

odeRevoluteLinkFeedback::~odeRevoluteLinkFeedback() {
	SetEnabled(false);
	delete m_odeFeedback;
	m_odeFeedback = 0;
	m_odeJoint = 0;
}

bool odeRevoluteLinkFeedback::IsEnabled() const {
	return dJointGetFeedback(m_odeJoint) != 0;
}

bool odeRevoluteLinkFeedback::SetEnabled(bool enable) {
	dJointFeedback* currentFeedback = dJointGetFeedback(m_odeJoint);
	bool enabled;
	if (enable && !currentFeedback) {
		if (!m_odeFeedback) {
			m_odeFeedback = new dJointFeedback;
			memset(m_odeFeedback, 0, sizeof(*m_odeFeedback));
		}
		dJointSetFeedback(m_odeJoint, m_odeFeedback);
		enabled = true;
	}
	else if (!enable && currentFeedback) {
		dJointSetFeedback(m_odeJoint, 0);
		enabled = false;
	}
	return enabled;
}

Float odeRevoluteLinkFeedback::GetValue() const {
	dJointFeedback* currentFeedback = dJointGetFeedback(m_odeJoint);
	Float value;
	if (currentFeedback) {
		value = dLENGTH(currentFeedback->t1) + dLENGTH(currentFeedback->t2);
	}
	else {
		value = 0;
	}

	return value;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODEPrismaticLink::palODEPrismaticLink() {
}

void palODEPrismaticLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	m_FrameA = parentFrame;
	m_FrameB = childFrame;
	CallAnchorAxisInitWithFrames(parentFrame, childFrame, 0, disableCollisionsBetweenLinkedBodies);
}

void palODEPrismaticLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	if (GetParentBody() != parent)
	{
		SetBodies(parent, child);
		ComputeFramesFromPivot(m_FrameA, m_FrameB, pos, axis, palVector3(1.0, 0.0, 0.0));
	}
	palODEBody *body0 = dynamic_cast<palODEBody *> (parent);
	palODEBody *body1 = dynamic_cast<palODEBody *> (child);
	//	printf("%d and %d\n",body0,body1);

	odeJoint = dJointCreateSlider(g_world, 0);
	dJointAttach(odeJoint, body0->odeBody, body1->odeBody);

	SetAxis(axis);
}

void palODEPrismaticLink::ComputeFrameParent(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameA;
}

void palODEPrismaticLink::ComputeFrameChild(palMatrix4x4& frameOut) const
{
	frameOut = m_FrameB;
}

void palODEPrismaticLink::SetAxis(const palVector3& axis) {
	dJointSetSliderAxis(odeJoint, axis.x, axis.y, axis.z);
}


bool palODEPrismaticLink::SetParam(int parameterCode, Float value, int axis) {
	if (axis > -1)
		return false;
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		dJointSetSliderParam(ODEGetJointID(), parameterCode, dReal(value));
		return dJointGetSliderParam(ODEGetJointID(), parameterCode) != 0 || dReal(value) == dReal(0.0);
	}
	return false;
}

Float palODEPrismaticLink::GetParam(int parameterCode, int axis) {
	if (axis > -1)
		return -1.0f;
	if (MapLinkParameterToODEParam(parameterCode, axis))
	{
		return Float(dJointGetSliderParam(ODEGetJointID(), parameterCode));
	}
	else if (parameterCode == PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE)
	{
		return dJointGetSliderPosition(ODEGetJointID());
	}
	return Float(-1.0f);


}

bool palODEPrismaticLink::SupportsParameters() const {
	return true;
}

bool palODEPrismaticLink::SupportsParametersPerAxis() const {
	return false;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODETerrain::palODETerrain() {
	odeGeom = 0;
}

palODETerrain::~palODETerrain() {
	if (odeGeom) {
		dGeomDestroy(odeGeom);
		odeGeom = 0;
	}
}

const palMatrix4x4& palODETerrain::GetLocationMatrix() const {
	memset(&m_mLoc, 0, sizeof(m_mLoc));
	m_mLoc._11 = 1;
	m_mLoc._22 = 1;
	m_mLoc._33 = 1;
	m_mLoc._44 = 1;
	return m_mLoc;
}

palODETerrainPlane::palODETerrainPlane() {
}

const palMatrix4x4& palODETerrainPlane::GetLocationMatrix() const {
	memset(&m_mLoc, 0, sizeof(m_mLoc));
	m_mLoc._11 = 1;
	m_mLoc._22 = 1;
	m_mLoc._33 = 1;
	m_mLoc._44 = 1;
	return m_mLoc;
}

void palODETerrainPlane::Init(Float x, Float y, Float z, Float size) {
	palTerrainPlane::Init(x, y, z, size);
	odeGeom = dCreatePlane(g_space, 0, 1, 0, y);
	dGeomSetData(odeGeom, static_cast<palBodyBase *> (this));
}

palODEOrientatedTerrainPlane::palODEOrientatedTerrainPlane() {
}

void palODEOrientatedTerrainPlane::Init(Float x, Float y, Float z, Float nx, Float ny, Float nz,
		Float min_size) {
	palOrientatedTerrainPlane::Init(x, y, z, nx, ny, nz, min_size);
	odeGeom = dCreatePlane(g_space, nx, ny, nz, CalculateD());
	dGeomSetData(odeGeom, static_cast<palBodyBase *> (this));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODETerrainHeightmap::palODETerrainHeightmap() {
}

void palODETerrainHeightmap::Init(Float px, Float py, Float pz, Float width, Float depth,
		int terrain_data_width, int terrain_data_depth, const Float *pHeightmap) {
#if 0
	palTerrainHeightmap::Init(px,py,pz,width,depth,terrain_data_width,terrain_data_depth,pHeightmap);
	int iTriIndex;
	float fTerrainX, fTerrainZ;
	int x,z;

	dVector3 *vertices; // vertex array for trimesh geom
	int *indices; // index array for trimesh geom
	int vertexcount; // number of vertices in the vertex array
	int indexcount; // number of indices in the index array

	int nv=m_iDataWidth*m_iDataDepth;
	int ni=(m_iDataWidth-1)*(m_iDataDepth-1)*2*3;

	vertexcount=nv;
	indexcount=ni;

	vertices = new dVector3[nv];
	indices = new int[ni];

	// Set the vertex values
	fTerrainZ = -m_fDepth/2;
	for (z=0; z<m_iDataDepth; z++)
	{
		fTerrainX = -m_fWidth/2;
		for (x=0; x<m_iDataWidth; x++)
		{
			//triVertices[x + z*m_iDataWidth].Set(fTerrainX, gfTerrainHeights[x][z], fTerrainZ);
			vertices[x + z*m_iDataWidth][0]=fTerrainX;
			vertices[x + z*m_iDataWidth][1]=pHeightmap[x+z*m_iDataWidth];
			vertices[x + z*m_iDataWidth][2]=fTerrainZ;
			printf("(%d,%d),%f\n",x,z,pHeightmap[x+z*m_iDataWidth]);
			fTerrainX += (m_fWidth / (m_iDataWidth-1));
		}
		fTerrainZ += (m_fDepth / (m_iDataDepth-1));
	}

	int xDim=m_iDataWidth;
	int yDim=m_iDataDepth;
	int y;
	for (y=0;y < yDim-1;y++)
		for (x=0;x < xDim-1;x++) {
			/*
		 //		SetIndex(((x+y*(xDim-1))*2)+0,(y*xDim)+x,(y*xDim)+xDim+x,(y*xDim)+x+1);
		 indices[(((x+y*(xDim-1))*2)+0)*3+0]=(y*xDim)+x;
		 indices[(((x+y*(xDim-1))*2)+0)*3+1]=(y*xDim)+xDim+x;
		 indices[(((x+y*(xDim-1))*2)+0)*3+2]=(y*xDim)+x+1;

		 //		SetIndex(((x+y*(xDim-1))*2)+1,(y*xDim)+x+1,(y*xDim)+xDim+x,(y*xDim)+x+xDim+1);

		 indices[(((x+y*(xDim-1))*2)+1)*3+0]=(y*xDim)+x+1;
		 indices[(((x+y*(xDim-1))*2)+1)*3+1]=(y*xDim)+xDim+x;
		 indices[(((x+y*(xDim-1))*2)+1)*3+2]=(y*xDim)+x+xDim+1;
			 */
			indices[iTriIndex*3+0]=(y*xDim)+x;
			indices[iTriIndex*3+1]=(y*xDim)+xDim+x;
			indices[iTriIndex*3+2]=(y*xDim)+x+1;
			// Move to the next triangle in the array
			iTriIndex += 1;

			indices[iTriIndex*3+0]=(y*xDim)+x+1;
			indices[iTriIndex*3+1]=(y*xDim)+xDim+x;
			indices[iTriIndex*3+2]=(y*xDim)+x+xDim+1;
			// Move to the next triangle in the array
			iTriIndex += 1;
		}

	// build the trimesh data
	dTriMeshDataID data=dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSimple(data,(dReal*)vertices,vertexcount,indices,indexcount);
	// build the trimesh geom
	odeGeom=dCreateTriMesh(g_space,data,0,0,0);
	// set the geom position
	dGeomSetPosition(odeGeom,m_fPosX,m_fPosY,m_fPosZ);
	// in our application we don't want geoms constructed with meshes (the terrain) to have a body
	dGeomSetBody(odeGeom,0);
#else
	palTerrainHeightmap::Init(px, py, pz, width, depth, terrain_data_width, terrain_data_depth,
			pHeightmap);
	int iTriIndex;
	float fTerrainX, fTerrainZ;
	int x, z;

	int nv = m_iDataWidth * m_iDataDepth;
	int ni = (m_iDataWidth - 1) * (m_iDataDepth - 1) * 2 * 3;

	Float *v = new Float[nv * 3];
	int *ind = new int[ni];

	// Set the vertex values
	fTerrainZ = -m_fDepth / 2;
	for (z = 0; z < m_iDataDepth; z++) {
		fTerrainX = -m_fWidth / 2;
		for (x = 0; x < m_iDataWidth; x++) {
			v[(x + z * m_iDataWidth) * 3 + 0] = fTerrainX + m_mLoc._41;
			v[(x + z * m_iDataWidth) * 3 + 1] = pHeightmap[x + z * m_iDataWidth] + m_mLoc._42;
			v[(x + z * m_iDataWidth) * 3 + 2] = fTerrainZ + m_mLoc._43;

			fTerrainX += (m_fWidth / (m_iDataWidth - 1));
		}
		fTerrainZ += (m_fDepth / (m_iDataDepth - 1));
	}

	iTriIndex = 0;
	int xDim = m_iDataWidth;
	int yDim = m_iDataDepth;
	int y;
	for (y = 0; y < yDim - 1; y++)
		for (x = 0; x < xDim - 1; x++) {
			ind[iTriIndex * 3 + 0] = (y * xDim) + x;
			ind[iTriIndex * 3 + 1] = (y * xDim) + xDim + x;
			ind[iTriIndex * 3 + 2] = (y * xDim) + x + 1;
			// Move to the next triangle in the array
			iTriIndex += 1;

			ind[iTriIndex * 3 + 0] = (y * xDim) + x + 1;
			ind[iTriIndex * 3 + 1] = (y * xDim) + xDim + x;
			ind[iTriIndex * 3 + 2] = (y * xDim) + x + xDim + 1;
			// Move to the next triangle in the array
			iTriIndex += 1;
		}
	palODETerrainMesh::Init(px, py, pz, v, nv, ind, ni);

	delete[] v;
	delete[] ind;
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
palODETerrainMesh::palODETerrainMesh() {
}
/*
 palMatrix4x4& palODETerrainMesh::GetLocationMatrix() {
 memset(&m_mLoc,0,sizeof(m_mLoc));
 m_mLoc._11=1;m_mLoc._22=1;m_mLoc._33=1;m_mLoc._44=1;
 m_mLoc._41=m_fPosX;
 m_mLoc._42=m_fPosY;
 m_mLoc._43=m_fPosZ;
 return m_mLoc;
 }
 */

void palODETerrainMesh::Init(Float px, Float py, Float pz, const Float *pVertices, int nVertices,
		const int *pIndices, int nIndices) {
	palTerrainMesh::Init(px, py, pz, pVertices, nVertices, pIndices, nIndices);

	odeGeom = CreateTriMesh(pVertices, nVertices, pIndices, nIndices);
	// set the geom position
	dGeomSetPosition(odeGeom, m_mLoc._41, m_mLoc._42, m_mLoc._43);
	// in our application we don't want geoms constructed with meshes (the terrain) to have a body
	dGeomSetBody(odeGeom, 0);
	dGeomSetData(odeGeom, static_cast<palBodyBase *> (this));

	//delete [] spacedvert;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

palODEMotor::palODEMotor(): m_Link(0), odeJoint(0) {
}

palODEMotor::~palODEMotor() {
	DisableMotor();
}

void palODEMotor::Init(palLink *pLink, int axis) {
	palODERevoluteLink *porl = dynamic_cast<palODERevoluteLink *> (m_Link);
	if (porl) {
		m_Link = porl;
		odeJoint = porl->ODEGetJointID();
		dJointSetHingeParam(odeJoint, dParamFMax, 0.0f);
		dJointSetHingeParam(odeJoint, dParamVel, 0.0f);
	}
}

void palODEMotor::Update(Float targetVelocity, Float Max) {
	if (odeJoint != 0)
	{
		dJointSetHingeParam(odeJoint, dParamFMax, Max);
		dJointSetHingeParam(odeJoint, dParamVel, targetVelocity);
	}
}

palLink *palODEMotor::GetLink() const { return m_Link; }

void palODEMotor::DisableMotor() {
	if (odeJoint != 0) {
		dJointSetHingeParam(odeJoint, dParamFMax, 0.0f);
		dJointSetHingeParam(odeJoint, dParamVel, 0.0f);
		odeJoint = 0;
	}
}

void palODEMotor::Apply(float dt) {

}
