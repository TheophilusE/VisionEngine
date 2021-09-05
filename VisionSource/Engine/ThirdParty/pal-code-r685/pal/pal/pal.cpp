//#include "pal.h"
#include "palFactory.h"
#include <algorithm>
#include <iostream>
/*
	Abstract:
		PAL - Physics Abstraction Layer.
		Implementation File

	Author:
		Adrian Boeing
	Revision History:
		Version 0.84:19/09/06 GPS, remerged
		Version 0.83:17/02/05 velocimeter update
		Version 0.82:16/02/05 Changed velocimeter to relative coordinates
		Version 0.81:10/06/04 Correction to palBody::SetPosition
		Version 0.8 : 3/06/04
	TODO:
		-saferize vertex copyign for terrain heightmap and mesh
		-defines for infninity for joint limts
*/

#ifdef MEMDEBUG
#include <crtdbg.h>
#define new new(_NORMAL_BLOCK,__FILE__, __LINE__)
#endif


void palPhysics::SetFactoryInstance(palFactory *pf) {
	palFactory::SetInstance(pf);
}

void palPhysics::AddAction(palAction *action) {
	if (action != NULL)
		m_Actions.push_back(action);
}

void palPhysics::RemoveAction(palAction *action) {
	if (action != NULL)
	{
		PAL_LIST<palAction*>::iterator found = std::find(m_Actions.begin(),m_Actions.end(),action);
		if (found != m_Actions.end()) {
			m_Actions.erase(found);
		}
	}
}

void palPhysics::SetDebugDraw(palDebugDraw* debugDraw) {
	m_pDebugDraw = debugDraw;
}

palDebugDraw* palPhysics::GetDebugDraw() {
	return m_pDebugDraw;
}

struct ActionCaller {
	void operator()(palAction *action)
	{
		(*action)(m_fTimeStep);
	}
	float m_fTimeStep;
};

void palPhysics::CallActions(Float timestep) {
	ActionCaller ac;
	ac.m_fTimeStep = timestep;
	std::for_each(m_Actions.begin(), m_Actions.end(), ac);
}

#if 0
void paldebug_printmatrix(palMatrix4x4 *pm) {
	for (int i=0;i<16;i++) {
		printf("%f ",pm->_mat[i]);
		if (i%4 == 3)
			printf("\n");
	}
}
void paldebug_printvector3(palVector3 *pv) {
	printf("%f %f %f\n",pv->x,pv->y,pv->z);
}

//void paldebug_printvector4(palVector4 *pv) {
//	printf("%f %f %f %f\n",pv->x,pv->y,pv->z,pv->w);
//}
#endif

void palSphere::GenericInit(palMatrix4x4 &pos, void *param_array) {
	Float *p=(Float *)param_array;
	Init(pos._41,pos._42,pos._43,p[0],p[1]);
	//SetPosition(pos);
}

void palCapsule::GenericInit(palMatrix4x4 &pos, void *param_array) {
	Float *p=(Float *)param_array;
	Init(pos._41,pos._42,pos._43,p[0],p[1],p[2]);
	//SetPosition(pos);
}

void palCompoundBody::GenericInit(palMatrix4x4 &pos, void *param_array) {
	Init(pos._41,pos._42,pos._43);
	//SetPosition(pos);
}

/*
void palBox::GenericInit(void *param, ...) {
	Float p[7];
	va_list args;
	va_start( args, param);

	void *ptr;
	char *szParam;

	p[0]=atof( (char *)param );
	for (int i=1;i<7;i++) {
		ptr = va_arg( args, void *);
		szParam = (char *)ptr;
		p[i]=atof(szParam);
	}
	this->Init(p[0],p[1],p[2], p[3],p[4],p[5], p[6]);
}
*/
/*
void palSphere::GenericInit(void *param, ...) {
	Float p[5];
	va_list args;
	va_start( args, param);

	void *ptr;
	char *szParam;

	p[0]=atof( (char *)param );
	for (int i=1;i<5;i++) {
		ptr = va_arg( args, void *);
		szParam = (char *)ptr;
		p[i]=atof(szParam);
	}
	this->Init(p[0],p[1],p[2], p[3],p[4]);
}
*/

/*
void palCapsule::GenericInit(void *param, ...) {
	Float p[6];
	va_list args;
	va_start( args, param);

	void *ptr;
	char *szParam;

	p[0]=atof( (char *)param );
	for (int i=1;i<6;i++) {
		ptr = va_arg( args, void *);
		szParam = (char *)ptr;
		p[i]=atof(szParam);
	}
	this->Init(p[0],p[1],p[2], p[3],p[4],p[5]);
}
*/


/*
void palTriangleMesh::Init(Float x, Float y, Float z, const Float *pVertices, int nVertices, const int *pIndices, int nIndices) {
	palBody::SetPosition(x,y,z);
	m_nVertices=nVertices;
	m_nIndices=nIndices;
	m_pVertices=(Float *) pVertices;
	m_pIndices=(int *) pIndices;
}*/

////////////////////////////////////////
////////////////////////////////////////
const FLOAT palPhysicsDesc::DEFAULT_GRAVITY_X = 0.0f;
const FLOAT palPhysicsDesc::DEFAULT_GRAVITY_Y = -9.80665f; //!< Standard gravity, according to NIST Special Publication 330, p. 39
const FLOAT palPhysicsDesc::DEFAULT_GRAVITY_Z = 0.0f;

palPhysicsDesc::palPhysicsDesc()
	: m_vGravity(DEFAULT_GRAVITY_X, DEFAULT_GRAVITY_Y, DEFAULT_GRAVITY_Z), m_nUpAxis(PAL_Y_AXIS)
{
}

////////////////////////////////////////
////////////////////////////////////////
void palPhysics::Init(const palPhysicsDesc& desc) {
	m_fGravityX=desc.m_vGravity.x;
	m_fGravityY=desc.m_vGravity.y;
	m_fGravityZ=desc.m_vGravity.z;
	m_nUpAxis=desc.m_nUpAxis;

	// if the up axis is out of bounds, just set it to the default.
	if (m_nUpAxis >= PAL_AXIS_COUNT)
	{
		m_nUpAxis = PAL_Y_AXIS;
	}

	m_Properties=desc.m_Properties;

	m_pMaterials = palFactory::GetInstance()->CreateObject<palMaterials>("palMaterials");
}

palPhysics::palPhysics()
  : m_bListen(false), m_fGravityX(0), m_fGravityY(0), m_fGravityZ(0), m_fLastTimestep(0),
    m_fTime(0), m_nUpAxis(PAL_Y_AXIS), m_pMaterials(0), m_pDebugDraw(0) {
}

palPhysics::~palPhysics() {
	delete m_pMaterials;
	m_pMaterials = 0;
}

void palPhysics::GetPropertyDocumentation(PAL_MAP<PAL_STRING, PAL_STRING>& docOut) const
{
}

void palPhysics::Update(Float timestep) {
#ifdef INTERNAL_DEBUG
	std::cout << "palPhysics::Update: timestep = " << timestep << " (==0.02? " << (timestep == 0.02f) << ")" << std::endl;
#endif
	if (GetDebugDraw() != NULL) {
		GetDebugDraw()->Clear();
	}
	CallActions(timestep);
	Iterate(timestep);
	m_fTime+=timestep;
	m_fLastTimestep=timestep;
}

palTerrainType palTerrain::GetType() const {
	return m_Type;
}

void palPhysics::GetGravity(palVector3& g) const {
	g.x = m_fGravityX;
	g.y = m_fGravityY;
	g.z = m_fGravityZ;
}

Float palPhysics::GetTime() const {
	return m_fTime;
}

Float palPhysics::GetLastTimestep() const {
	return m_fLastTimestep;
}

void palPhysics::SetGroupCollision(palGroup a, palGroup b, bool enabled) {
}

//virtual void NotifyGeometryAdded(palGeometry* pGeom);
//virtual void NotifyBodyAdded(palBodyBase* pBody);
void palPhysics::NotifyGeometryAdded(palGeometry* pGeom) {
	//m_Geometries.push_back(pGeom);
}

void palPhysics::NotifyBodyAdded(palBodyBase* pBody) {
	//m_Bodies.push_back(pBody);
}

const PAL_STRING& palPhysics::GetInitProperty(const PAL_STRING& name, const PAL_STRING& defaultVal) const
{
	PropertyMap::const_iterator i = m_Properties.find(name);
	if (i == m_Properties.end())
	{
		return defaultVal;
	}
	return i->second;
}

palSolver* palPhysics::asSolver() { return 0; }
palCollisionDetection* palPhysics::asCollisionDetection() { return 0; }

