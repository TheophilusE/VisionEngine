#include "palFactory.h"
#include <memory.h>
#include <cmath>
#include <cfloat>
#include <iostream>
#include <sstream>

/*
	Abstract:
		PAL - Physics Abstraction Layer.
		Implementation File (bodies)

	Author:
		Adrian Boeing
	Revision History:
		Version 0.1 :19/10/07 split from pal.cpp
	TODO:
*/

#ifdef MEMDEBUG
#include <crtdbg.h>
#define new new(_NORMAL_BLOCK,__FILE__, __LINE__)
#endif

std::string palLinkFeedback::toString() const
{
	std::ostringstream oss;
		oss << "LinkFeedback[enabled=" << IsEnabled();
		if (IsEnabled()) {
			oss << ";value=" << GetValue();
		}
		oss << "]";
	    return oss.str();
}

palLink::palLink()
    : m_Type(PAL_LINK_NONE), m_pParent(0), m_pChild(0)
{
}

palLink::palLink(palLinkType linkType)
    : m_Type(linkType), m_pParent(0), m_pChild(0)
{
}

palLink::~palLink()
{
}
 
void palLink::SetBodies(palBodyBase *parent, palBodyBase *child)
{
    m_pParent = parent;
    m_pChild = child;
}

// Doesn't support anything by default.
bool palLink::SetParam(int /*parameterCode*/, Float /*value*/, int /*axis*/) { return false; }
Float palLink::GetParam(int /*parameterCode*/, int /*axis*/) { return -1; }
bool palLink::SupportsParameters() const { return false; }
bool palLink::SupportsParametersPerAxis() const { return false; }


std::string palLink::toString() const
{
    std::ostringstream oss;
	oss << "Link[type=" << m_Type << ",parent=" << m_pParent << ",child=" << m_pChild
        << " ]";
    return oss.str();
}

palLinkFeedback* palLink::GetFeedback() const throw(palIllegalStateException) {
	return 0;
}

void palLink::ComputeFramesFromPivot(palMatrix4x4& frameA, palMatrix4x4& frameB, const palVector3& pivot, const palVector3& axis, const palVector3& constraintDefaultAxis) const
{
	/* Even though we'll only use the location of the pivot and not
	 * its orientation, we need to account for rotation of the parent
	 * and child bodies because we need the location of the pivot in
	 * their frames of reference, which might be rotated. (For
	 * example, if the parent is translated by (-5,0,0) and rotated 90
	 * degrees clockwise about z, the global origin isn't just
	 * translated for the parent to (5,0,0), it's rotated to be at
	 * (0,5,0) in the parent's coordinate system.) */

	//Rotation to align x with axis
	Float angle = std::acos(vec_dot(&constraintDefaultAxis, &axis));
	palVector3 direction;
	vec_cross(&direction, &constraintDefaultAxis, &axis);
	Float vecMag = vec_norm(&direction);

	palMatrix4x4 ctWorldTransform;
	mat_identity(&ctWorldTransform);
	mat_set_translation(&ctWorldTransform, pivot.x, pivot.y, pivot.z);
	//This takes care of the case of busted direction vec.
	mat_rotate(&ctWorldTransform, angle, direction.x, direction.y, direction.z);

	palMatrix4x4 worldToBody;

	mat_invert(&worldToBody, &m_pParent->GetLocationMatrix());
	mat_multiply(&frameA, &worldToBody, &ctWorldTransform);

	mat_invert(&worldToBody, &m_pChild->GetLocationMatrix());
	mat_multiply(&frameB, &worldToBody, &ctWorldTransform);
}

void palLink::GetPosition(palVector3& pos) const {
	//Convert link_rel to the global coordinate system
	//Link_abs=(Link_rel * R_Inv) - parent_abs

	palMatrix4x4 frameA;
	ComputeFrameParent(frameA);

	//Transpose the matrix to get Normal rotation matrixes.
	palMatrix4x4 a_PAL = m_pParent->GetLocationMatrix();

	palMatrix4x4 result;
	mat_multiply(&result, &a_PAL, &result);
	palVector3 posVec;
	mat_get_translation(&result, &pos);
}

void palLink::CallAnchorAxisInitWithFrames(const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, unsigned axisRow, bool disableCollisionsBetweenLinkedBodies)
{
	// must be 0, 1, or 2
	if (axisRow > 2)
	{
		axisRow = 2;
	}

	palMatrix4x4 worldJointXform, childFrameInvert, newChildLocation;
	mat_multiply(&worldJointXform, &m_pParent->GetLocationMatrix(), &parentFrame);

	mat_multiply(&worldJointXform, &m_pChild->GetLocationMatrix(), &parentFrame);
	mat_invert(&childFrameInvert, &childFrame);
	mat_multiply(&newChildLocation, &worldJointXform, &childFrameInvert);

	// Have to move the "child" for the frame to be correct because ode looks for a world position for an anchor and updates the bodies based on their current positions.
	m_pChild->SetPosition(newChildLocation);

	palVector3 anchor, axis;
	mat_get_translation(&worldJointXform, &anchor);
	mat_get_row(&worldJointXform, &axis, axisRow);
	Init(m_pParent, m_pChild, anchor, axis, disableCollisionsBetweenLinkedBodies);
}

palSpringDesc::palSpringDesc()
: m_fDamper(0.0)
, m_fSpringCoef(0.0)
, m_fTarget(0.0)
{
}

palSphericalLink::palSphericalLink()
    : palLink(PAL_LINK_SPHERICAL)
{
}

palSphericalLink::~palSphericalLink()
{
}

//#define PRL_DEBUG

#define PV(x) printf(#x);printPalVector(x);
#define PQ(x) printf(#x);printPalQuaternion(x);

palRevoluteLink::palRevoluteLink()
: palLink(PAL_LINK_REVOLUTE)
{
}

palRevoluteLink::~palRevoluteLink()
{
}

palVector3 palRevoluteLink::GetAxis() const {
	palMatrix4x4 a = GetParentBody()->GetLocationMatrix();
	palMatrix4x4 frameA;
	ComputeFrameParent(frameA);
	palVector3 axis;
	mat_get_row(&frameA, &axis, 2);

	palVector3 axisWorld;
	vec_mat_mul(&axisWorld,&a,&axis);
	return axisWorld;
}


void palRevoluteLink::ApplyTorque(Float torque) {
	Float t0,t1,t2;
	palVector3 axis = GetAxis();
	t0= axis.x * torque;
	t1= axis.y * torque;
	t2= axis.z * torque;
	palBody * pb =dynamic_cast<palBody *>(GetParentBody());
	palBody * cb =dynamic_cast<palBody *>(GetChildBody());
	if (pb)
		pb->ApplyTorque(t0,t1,t2);
	if (cb)
		cb->ApplyTorque(-t0,-t1,-t2);
}

void palRevoluteLink::ApplyAngularImpulse(Float torque) {
	palMatrix4x4 a = GetParentBody()->GetLocationMatrix();
	palVector3 axis = GetAxis();
	palVector3 axisA;
	vec_mat_mul(&axisA,&a,&axis);
	vec_mul(&axisA,torque);

	palBody * pb =dynamic_cast<palBody *>(GetParentBody());
	palBody * cb =dynamic_cast<palBody *>(GetChildBody());
	if (pb)
		pb->ApplyAngularImpulse(axisA.x, axisA.y, axisA.z);
	if (cb)
		cb->ApplyAngularImpulse(-axisA.x,-axisA.y,-axisA.z);
}

Float palRevoluteLink::GetAngularVelocity() const {
	palVector3 av1,av2,axis;
	palBody *pb =dynamic_cast<palBody *>(GetParentBody());
	palBody *cb =dynamic_cast<palBody *>(GetChildBody());
	vec_set(&av1,0,0,0);
	vec_set(&av2,0,0,0);
	if (pb)
		pb->GetAngularVelocity(av1);
	if (cb)
		cb->GetAngularVelocity(av2);

	axis = GetAxis();

	Float rate;
	rate =vec_dot(&axis,&av1);
	rate-=vec_dot(&axis,&av2);
	return rate;
}

Float palRevoluteLink::GetAngle() const {

	if (GetParentBody()==NULL) return 0.0f;
	if (GetChildBody() ==NULL) return 0.0f;

	//palMatrix4x4 a_PAL,b_PAL;
	palMatrix4x4 a,b;
	a=GetParentBody()->GetLocationMatrix();
	b=GetChildBody()->GetLocationMatrix();

	palMatrix4x4 frameA, frameB;
	ComputeFrameParent(frameA);
	ComputeFrameChild(frameB);

	palVector3 fac0;
	mat_get_row(&frameA,&fac0,0);
	palVector3 refAxis0;
	vec_mat_mul(&refAxis0,&a,&fac0);

	palVector3 fac1;
	mat_get_row(&frameA,&fac1,1);
	palVector3 refAxis1;
	vec_mat_mul(&refAxis1,&a,&fac1);

	palVector3 fbc1;
	mat_get_row(&frameB,&fbc1,1);
	palVector3 swingAxis;
	vec_mat_mul(&swingAxis,&b,&fbc1);

	Float d0 = vec_dot(&swingAxis,&refAxis0);
	Float d1 = vec_dot(&swingAxis,&refAxis1);
	return std::atan2(d0,d1);
}

std::string palRevoluteLink::toString() const
{
    std::ostringstream oss;
    oss << palLink::toString()
		<< "[angle=" << GetAngle()
		<< ",omega=" << GetAngularVelocity()
        << ",axis=" << GetAxis()
		<< "]";
    return oss.str();
}

palPrismaticLink::palPrismaticLink()
    : palLink(PAL_LINK_PRISMATIC)
{
}


palPrismaticLink::~palPrismaticLink()
{
}

palGenericLink::palGenericLink()
    : palLink(PAL_LINK_GENERIC)
{
}


palGenericLink::~palGenericLink()
{
}


palRigidLink::palRigidLink()
    : palLink(PAL_LINK_RIGID)
{
}

palRigidLink::~palRigidLink()
{
}

void palRigidLink::Init(palBodyBase *parent, palBodyBase *child, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	palVector3 parentPos, childPos;
	const palMatrix4x4& locParent(parent->GetLocationMatrix());
	const palMatrix4x4& locChild(child->GetLocationMatrix());
	mat_get_translation(&locParent, &parentPos);
	mat_get_translation(&locChild, &childPos);

	palAxis upAxis = dynamic_cast<palPhysics*>(GetParent())->GetUpAxis();

	palMatrix4x4 locParentInv, locChildInv, locPivot;
	mat_invert(&locParentInv, &locParent);
	mat_invert(&locChildInv, &locChild);

	parent->GetPosition(parentPos);
	child->GetPosition(childPos);
	palVector3 pivot = (parentPos + childPos) / 2;
	mat_identity(&locPivot);
	mat_set_translation(&locPivot, pivot.x, pivot.y, pivot.z);
	palMatrix4x4 frameA, frameB;

	mat_multiply(&frameA, &locParentInv, &locPivot);
	mat_multiply(&frameB, &locChildInv, &locPivot);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}

