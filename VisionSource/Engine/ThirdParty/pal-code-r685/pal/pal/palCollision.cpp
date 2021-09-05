#include "palCollision.h"
/*
	Abstract:
		PAL - Physics Abstraction Layer.
		Implementation File (collision)

	Author:
		Adrian Boeing
	Revision History:
		Version 0.1   : 05/07/08 - Original
	TODO:
*/

palCollisionDetection::palCollisionDetection(){
}

void palCollisionDetection::GetContacts(palBodyBase *pBody, palContact& contact) const {
	for (auto i = m_vContacts.begin(), iend = m_vContacts.end(); i != iend; ++i) {
		const palContactPoint& curContact = *i;
		if (curContact.m_pBody1 == pBody ||  curContact.m_pBody2 == pBody)
		{
			contact.m_ContactPoints.push_back(curContact);
		}
	}
}

void palCollisionDetection::GetContacts(palBodyBase *a, palBodyBase *b, palContact& contact) const {
	for (auto i = m_vContacts.begin(), iend = m_vContacts.end(); i != iend; ++i) {
		const palContactPoint& curContact = *i;
		if ((curContact.m_pBody1 == a && curContact.m_pBody2 == b) || (curContact.m_pBody1 == b && curContact.m_pBody2 == a))
		{
			contact.m_ContactPoints.push_back(curContact);
		}
	}
}

void palCollisionDetection::EmitContact(palContactPoint& contactPoint)
{
	m_vContacts.push_back(contactPoint);
}

void palCollisionDetection::ClearContacts(palBodyBase* pBody)
{
	m_vContacts.erase(std::remove_if(m_vContacts.begin(), m_vContacts.end(), [pBody](const palContactPoint& curContact)
	{
		return curContact.m_pBody1 == pBody ||  curContact.m_pBody2 == pBody;
	}), m_vContacts.end());
}

void palCollisionDetection::ClearContacts()
{
	m_vContacts.clear();
}


palContactPoint::palContactPoint()
: m_pBody1(NULL)
, m_pBody2(NULL)
, m_fDistance(0.0f) //!< The distance between closest points. Negative distance indicates interpenetrations
, m_fImpulse(0.0f) //!< The impulse magnitude used to resolve the constraints on the bodies along the normal.
{
}

palContact::palContact() {
}

palRayHit::palRayHit() {
	Clear();
}

void palRayHit::Clear() {
	m_bHit = false;
	m_bHitPosition = false;
	m_bHitNormal = false;
	m_pBody = 0;
	m_pGeom = 0;
	m_fDistance = -1;
}

void palRayHit::SetHitPosition(Float x, Float y, Float z) {
	m_bHitPosition = true;
	m_vHitPosition.x=x;
	m_vHitPosition.y=y;
	m_vHitPosition.z=z;
}
void palRayHit::SetHitNormal(Float x, Float y, Float z) {
	m_bHitNormal = true;
	m_vHitNormal.x = x;
	m_vHitNormal.y = y;
	m_vHitNormal.z = z;
}

palRayHitCallback::palRayHitCallback() {
}

palCollisionDetectionExtended::palCollisionDetectionExtended() {

}

std::ostream& operator<<(std::ostream &os, const palContactPoint& cp)
{
    os << "palContactPoint[body1=0x" << std::hex << cp.m_pBody1 << ",body2=0x" << cp.m_pBody2
       << ",contactPosition=" << cp.m_vContactPosition << ",contactNormal="
       << cp.m_vContactNormal << ",distance=" << cp.m_fDistance << ",impulse="
       << cp.m_fImpulse << "]";
    return os;
}
