/*
 * palMaterials.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: David Guthrie
 */
#include "palFactory.h"
#include "palMaterials.h"

FACTORY_CLASS_IMPLEMENTATION(palMaterials);
FACTORY_CLASS_IMPLEMENTATION(palMaterial);
FACTORY_CLASS_IMPLEMENTATION(palMaterialInteraction);

palMaterialInteractionCollisionCallback::~palMaterialInteractionCollisionCallback() {}

palMaterialDesc::palMaterialDesc()
: m_fStatic(0.0)
, m_fKinetic(0.0)
, m_fRestitution(0.5)
, m_vDirAnisotropy(1.0f, 0.0f, 0.0f)
, m_bEnableAnisotropicFriction(false)
, m_bDisableStrongFriction(false)
{
	for (unsigned i = 0; i < 3; ++i)
	{
		m_vStaticAnisotropic._vec[i] = 1.0f;
		m_vKineticAnisotropic._vec[i] = 1.0f;
	}
}

void palMaterialDesc::SetParameters(const palMaterialDesc& matDesc) {
	m_fStatic = matDesc.m_fStatic;
	m_fKinetic = matDesc.m_fKinetic;
	m_fRestitution = matDesc.m_fRestitution;
	m_vStaticAnisotropic = matDesc.m_vStaticAnisotropic;
   m_vKineticAnisotropic = matDesc.m_vKineticAnisotropic;
   m_vDirAnisotropy = matDesc.m_vDirAnisotropy;
	m_bEnableAnisotropicFriction = matDesc.m_bEnableAnisotropicFriction;
   m_bDisableStrongFriction = matDesc.m_bDisableStrongFriction;
}

palMaterial::palMaterial()
: m_Name()
, m_Id(0)
, m_bHasCustomMaterialInteractions(false)
{}

palMaterial::palMaterial(const palMaterial& m):m_Id(0),m_bHasCustomMaterialInteractions(false) {}

palMaterial::~palMaterial() {}

void palMaterial::SetHasCustomInteractions(bool enabled) { m_bHasCustomMaterialInteractions = enabled; }


void palMaterial::Init(const PAL_STRING& name, const palMaterialDesc& matDesc) {
	SetParameters(matDesc);
	m_Name=name;
}

const PAL_STRING& palMaterial::GetName() const
{
	return m_Name;
}

palMaterialInteraction::palMaterialInteraction()
: m_pMaterial1(), m_pMaterial2(), m_pCollsionCallback() {
}
palMaterialInteraction::palMaterialInteraction(const palMaterialInteraction& pmi)
: m_pMaterial1(pmi.m_pMaterial1), m_pMaterial2(pmi.m_pMaterial2), m_pCollsionCallback(pmi.m_pCollsionCallback) {}

palMaterialInteraction& palMaterialInteraction::operator=(const palMaterialInteraction& pmi) {
	m_pMaterial1 = pmi.m_pMaterial1;
	m_pMaterial2 = pmi.m_pMaterial2;
	return *this;
}

void palMaterialInteraction::Init(palMaterial* pM1, palMaterial* pM2, const palMaterialDesc& matDesc) {
	palMaterialDesc::SetParameters(matDesc);
	m_pMaterial1 = pM1;
	m_pMaterial2 = pM2;
	pM1->SetHasCustomInteractions(true);
	pM2->SetHasCustomInteractions(true);
}

palMaterials::palMaterials() {
};

palMaterials::~palMaterials() {
	// Ugly memory cleanup workaround.
	palPhysics* physics = dynamic_cast<palPhysics*>(GetParent());
	if (physics != 0) physics->SetMaterialsNull();
}

unsigned palMaterials::GetIndex(const PAL_STRING& name) const {
//	PAL_VECTOR<PAL_STRING>::iterator obj;
//	obj = std::find(m_MaterialNames.begin(), m_MaterialNames.end(), name);
	unsigned size = m_Materials.size();
	for (unsigned int i=0; i < size ;i++)
		if (m_Materials[i]->GetName() == name)
			return i;
	return UINT_MAX;
}

palMaterial* palMaterials::GetMaterial(const PAL_STRING& name) {
	unsigned pos = GetIndex(name);
	if (pos == UINT_MAX) return NULL;
	palMaterial *pM= m_Materials[pos];
	return pM;
}

bool palMaterials::HandleCustomInteraction(palMaterial* pm1, palMaterial* pm2, palMaterialDesc& matToAdjust, palContactPoint& contactToAdjust, bool combine)
{
	bool result = false;
	if (pm1 == NULL)
	{
		return false;
	}
	if (pm2 == NULL)
	{
		return false;
	}
	if (pm1 == pm2 && combine)
	{
		CombineMaterials(*pm1, *pm2, matToAdjust);
		result = true;
	}

	if (!result && pm1->GetHasCustomInteractions() && pm2->GetHasCustomInteractions())
	{
		palMaterialInteraction* pMI = m_MaterialInteractions.Get(pm1->GetId(), pm2->GetId()).m_pMatInteration;
		// The constructor sets this value to null, so check that to see if it's uninitialized.
		if (pMI != NULL)
		{
			palMaterialInteractionCollisionCallback* callback = pMI->GetCollisionCallback();
			if (callback != NULL)
			{
				result = (*callback)(*pMI, matToAdjust, contactToAdjust);
			}
			else
			{
				matToAdjust.SetParameters(*pMI);
				result = true;
			}
		}
	}

	if (!result && combine)
	{
		CombineMaterials(*pm1, *pm2, matToAdjust);
		result = true;
	}

	return result;
}


void palMaterials::CombineMaterials(const palMaterialDesc& one, const palMaterialDesc& two, palMaterialDesc& result) {
	result.m_bDisableStrongFriction = one.m_bDisableStrongFriction && two.m_bDisableStrongFriction;
	result.m_fKinetic = one.m_fKinetic * two.m_fKinetic;
	result.m_fStatic = one.m_fStatic * two.m_fStatic;
	result.m_fRestitution = one.m_fRestitution * two.m_fRestitution;
	result.m_bEnableAnisotropicFriction = one.m_bEnableAnisotropicFriction || two.m_bEnableAnisotropicFriction;

	// Combining anisotropic friction makes no sense, so we pick one.
	if (one.m_bEnableAnisotropicFriction) {
		result.m_vKineticAnisotropic = one.m_vKineticAnisotropic;
		result.m_vStaticAnisotropic = one.m_vStaticAnisotropic;
	} else if (two.m_bEnableAnisotropicFriction) {
		result.m_vKineticAnisotropic = two.m_vKineticAnisotropic;
		result.m_vStaticAnisotropic = two.m_vStaticAnisotropic;
	}
}

palMaterial* palMaterials::NewMaterial(const PAL_STRING& name, const palMaterialDesc& matDesc) {
	if (GetIndex(name)<UINT_MAX)
	{
		SET_WARNING("Can not replace existing materials!");
		return NULL;
	}

	palMaterial *pM = NULL;
	pM = PF->CreateObject<palMaterial>("palMaterial");
	if (pM == NULL) {
		SET_ERROR("Could not create material");
		return NULL;
	}
	pM->Init(name,matDesc);

	unsigned pos = m_Materials.size();
	m_Materials.push_back(pM);
	pM->SetId(pos);

	m_MaterialInteractions.Resize(pos+1, pos+1);

	return pM;
}


void palMaterials::SetMaterialInteraction(palMaterial* pm1, palMaterial* pm2, const palMaterialDesc& matDesc) {
	unsigned x,y, id1=0, id2=0;
	m_MaterialInteractions.GetDimensions(x,y);
	id1 = (pm1 != NULL) ? pm1->GetId(): UINT_MAX;
	id2 = (pm2 != NULL) ? pm2->GetId(): UINT_MAX;
	if (id1 > x || id2 > y)
	{
		return;
	}

	palMaterialInteraction* pMI = m_MaterialInteractions.Get(id1, id2).m_pMatInteration;
	// The constructor sets this value to null, so check that to see if it's uninitialized.
	if (pMI == NULL)
	{
		palMaterialInteraction* pMI = PF->CreateObject<palMaterialInteraction>("palMaterialInteraction");
		if (pMI == NULL) {
			SET_ERROR("Could not create material");
			return;
		}
		pMI->Init(m_Materials[id1], m_Materials[id2], matDesc);
		m_MaterialInteractions.Get(id1,id2).m_pMatInteration = pMI;
		m_MaterialInteractions.Get(id2,id1).m_pMatInteration = pMI;
	}
	else
	{
		pMI->SetParameters(matDesc);
	}
}
void palMaterials::SetMaterialInteraction(const PAL_STRING& name1, const PAL_STRING& name2, const palMaterialDesc& matDesc) {
	palMaterial* p1=GetMaterial(name1);
	palMaterial* p2=GetMaterial(name2);
	if (p1 != NULL && p2 != NULL)
	{
		SetMaterialInteraction(p1, p2, matDesc);
	}
}

palMaterialInteraction* palMaterials::GetMaterialInteraction(const PAL_STRING& name1, const PAL_STRING& name2)
{
	palMaterial* pm1 = GetMaterial(name1);
	palMaterial* pm2 = GetMaterial(name2);
	return GetMaterialInteraction(pm1, pm2);
}

palMaterialInteraction* palMaterials::GetMaterialInteraction(palMaterial* pm1, palMaterial* pm2)
{
	if (pm1 == NULL || pm2 == NULL)
	{
		return NULL;
	}
	return  m_MaterialInteractions.Get(pm1->GetId(), pm2->GetId()).m_pMatInteration;
}

////////////////////////////////////////



