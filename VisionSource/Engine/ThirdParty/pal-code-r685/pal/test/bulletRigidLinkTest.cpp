/*
 * rigidLinkTest.cpp
 *
 *  Created on: Apr 9, 2010
 *      Author: Chris Long
 *  Copyright (C) 2010 SET Corporation
 */

#include <pal/pal.h>
#include <pal/palFactory.h>
#include <iostream>
#include <cmath>
#include <pal_i/bullet/bullet_pal.h>
#include <pal_i/bullet/bullet_palLinks.h>

const int STEPS = 20;

int main(int argc, char* argv[])
{
	PF->LoadPhysicsEngines();
	PF->SelectEngine("Bullet");		 // Here is the name of the physics engine you wish to use. You could replace DEFAULT_ENGINE with "Tokamak", "ODE", etc...
	palPhysics *pp = PF->CreatePhysics(); //create the main physics class
	if (pp == NULL) {
		std::cerr << "Failed to create the physics engine. Check to see if you spelt the engine name correctly, or that the engine DLL is in the right location" << std::endl;
		return 1;
	}
	palPhysicsDesc desc;
	desc.m_vGravity = palVector3(0.0f, 0.0f, 0.0f);
	pp->Init(desc); //initialize it, set the main gravity vector

	palMatrix4x4 mat;
	mat_identity(&mat);
	mat_set_translation(&mat, 0.0f, 0.0f, 6.0f);
	mat_set_rotation(&mat, M_PI, 0.0f, 0.0f);

	palCustomConcaveGeometry* pcc = PF->CreateCustomConcaveGeometry();
	if (pcc == NULL)
	{
		std::cerr << "Failed to create a custom concave geometry" << std::endl;
	}

	palGenericBody* boxA = PF->CreateGenericBody();
	palBoxGeometry* boxGeom = PF->CreateBoxGeometry();
	boxA->SetMass(100.0f);
	boxGeom->Init(mat, 10.0f, 10.0f, 10.0f, 100.0f);
	boxA->Init(mat);
	boxA->ConnectGeometry(boxGeom);

	mat_identity(&mat);
	mat_set_translation(&mat, 20.0f, 0.0f, 5.0f);
	mat_set_rotation(&mat, M_PI, M_PI, M_PI);

	palGenericBody* boxB = PF->CreateGenericBody();
	palBoxGeometry* boxGeomB = PF->CreateBoxGeometry();
	boxGeomB->Init(mat, 1.0f, 1.0f, 1.0f, 1.0f);
	boxB->SetMass(1.0f);
	boxB->Init(mat);
	boxB->ConnectGeometry(boxGeomB);

	palRigidLink* link = PF->CreateRigidLink(boxA, boxB);

	palTerrainPlane *pt= PF->CreateTerrainPlane(); //create the ground
	pt->Init(0,0,0,50.0f); //initialize it, set its location to 0,0,0 and minimum size to 50

	palBulletRigidLink* brLink = dynamic_cast<palBulletRigidLink*>(link);

	for (int i = 0; i < STEPS; i++) {
		palVector3 aPos;
		boxA->GetPosition(aPos);
		palVector3 bPos;
		boxB->GetPosition(bPos);

		std::cout << i << "\t" << aPos << "\t" << bPos << "\t" << "\t" << "\t" << std::endl;
		pp->Update(0.05);
	}
	PF->Cleanup();
	return 0;
}
