/*
 * palBodyTest.cpp
 *
 *  Created on: Jul 1, 2010
 *      Author: Chris Long
 *  Copyright (C) 2010 SET Corporation
 */


#include <pal/pal.h>
#include <pal/palFactory.h>
#include <iostream>
#include <cstdlib>
#include <float.h>
#include <assert.h>

int main(int argc, char* argv[])
{
	//std::cout << "pAMT: type_info for pAM = " << &typeid(palMotor) << std::endl;
	PF->LoadPhysicsEngines();
	PF->SelectEngine("Bullet");		 // Here is the name of the physics engine you wish to use. You could replace DEFAULT_ENGINE with "Tokamak", "ODE", etc...
	palPhysics *pp = PF->CreatePhysics(); //create the main physics class
	if (pp == NULL) {
		std::cerr << "Failed to create the physics engine. Check to see if you spelt the engine name correctly, or that the engine DLL is in the right location" << std::endl;
		return 1;
	}
	palPhysicsDesc desc;
	pp->Init(desc); //initialize it, set the main gravity vector

	palFactoryObject* obj = PF->CreateObject("palMotor");
	palMotor* Motor = dynamic_cast<palMotor*>(obj);
	//std::cout << "pAMT: type_info for pAM = " << &typeid(palMotor) << std::endl;

	assert(Motor != 0);
	std::cout << "Motor = " << Motor << std::endl;
	
	PF->Cleanup();
	std::cout << "success" << std::endl;
	return 0;
}
