#ifndef BULLET_PALVEHICLE_H
#define BULLET_PALVEHICLE_H
//(c) Adrian Boeing 2007, see liscence.txt (BSD liscence)
/*
	Abstract:
		PAL - Physics Abstraction Layer. Bullet vehicle implementation.
		This enables the use of bullet vehicles via PAL.
	Author:
		Adrian Boeing
	Revision History:
	Version 0.0.1 : 17/08/07 - Vehicle and wheel
	TODO:
		- motor,gears,etc.
	notes:
*/



#include "bullet_pal.h"
#include "../pal/palVehicle.h"

class palBulletWheel : public palWheel {
public:
	palBulletWheel();
	virtual void Init(const palWheelInfo& wheelInfo);
	virtual palMatrix4x4& GetLocationMatrix();
	btRaycastVehicle*	m_vehicle;
	int m_WheelIndex;
};

class palBulletVehicle : public palVehicle , public btActionInterface {
public:
	palBulletVehicle();
	virtual ~palBulletVehicle();
	virtual void Init(palBody *chassis, Float MotorForce, Float BrakeForce);

	virtual palWheel* AddWheel();
	virtual void Finalize();

	virtual void Control(Float steering, Float acceleration, bool brakes);
	virtual void Update() {
		//do nothing...
	}

	//bullet code:
	virtual void ForceControl(Float steering, Float accelerationforce, Float brakeforce);

	virtual Float GetEngineForce() const { return m_cEngineForce; };
	virtual Float GetBreakingForce() const { return m_cBreakingForce; };
	virtual Float GetVehicleSteering() const { return m_cVehicleSteering; };

	virtual btRaycastVehicle::btVehicleTuning& BulletGetTuning();

	virtual void updateAction( btCollisionWorld* collisionWorld, btScalar deltaTimeStep);

   virtual void debugDraw(btIDebugDraw* debugDrawer)
   {
      m_vehicle->debugDraw(debugDrawer);
   }

   virtual unsigned GetSubStepCount() const { return m_iSubstepCount; }
   virtual void SetSubStepCount(unsigned steps) { m_iSubstepCount = steps; }

private:
	Float	m_cEngineForce;
	Float	m_cBreakingForce;
	Float	m_cVehicleSteering;

	btRigidBody* m_carChassis;
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btDynamicsWorld* m_dynamicsWorld;

	unsigned m_iSubstepCount;
   btVector3 m_vGravity;

	FACTORY_CLASS(palBulletVehicle,palVehicle,Bullet,1)
};

#endif
