#include "bullet_palVehicle.h"


//#define SWAPXZ

palBulletVehicle::palBulletVehicle()
: m_carChassis(NULL)
, m_vehicleRayCaster(NULL)
, m_vehicle(NULL)
, m_dynamicsWorld(NULL)
, m_iSubstepCount(1U)
, m_vGravity(0.0, 0.0, 0.0)
{
}

palBulletVehicle::~palBulletVehicle() {

	if (m_dynamicsWorld != NULL)
	{
		m_dynamicsWorld->removeVehicle(this);
	}

	delete m_vehicleRayCaster;
	m_vehicleRayCaster = NULL;
	delete m_vehicle;
	m_vehicle = NULL;
}


void palBulletVehicle::Init(palBody *chassis, Float MotorForce, Float BrakeForce) {
	palVehicle::Init(chassis,MotorForce,BrakeForce);
	m_cEngineForce = 0;
	m_cBreakingForce = 0;
	m_cVehicleSteering = 0;

	palBulletPhysics *pbp = static_cast<palBulletPhysics*>(GetParent());
	m_dynamicsWorld = pbp->BulletGetDynamicsWorld();
	palBulletBody *pbb = dynamic_cast<palBulletBody *>(chassis);
	m_carChassis = pbb->BulletGetRigidBody();
#if 1
	btTransform tr;
	btQuaternion q;
	q.setRotation(btVector3(0,0,1),btScalar(M_PI*0.5f));
	tr.setIdentity();
	tr.setRotation(q);
	//		m_carChassis->setCenterOfMassTransform(tr);
	//		m_carChassis->setWorldTransform(tr);
#endif

	m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
	m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);

	///never deactivate the vehicle
	m_carChassis->setActivationState(DISABLE_DEACTIVATION);

	m_dynamicsWorld->addVehicle(this);

	unsigned int upAxis = static_cast<palPhysics*>(GetParent())->GetUpAxis();

	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;

	if (upAxis == 2) {
		rightIndex = 0;
		upIndex = 2;
		forwardIndex = 1;
	} else if (upAxis == 0) {
		rightIndex = 1;
		upIndex = 0;
		forwardIndex = 2;
	}

	//ie: x is z, y is y, z is x
	//choose coordinate system
	m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);


}


void palBulletVehicle::Finalize() {
	float mass = m_pbChassis->m_fMass;

	//float	wheelFriction = 1000;//1e30f;
	for (int i=0; i<m_vehicle->getNumWheels(); i++) {
		btWheelInfo& wheel = m_vehicle->getWheelInfo(i);

		// Divide these values by the mass because they get multipled by the mass internally.
		// I have no idea why.

		wheel.m_suspensionStiffness = m_vWheels[i]->m_WheelInfo.m_fSuspension_Ks / mass;
		wheel.m_wheelsDampingRelaxation = (m_vWheels[i]->m_WheelInfo.m_fSuspension_Kd / mass) * 1.5;
		wheel.m_wheelsDampingCompression = (m_vWheels[i]->m_WheelInfo.m_fSuspension_Kd / mass) * 0.666667;
		//wheel.m_frictionSlip = wheelFriction;
		wheel.m_rollInfluence = m_vWheels[i]->m_WheelInfo.m_fRoll_Influence;
		// Make the max force equal to 3 times approximate curb load for each wheel.
		wheel.m_maxSuspensionForce = (3.0f/float(m_vehicle->getNumWheels())) * mass *
					dynamic_cast<palBulletBody*>(m_pbChassis)->BulletGetRigidBody()->getGravity().length();
		// Max force equals the force of the suspension at the maximum compression, i.e. at the bump stop.
		wheel.m_maxSuspensionForce = m_vWheels[i]->m_WheelInfo.m_fSuspension_Ks
					* m_vWheels[i]->m_WheelInfo.m_fSuspension_Rest_Length;

		((palBulletWheel*)m_vWheels[i])->m_WheelIndex = i;
	}
}

void palBulletVehicle::ForceControl(Float steering, Float acceleration, Float brakes) {
	m_cEngineForce = acceleration;
	m_cBreakingForce = brakes;
	m_cVehicleSteering = steering;
	//for (int i=0;i<m_vehicle->getNumWheels();i++) {
	for (PAL_VECTOR<palWheel *>::size_type i=0;i<m_vWheels.size();i++) {
		if (m_vWheels[i]->m_WheelInfo.m_bDrive )
			m_vehicle->applyEngineForce(m_cEngineForce,(int)i);
		if (m_vWheels[i]->m_WheelInfo.m_bBrake )
			m_vehicle->setBrake(m_cBreakingForce,(int)i);
		if (m_vWheels[i]->m_WheelInfo.m_bSteer )
			m_vehicle->setSteeringValue(m_cVehicleSteering,(int)i);
	}
}

void palBulletVehicle::Control(Float steering, Float acceleration, bool brakes) {
	if (brakes)
		m_cBreakingForce = m_fBrakeForce;
	m_cEngineForce = acceleration * m_fMotorForce;
	m_cVehicleSteering = steering*0.3f;
	ForceControl(m_cVehicleSteering,m_cEngineForce,m_cBreakingForce);
}


palWheel* palBulletVehicle::AddWheel() {
	palBulletWheel *pbw = new palBulletWheel;
	pbw->m_pVehicle =  this;
	pbw->m_vehicle = m_vehicle;
	m_vWheels.push_back(pbw);
	return pbw;
}

btRaycastVehicle::btVehicleTuning& palBulletVehicle::BulletGetTuning()
{
   return m_tuning;
}

void palBulletVehicle::updateAction( btCollisionWorld* collisionWorld, btScalar deltaTimeStep)
{
	btVector3 gravity = m_carChassis->getGravity();
	int upAxis = m_vehicle->getUpAxis();
	// Gravity is active on the m_carChassis, so set it to really SIMD_EPSILON so it's not zero, but won't
	// do anything and remember the gravity
	if (gravity[upAxis] < -SIMD_EPSILON)
	{
		btVector3 smallGravity(0.0, 0.0, 0.0);
		smallGravity[upAxis] = -SIMD_EPSILON;
		m_carChassis->setGravity(smallGravity);
		m_vGravity = gravity;
	}

	// gravity is zero on the m_carChassis, so it's disabled, so clear the saved gravity
	if (gravity[upAxis] == btScalar(0.0))
	{
		m_vGravity = gravity;
	}

	for (int i=0; i<m_vehicle->getNumWheels(); i++) {

		float speed2 = m_carChassis->getLinearVelocity().length2() - 0.5f;
		if (speed2 > 1.0f)
		{
			speed2 = 1.0f;
		}
		else if (speed2 < 0.0f)
		{
			speed2 = 0.0f;
		}

		// scale the roll influence back up once the speed goes below 1.
		// the roll influence should be 1.0 at 0 speed and the configured value at a speed of 1.
		//btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
		//wheel.m_rollInfluence = 1.0f - ((speed2) * (1.0f - m_vWheels[i]->m_WheelInfo.m_fRoll_Influence));
		//printf("palVehicle roll influence per wheel %d %f\n", i, wheel.m_rollInfluence);
	}


	btScalar timestep = deltaTimeStep / btScalar(m_iSubstepCount);

	btVector3 gravityImpulse = m_vGravity * timestep / m_carChassis->getInvMass() ;
	for (unsigned i = 0; i < m_iSubstepCount; ++i)
	{
		m_carChassis->applyCentralImpulse(gravityImpulse);
		m_vehicle->updateAction(collisionWorld, timestep);
	}
}


palBulletWheel::palBulletWheel() {
	m_WheelIndex = -1;
};

void palBulletWheel::Init(const palWheelInfo& wheelInfo) {
	palWheel::Init(wheelInfo);


	unsigned int upAxis = PF->GetActivePhysics()->GetUpAxis();

	btVector3 wheelDirectionCS0(0.0f, 0.0f, 0.0f);
	wheelDirectionCS0[upAxis] = -1.0f;

	btVector3 wheelAxleCS(0.0f, 0.0f, 1.0f);

	if (upAxis == 2) {
	   wheelAxleCS.setValue(1.0f, 0.0f, 0.0f);
	} else if (upAxis == 0) {
		wheelAxleCS.setValue(0.0f, 1.0f, 0.0f);
	}

	btVector3 connectionPointCS0(wheelInfo.m_fPosX, wheelInfo.m_fPosY, wheelInfo.m_fPosZ);

	palBulletVehicle *pbv = dynamic_cast<palBulletVehicle *>(m_pVehicle);
	btRaycastVehicle::btVehicleTuning tuning = pbv->BulletGetTuning();
	tuning.m_frictionSlip = wheelInfo.m_fFriction_Slip;
	tuning.m_maxSuspensionTravelCm = wheelInfo.m_fSuspension_Travel * 100.0f; // convert to centimeters
	m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,wheelInfo.m_fSuspension_Rest_Length,wheelInfo.m_fRadius,tuning,wheelInfo.m_bSteer);
}
palMatrix4x4& palBulletWheel::GetLocationMatrix() {
	if (m_WheelIndex<0)
		return m_mLoc;
	m_vehicle->updateWheelTransform(m_WheelIndex,true);
	convertBtTransformToPalMat(m_mLoc, m_vehicle->getWheelInfo(m_WheelIndex).m_worldTransform);
	return m_mLoc;
}


