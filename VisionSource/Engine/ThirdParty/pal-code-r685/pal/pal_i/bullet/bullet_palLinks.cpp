#include "bullet_palLinks.h"

////////////////////////////////////////////////////
static bool SetAnyBulletParam(btTypedConstraint* constraint, int parameter, btScalar value, int axis)
{
	bool result = constraint != NULL;
	if (result)
	{
		btConstraintParams param = btConstraintParams(parameter);
		switch (parameter)
		{
		case PAL_LINK_PARAM_STOP_ERP:
			param = BT_CONSTRAINT_STOP_ERP;
			constraint->setParam(param, value, axis);
			break;
		case PAL_LINK_PARAM_CFM:
			param = BT_CONSTRAINT_CFM;
			constraint->setParam(param, value, axis);
			break;
		case PAL_LINK_PARAM_STOP_CFM:
			param = BT_CONSTRAINT_STOP_CFM;
			constraint->setParam(param, value, axis);
			break;
		case PAL_LINK_PARAM_BREAK_IMPULSE:
			constraint->setBreakingImpulseThreshold(value);
			break;
			// nothing in bullet currently supports this, and it asserts out if you pass it.
		case PAL_LINK_PARAM_ERP:
		default:
			result = false;
		}
	}
	return result;
}

////////////////////////////////////////////////////
static btScalar GetAnyBulletParam(btTypedConstraint* constraint, int parameter, int axis)
{
	btScalar result = -1.0f;
	if (constraint != NULL)
	{
		btConstraintParams param = btConstraintParams(parameter);
		switch (parameter)
		{
		case PAL_LINK_PARAM_STOP_ERP:
			param = BT_CONSTRAINT_STOP_ERP;
			result = constraint->getParam(param, axis);
			break;
		case PAL_LINK_PARAM_CFM:
			param = BT_CONSTRAINT_CFM;
			result = constraint->getParam(param, axis);
			break;
		case PAL_LINK_PARAM_STOP_CFM:
			param = BT_CONSTRAINT_STOP_CFM;
			result = constraint->getParam(param, axis);
			break;
		case PAL_LINK_PARAM_BREAK_IMPULSE:
			result = constraint->getBreakingImpulseThreshold();
			break;
			// nothing in bullet currently supports this, and it asserts out if you pass it.
		case PAL_LINK_PARAM_ERP:
		default:
			result = false;
		}
	}
	return result;
}


palBulletSphericalLink::palBulletSphericalLink()
: m_btConeTwist(0) {}

palBulletSphericalLink::~palBulletSphericalLink() {
	if (m_btConeTwist) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btConeTwist);
		delete m_btConeTwist;
		m_btConeTwist = 0;
	}
}

void palBulletSphericalLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	palBulletBodyBase *parentBodyBase = dynamic_cast<palBulletBodyBase *> (parent);
	palBulletBodyBase *childBodyBase = dynamic_cast<palBulletBodyBase *> (child);
	btRigidBody* parentBulletBody = parentBodyBase->BulletGetRigidBody();
	btRigidBody* childBulletBody = childBodyBase->BulletGetRigidBody();

	btTransform frameA, frameB;

	convertPalMatToBtTransform(frameA, parentFrame);
	convertPalMatToBtTransform(frameB, childFrame);

	btConeTwistConstraint* coneTwist = new btConeTwistConstraint(*parentBulletBody, *childBulletBody,
			frameA, frameB);

	m_btConeTwist = coneTwist;
	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btConeTwist, disableCollisionsBetweenLinkedBodies);
}

void palBulletSphericalLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(1.0, 0.0, 0.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}

bool palBulletSphericalLink::SetParam(int parameterCode, Float value, int axis) {

	bool result = false;
	if (axis >= 0)
	{
		switch (parameterCode)
		{
		case PAL_LINK_PARAM_DOF_MIN:
			// Minimums mean nothing in this joint because the max accounts for both positive and negative.
			result = false;
			break;
		case PAL_LINK_PARAM_DOF_MAX:
			if (axis >= 3 && axis < 6)
			{
				m_btConeTwist->setLimit(axis, btFabs(value));
				result = true;
			}
			break;
		default:
			result = SetAnyBulletParam(m_btConeTwist, parameterCode, btScalar(value), axis);
		}
	}
	else if (axis < 0)
	{
		result = true;
		// This joint just splits between linear and rotational, so this just sets it once for each.
		result = result && SetAnyBulletParam(m_btConeTwist, parameterCode, btScalar(value), 0);
		result = result && SetAnyBulletParam(m_btConeTwist, parameterCode, btScalar(value), 3);
	}
	return result;
}

Float palBulletSphericalLink::GetParam(int parameterCode, int axis) {
	if (axis < 0)
		axis = 0;
	float result = 0.0;
	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
		result = 0.0f;
		break;
	case PAL_LINK_PARAM_DOF_MAX:
		if (axis == 3)
		{
			result = m_btConeTwist->getTwistSpan();
		}
		else if (axis == 4)
		{
			result = m_btConeTwist->getSwingSpan2();
		}
		else if (axis == 5)
		{
			result = m_btConeTwist->getSwingSpan1();
		}
		break;
	default:
		result = GetAnyBulletParam(m_btConeTwist, parameterCode, axis);
	}
	return result;
}

bool palBulletSphericalLink::SupportsParameters() const {
	return true;
}

bool palBulletSphericalLink::SupportsParametersPerAxis() const {
	return true;
}

/**
 * The Bullet class btHingeConstraint uses the btAdjustAngleToLimits function
 * (from btTypedConstraint.h), which has a bug
 * (http://code.google.com/p/bullet/issues/detail?id=377). We can't
 * replace btAdjustAngleToLimits, so we'll subclass btHingeConstraint,
 * instead.
 *
 * The following method and the palHingeConstraint class are based on
 * code from Bullet. These may be removed an btHingeConstraint used
 * again if/when this bug is fixed.  See the end of this file for the
 * Bullet license.
 *
 */

btScalar adjustAngleToLimits(btScalar angleInRadians, btScalar angleLowerLimitInRadians, btScalar angleUpperLimitInRadians) {
	if(angleLowerLimitInRadians >= angleUpperLimitInRadians) {
		return angleInRadians;
	}
	else if(angleInRadians < angleLowerLimitInRadians) {
		btScalar diffLo = btFabs(btNormalizeAngle(angleLowerLimitInRadians - angleInRadians));
		btScalar diffHi = btFabs(btNormalizeAngle(angleUpperLimitInRadians - angleInRadians));
		return (diffLo < diffHi) ? angleInRadians : (angleInRadians + SIMD_2_PI);
	}
	else if (angleInRadians > angleUpperLimitInRadians) {
		btScalar diffHi = btFabs(btNormalizeAngle(angleInRadians - angleUpperLimitInRadians));
		btScalar diffLo = btFabs(btNormalizeAngle(angleInRadians - angleLowerLimitInRadians));
		return (diffLo < diffHi) ? (angleInRadians - SIMD_2_PI) : angleInRadians;
	}
	else {
		return angleInRadians;
	}
}


palBulletRevoluteLink::palBulletRevoluteLink()
: palLink(PAL_LINK_REVOLUTE), m_btHinge(0), m_feedback(0) {}

palBulletRevoluteLink::~palBulletRevoluteLink() {
	if (m_btHinge) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btHinge);
		delete m_btHinge;
		m_btHinge = NULL;
	}
}

void palBulletRevoluteLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (m_btHinge) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btHinge);
		delete m_btHinge;
		m_btHinge = NULL;
	}
	const palBulletBodyBase *parentBodyBase = dynamic_cast<palBulletBodyBase *> (parent);
	const palBulletBodyBase *childBodyBase = dynamic_cast<palBulletBodyBase *> (child);
	btRigidBody *parentBulletBody = parentBodyBase->BulletGetRigidBody();
	btRigidBody *childBulletBody = childBodyBase->BulletGetRigidBody();

	btTransform frameA, frameB;

	convertPalMatToBtTransform(frameA, parentFrame);
	convertPalMatToBtTransform(frameB, childFrame);

	// DEBUG
	//std::cout << "pal frame A: " << m_frameA << "\tpal frame B: " << m_frameB << std::endl;
	//std::cout << "bullet frame A: " << frameA << "\tbullet frame B: " << frameB << std::endl;
	m_btHinge = new btHingeConstraint(*parentBulletBody, *childBulletBody, frameA, frameB, false);
	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btHinge, disableCollisionsBetweenLinkedBodies);
}

void palBulletRevoluteLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (m_btHinge) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btHinge);
		delete m_btHinge;
		m_btHinge = NULL;
	}
	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(0.0, 0.0, 1.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}


void palBulletRevoluteLink::GetPosition(palVector3& pos) const {
	//Get the pivot in the frame A and transform it to global coordinates
	palBulletBodyBase *body0 = dynamic_cast<palBulletBodyBase *> (GetParentBody());
	btTransform pivotInGlobal = (body0->BulletGetRigidBody())->getCenterOfMassTransform() * m_btHinge->getAFrame();

	pos.x = pivotInGlobal.getOrigin().x();
	pos.y = pivotInGlobal.getOrigin().y();
	pos.z = pivotInGlobal.getOrigin().z();
}

Float palBulletRevoluteLink::GetAngle() const {
	return m_btHinge->getHingeAngle();
}

palLinkFeedback* palBulletRevoluteLink::GetFeedback() const
throw(palIllegalStateException) {
	if (!m_btHinge) {
		throw palIllegalStateException("Init must be called first");
	}
	if (!m_feedback) {
		const_cast<palBulletRevoluteLink*>(this)->m_feedback = new bulletRevoluteLinkFeedback(m_btHinge);
	}
	return m_feedback;
}

bool palBulletRevoluteLink::SetParam(int parameterCode, Float value, int axis) {
	bool result = false;
	if ((axis == 5 || axis == -1))
	{
		switch (parameterCode)
		{
		case PAL_LINK_PARAM_DOF_MIN:
			m_btHinge->setLimit(btScalar(value), m_btHinge->getUpperLimit());
			result = true;
			break;
		case PAL_LINK_PARAM_DOF_MAX:
			m_btHinge->setLimit(m_btHinge->getLowerLimit(), btScalar(value));
			result = true;
			break;
		default:
			result = SetAnyBulletParam(m_btHinge, parameterCode, btScalar(value), axis);
		}
	}
	return result;
}

Float palBulletRevoluteLink::GetParam(int parameterCode, int axis) {
	Float result = -1.0f;
	if ((axis == 5 || axis == -1))
	{
		switch (parameterCode)
		{
		case PAL_LINK_PARAM_DOF_MIN:
			result = m_btHinge->getLowerLimit();
			break;
		case PAL_LINK_PARAM_DOF_MAX:
			result = m_btHinge->getUpperLimit();
			break;
		case PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE:
			result = Float(m_btHinge->getHingeAngle());
			break;
		default:
			result = GetAnyBulletParam(m_btHinge, parameterCode, axis);
		}
	}
	return result;
}

bool palBulletRevoluteLink::SupportsParameters() const {
	return true;
}

bool palBulletRevoluteLink::SupportsParametersPerAxis() const {
	// This joint only supports the z axis only, so it's best to return false.
	return false;
}


bulletRevoluteLinkFeedback::bulletRevoluteLinkFeedback(btHingeConstraint *hinge)
: m_btHinge(hinge)
{
}

bool bulletRevoluteLinkFeedback::IsEnabled() const {
	return m_btHinge->needsFeedback();
}

Float bulletRevoluteLinkFeedback::GetValue() const {
	return m_btHinge->getAppliedImpulse();
}

bool bulletRevoluteLinkFeedback::SetEnabled(bool enable) {
	m_btHinge->enableFeedback(enable);
	return IsEnabled();
}

////////////////////////////////////////////////////////


palBulletRevoluteSpringLink::palBulletRevoluteSpringLink()
: m_bt6Dof(0)
{
}

palBulletRevoluteSpringLink::~palBulletRevoluteSpringLink() {
	if (m_bt6Dof) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_bt6Dof);
		delete m_bt6Dof;
		m_bt6Dof = NULL;
	}
}

void palBulletRevoluteSpringLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (m_bt6Dof) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_bt6Dof);
		delete m_bt6Dof;
		m_bt6Dof = NULL;
	}
	const palBulletBodyBase *parentBodyBase = dynamic_cast<palBulletBodyBase *> (parent);
	const palBulletBodyBase *childBodyBase = dynamic_cast<palBulletBodyBase *> (child);
	btRigidBody *parentBulletBody = parentBodyBase->BulletGetRigidBody();
	btRigidBody *childBulletBody = childBodyBase->BulletGetRigidBody();

	btTransform frameA, frameB;

	convertPalMatToBtTransform(frameA, parentFrame);
	convertPalMatToBtTransform(frameB, childFrame);

#if BT_BULLET_VERSION < 283
	m_bt6Dof = new SubbtGeneric6DofSpringConstraint(*parentBulletBody,
			*childBulletBody,
			frameA,
			frameB,
			true);
#else
	m_bt6Dof = new btGeneric6DofSpring2Constraint(*parentBulletBody,
			*childBulletBody,
			frameA,
			frameB,
			RO_ZXY
			);
#endif

	// Set the lower limit higher that the upper limit by default.  This means free movement.
	m_bt6Dof->setAngularLowerLimit(btVector3(0.0f, 0.0f, SIMD_PI + 0.1f));
	m_bt6Dof->setAngularUpperLimit(btVector3(0.0f, 0.0f, SIMD_PI));

	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_bt6Dof, disableCollisionsBetweenLinkedBodies);
}

void palBulletRevoluteSpringLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (m_bt6Dof) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_bt6Dof);
		delete m_bt6Dof;
		m_bt6Dof = NULL;
	}
	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(0.0, 0.0, 1.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}

void palBulletRevoluteSpringLink::SetSpring(const palSpringDesc& springDesc) {
	bool enable = springDesc.m_fSpringCoef > SIMD_EPSILON || springDesc.m_fDamper > SIMD_EPSILON;
	m_bt6Dof->enableSpring(5, enable);
#if BT_BULLET_VERSION > 282
	m_bt6Dof->setStiffness(5, springDesc.m_fSpringCoef, false);
	m_bt6Dof->setDamping(5, springDesc.m_fDamper, false);
#else
	m_bt6Dof->setStiffness(5, springDesc.m_fSpringCoef);
	m_bt6Dof->setDamping(5, springDesc.m_fDamper);
#endif
	m_bt6Dof->setEquilibriumPoint(5, springDesc.m_fTarget);
#if BT_BULLET_VERSION > 282
	//m_bt6Dof->setBounce(5, enable ? btScalar(1.0): btScalar(0.0));
#endif
	//m_bt6Dof->getRotationalLimitMotor(2)->m_bounce = btScalar(0.3);
}

void palBulletRevoluteSpringLink::GetSpring(palSpringDesc& springDescOut) const {
#if BT_BULLET_VERSION < 283
	m_bt6Dof->getSpringDesc(5, springDescOut);
#else
	btRotationalLimitMotor2* motor = m_bt6Dof->getRotationalLimitMotor(2);
	springDescOut.m_fDamper = motor->m_springDamping;
	springDescOut.m_fSpringCoef = motor->m_springStiffness;
	springDescOut.m_fTarget = motor->m_equilibriumPoint;
#endif
}

bool palBulletRevoluteSpringLink::SetParam(int parameterCode, Float value, int axis) {
	// Check to prevent an assertion crash failure.
	if (axis != -1)
		return false;
	bool result = true;

	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
	{
		btVector3 upperLimits;
		m_bt6Dof->getAngularUpperLimit(upperLimits);
		m_bt6Dof->setLimit(5, btScalar(value), upperLimits.getZ());
		result = true;
		break;
	}
	case PAL_LINK_PARAM_DOF_MAX:
	{
		btVector3 lowerLimits;
		m_bt6Dof->getAngularLowerLimit(lowerLimits);
		m_bt6Dof->setLimit(5, lowerLimits.getZ(), btScalar(value));
		result = true;
		break;
	}
	default:
		for (unsigned axisI = 0; result && axisI < 5; ++axisI)
		{
			result = result && SetAnyBulletParam(m_bt6Dof, parameterCode, btScalar(value), axisI);
		}
	}


	return result;
}

Float palBulletRevoluteSpringLink::GetParam(int parameterCode, int axis) {
	if (axis != -1)
		return -1.0f;
	Float result = 0.0f;
	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
	{
		btVector3 lowerLimits;
		m_bt6Dof->getAngularLowerLimit(lowerLimits);
		result = lowerLimits.getZ();
		break;
	}
	case PAL_LINK_PARAM_DOF_MAX:
	{
		btVector3 upperLimits;
		m_bt6Dof->getAngularUpperLimit(upperLimits);
		result = upperLimits.getZ();
		break;
	}
	case PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE:
	{
		result = Float(m_bt6Dof->getAngle(2));
		break;
	}
	default:
		result = GetAnyBulletParam(m_bt6Dof, parameterCode, 5);
	}
	return result;
}

bool palBulletRevoluteSpringLink::SupportsParameters() const {
	return true;
}

bool palBulletRevoluteSpringLink::SupportsParametersPerAxis() const {
	// This joint only supports the z axis only, so it's best to return false.
	return false;
}


////////////////////////////////////////////////////////

palBulletPrismaticLink::palBulletPrismaticLink()
: palLink(PAL_LINK_PRISMATIC), m_btSlider(0) {}

palBulletPrismaticLink::~palBulletPrismaticLink()
{
	if (m_btSlider) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btSlider);
		delete m_btSlider;
		m_btSlider = NULL;
	}
}

void palBulletPrismaticLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (m_btSlider) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btSlider);
		delete m_btSlider;
		m_btSlider = NULL;
	}

	const palBulletBodyBase *parentBodyBase = dynamic_cast<palBulletBodyBase *> (parent);
	const palBulletBodyBase *childBodyBase = dynamic_cast<palBulletBodyBase *> (child);
	btRigidBody *parentBulletBody = parentBodyBase->BulletGetRigidBody();
	btRigidBody *childBulletBody = childBodyBase->BulletGetRigidBody();

	btTransform frameA, frameB;

	convertPalMatToBtTransform(frameA, parentFrame);
	convertPalMatToBtTransform(frameB, childFrame);

	m_btSlider = new btSliderConstraint(*parentBulletBody, *childBulletBody, frameA, frameB, true);

	//Constraint the angular movement
	m_btSlider->setLowerAngLimit(0.0f);
	m_btSlider->setUpperAngLimit(0.0f);

	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btSlider, disableCollisionsBetweenLinkedBodies);
}

void palBulletPrismaticLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);

	if (m_btSlider) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btSlider);
		delete m_btSlider;
		m_btSlider = NULL;
	}

	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(1.0, 0.0, 0.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}

bool palBulletPrismaticLink::SetParam(int parameterCode, Float value, int axis) {
	bool result = false;
	// Check to prevent an assertion crash failure.
	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
		if (axis == 0)
		{
			m_btSlider->setLowerLinLimit(btScalar(value));
			result = true;
		}
		else if (axis == 3)
		{
			m_btSlider->setLowerAngLimit(btScalar(value));
			result = true;
		}
		break;
	case PAL_LINK_PARAM_DOF_MAX:
		if (axis == 0)
		{
			m_btSlider->setUpperLinLimit(btScalar(value));
			result = true;
		}
		else if (axis == 3)
		{
			m_btSlider->setUpperAngLimit(btScalar(value));
			result = true;
		}
		break;
	default:
		if (axis < 0 || axis == 3)
			result = SetAnyBulletParam(m_btSlider, parameterCode, btScalar(value), axis);
	}
	return result;
}

Float palBulletPrismaticLink::GetParam(int parameterCode, int axis) {
	Float result = 0.0f;
	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
		if (axis == 0)
		{
			result = Float(m_btSlider->getLowerLinLimit());
		}
		else if (axis == 3)
		{
			result = Float(m_btSlider->getLowerAngLimit());
		}
		break;
	case PAL_LINK_PARAM_DOF_MAX:
		if (axis == 0)
		{
			result = Float(m_btSlider->getUpperLinLimit());
		}
		else if (axis == 3)
		{
			result = Float(m_btSlider->getUpperAngLimit());
		}
		break;
	case PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE:
		if (axis == 0)
		{
			result = Float(m_btSlider->getLinearPos());
		}
		else if (axis == 3)
		{
			result = Float(m_btSlider->getAngularPos());
		}
		break;
	default:
		if (axis < 0 || axis == 3)
			result = GetAnyBulletParam(m_btSlider, parameterCode, axis);
	}
	return result;
}

bool palBulletPrismaticLink::SupportsParameters() const {
	return true;
}

bool palBulletPrismaticLink::SupportsParametersPerAxis() const {
	return true;
}


//////////////////////////////
palBulletGenericLink::palBulletGenericLink()
: genericConstraint(0) {}

palBulletGenericLink::~palBulletGenericLink() {
	if (genericConstraint) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(genericConstraint);
		delete genericConstraint;
		genericConstraint = NULL;
	}
}

void palBulletGenericLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame,
		const palMatrix4x4& childFrame,
		bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);
	if (genericConstraint) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(genericConstraint);
		delete genericConstraint;
		genericConstraint = NULL;
	}

	palBulletBodyBase *body0 = dynamic_cast<palBulletBodyBase *> (parent);
	palBulletBodyBase *body1 = dynamic_cast<palBulletBodyBase *> (child);

	btTransform frameInA, frameInB;

	convertPalMatToBtTransform(frameInA, parentFrame);
	convertPalMatToBtTransform(frameInB, childFrame);

#if BT_BULLET_VERSION < 283
	genericConstraint = new SubbtGeneric6DofSpringConstraint(
			*(body0->BulletGetRigidBody()),*(body1->BulletGetRigidBody()),
			frameInA,frameInB,false);
#else
	genericConstraint = new btGeneric6DofSpring2Constraint(
			*(body0->BulletGetRigidBody()),*(body1->BulletGetRigidBody()),
			frameInA,frameInB, RO_ZXY);
#endif
	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(genericConstraint, disableCollisionsBetweenLinkedBodies);
}

void palBulletGenericLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);

	if (genericConstraint) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(genericConstraint);
		delete genericConstraint;
		genericConstraint = NULL;
	}

	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(0.0, 0.0, 1.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}


bool palBulletGenericLink::SetParam(int parameterCode, Float value, int axis) {

	bool result = false;
	if (axis >= 0)
	{
		btVector3 limits;
		switch (parameterCode)
		{
		case PAL_LINK_PARAM_DOF_MIN:
			if (axis < 3)
			{
				genericConstraint->getLinearLowerLimit(limits);
				limits.m_floats[axis] = btScalar(value);
				genericConstraint->setLinearLowerLimit(limits);
			}
			else
			{
				axis -= 3U;
				genericConstraint->getAngularLowerLimit(limits);
				limits.m_floats[axis] = btScalar(value);
				genericConstraint->setAngularLowerLimit(limits);
			}
			result = true;
			break;
		case PAL_LINK_PARAM_DOF_MAX:
			if (axis < 3)
			{
				genericConstraint->getLinearUpperLimit(limits);
				limits.m_floats[axis] = btScalar(value);
				genericConstraint->setLinearUpperLimit(limits);
			}
			else
			{
				axis -= 3U;
				genericConstraint->getAngularUpperLimit(limits);
				limits.m_floats[axis] = btScalar(value);
				genericConstraint->setAngularUpperLimit(limits);
			}
			result = true;
			break;
		default:
			result = SetAnyBulletParam(genericConstraint, parameterCode, btScalar(value), axis);
		}
	}
	else if (axis < 0)
	{
		result = true;
		for (unsigned axisI = 0; axisI < 6; ++axisI)
		{
			result = result && SetAnyBulletParam(genericConstraint, parameterCode, btScalar(value), axisI);
		}
	}
	return result;
}

Float palBulletGenericLink::GetParam(int parameterCode, int axis) {
	if (axis < 0)
		axis = 0;
	float result = 0.0;
	btVector3 limits;
	switch (parameterCode)
	{
	case PAL_LINK_PARAM_DOF_MIN:
		if (axis < 3)
		{
			genericConstraint->getLinearLowerLimit(limits);
		}
		else
		{
			axis -= 3U;
			genericConstraint->getAngularLowerLimit(limits);
		}
		result = limits.m_floats[axis];
		break;
	case PAL_LINK_PARAM_DOF_MAX:
		if (axis < 3)
		{
			genericConstraint->getLinearUpperLimit(limits);
		}
		else
		{
			axis -= 3U;
			genericConstraint->getAngularUpperLimit(limits);
		}
		result = limits.m_floats[axis];
		break;
	case PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE:
		if (axis < 3)
		{
			result = Float(genericConstraint->getRelativePivotPosition(axis));
		}
		else if (axis < 6)
		{
			result = Float(genericConstraint->getAngle(axis-3));
		}
		break;
	default:
		result = GetAnyBulletParam(genericConstraint, parameterCode, axis);
	}
	return result;
}

bool palBulletGenericLink::SupportsParameters() const {
	return true;
}

bool palBulletGenericLink::SupportsParametersPerAxis() const {
	return true;
}


palBulletRigidLink::palBulletRigidLink()
:  palLink(PAL_LINK_RIGID), palRigidLink(), m_btFixed(), m_disableCollisionsBetweenLinkedBodies()
{
}

palBulletRigidLink::~palBulletRigidLink()
{
	if (m_btFixed) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btFixed);
		delete m_btFixed;
		m_btFixed = NULL;
	}
}

void palBulletRigidLink::Init(palBodyBase *parent, palBodyBase *child,
		const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies)
{
	SetBodies(parent, child);

	const palBulletBodyBase *parentBodyBase = dynamic_cast<palBulletBodyBase *> (parent);
	const palBulletBodyBase *childBodyBase = dynamic_cast<palBulletBodyBase *> (child);
	btRigidBody* parentBulletBody = parentBodyBase->BulletGetRigidBody();
	btRigidBody* childBulletBody = childBodyBase->BulletGetRigidBody();

	btTransform frameInA, frameInB;

	convertPalMatToBtTransform(frameInA, parentFrame);
	convertPalMatToBtTransform(frameInB, childFrame);

#if BT_BULLET_VERSION > 282
	//reinit. It doesn't allow changing the bodies, so if those change, it all must be deleted.
	if (m_btFixed && (&m_btFixed->getRigidBodyA() != parentBulletBody || &m_btFixed->getRigidBodyB() != childBulletBody)) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btFixed);
		delete m_btFixed;
		m_btFixed = NULL;
	}

	if (m_btFixed) {
		// Frames can be reset, but if disableCollisionsBetweenLinkedBodies changes, it has to be removed from the engine
		// and readded.
		if (m_disableCollisionsBetweenLinkedBodies != disableCollisionsBetweenLinkedBodies) {
			static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btFixed);
		}
		m_btFixed->setFrames(frameInA, frameInB);
		m_btFixed->setEnabled(true);
		if (m_disableCollisionsBetweenLinkedBodies != disableCollisionsBetweenLinkedBodies) {
			static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btFixed, disableCollisionsBetweenLinkedBodies);
		}
		m_disableCollisionsBetweenLinkedBodies = disableCollisionsBetweenLinkedBodies;
	} else {
		m_btFixed = new btFixedConstraint(*parentBulletBody, *childBulletBody, frameInA, frameInB);
		static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btFixed, disableCollisionsBetweenLinkedBodies);
	}

#else
	if (m_btFixed) {
		static_cast<palBulletPhysics*>(GetParent())->RemoveBulletConstraint(m_btFixed);
		delete m_btFixed;
		m_btFixed = NULL;
	}
	// No fixed joint exists before 2.82, so just use a locked hinge.
	m_btFixed = new btHingeConstraint(*parentBulletBody, *childBulletBody, frameInA, frameInB, false);
	const btScalar epsilon = SIMD_EPSILON;
	// Lock the joint
	m_btFixed->setLimit(epsilon, epsilon);

	static_cast<palBulletPhysics*>(GetParent())->AddBulletConstraint(m_btFixed, disableCollisionsBetweenLinkedBodies);
#endif
}

void palBulletRigidLink::Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies)
{
	palMatrix4x4 frameA, frameB;
	palVector3 defaultAxis(0.0, 0.0, 1.0);
	ComputeFramesFromPivot(frameA, frameB, pos, axis, defaultAxis);

	Init(parent, child, frameA, frameB, disableCollisionsBetweenLinkedBodies);
}

std::ostream& operator<<(std::ostream &os, const palBulletRigidLink& link)
{
	const palLink& superLink = *(static_cast<const palLink*>(&link));
	os << superLink;
	return os;
}
bool palBulletRigidLink::SetParam(int parameterCode, Float value, int axis) {
	if (axis >= 0)
		return false;
	return SetAnyBulletParam(m_btFixed, parameterCode, btScalar(value), axis);
}

Float palBulletRigidLink::GetParam(int parameterCode, int axis) {
	if (axis >= 0)
		return Float(0.0);
	return GetAnyBulletParam(m_btFixed, parameterCode, axis);
}

bool palBulletRigidLink::SupportsParameters() const {
	return true;
}

bool palBulletRigidLink::SupportsParametersPerAxis() const {
	return false;
}

//////////////////////////////////////////////////////////

palBulletGenericLinkSpring::palBulletGenericLinkSpring()
: m_pBulletLink(NULL)
{
}

void palBulletGenericLinkSpring::Init(palGenericLink* link) {
	BaseClass::Init(link);
	m_pBulletLink = dynamic_cast<palBulletGenericLink*>(link);
}

void palBulletGenericLinkSpring::SetLinearSpring(palAxis axis, const palSpringDesc& spring) {
	BaseClass::SetLinearSpring(axis, spring);
	if (axis >= PAL_AXIS_COUNT) return;
#if BT_BULLET_VERSION > 282
	m_pBulletLink->BulletGetGenericConstraint()->setStiffness(axis, spring.m_fSpringCoef, false);
	m_pBulletLink->BulletGetGenericConstraint()->setDamping(axis, spring.m_fDamper, false);
#else
	m_pBulletLink->BulletGetGenericConstraint()->setStiffness(axis, spring.m_fSpringCoef);
	m_pBulletLink->BulletGetGenericConstraint()->setDamping(axis, spring.m_fDamper);
#endif
	m_pBulletLink->BulletGetGenericConstraint()->setEquilibriumPoint(axis, spring.m_fTarget);
	bool enable = spring.m_fSpringCoef > FLT_EPSILON;
	m_pBulletLink->BulletGetGenericConstraint()->enableSpring(axis, enable);
#if BT_BULLET_VERSION > 282
	m_pBulletLink->BulletGetGenericConstraint()->setBounce(axis, enable ? btScalar(0.3): btScalar(0.0));
#endif
}

void palBulletGenericLinkSpring::GetLinearSpring(palAxis axis, palSpringDesc& out) const {
	BaseClass::GetLinearSpring(axis, out);
}

void palBulletGenericLinkSpring::SetAngularSpring(palAxis axis, const palSpringDesc& spring) {
	BaseClass::SetAngularSpring(axis, spring);
	if (axis >= PAL_AXIS_COUNT) return;
	unsigned axisIndex = int(axis) + int(PAL_AXIS_COUNT);
#if BT_BULLET_VERSION > 282
	m_pBulletLink->BulletGetGenericConstraint()->setStiffness(axisIndex, spring.m_fSpringCoef, false);
	m_pBulletLink->BulletGetGenericConstraint()->setDamping(axisIndex, spring.m_fDamper, false);
#else
	m_pBulletLink->BulletGetGenericConstraint()->setStiffness(axisIndex, spring.m_fSpringCoef);
	m_pBulletLink->BulletGetGenericConstraint()->setDamping(axisIndex, spring.m_fDamper);
#endif
	m_pBulletLink->BulletGetGenericConstraint()->setEquilibriumPoint(axisIndex, spring.m_fTarget);
	bool enable = spring.m_fSpringCoef > FLT_EPSILON;
	m_pBulletLink->BulletGetGenericConstraint()->enableSpring(axisIndex, enable);
#if BT_BULLET_VERSION > 282
	m_pBulletLink->BulletGetGenericConstraint()->setBounce(axis, enable ? btScalar(0.3): btScalar(0.0));
#endif
}

void palBulletGenericLinkSpring::GetAngularSpring(palAxis axis, palSpringDesc& out) const {
	BaseClass::GetAngularSpring(axis, out);
}

void palBulletGenericLinkSpring::Apply(float dt) {

}

