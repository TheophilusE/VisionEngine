#ifndef PALLINKS_H
#define PALLINKS_H
//(c) Adrian Boeing 2004, see liscence.txt (BSD liscence)
/**	\file palLinks.h	
	\brief
		PAL - Physics Abstraction Layer. 
		Links

	\author
		Adrian Boeing
	\version
	<pre>
	Revision History:
		Version 0.3.24: 14/10/08 - Generic link frame matrix
		Version 0.3.23: 01/10/08 - Generic link enum and init
		Version 0.3.22: 11/07/08 - Get angle epislon bugfix, get revolute position
		Version 0.3.21: 10/07/08 - Get angle bugfix
		Version 0.3.2 : 11/08/04 - Revolute link, torque&velocity
		Version 0.3.1 : 28/07/04 - Doxygen
		Version 0.3   : 04/07/04 - Split from pal.h 
	</pre>
	\todo
 */
#include "palBodies.h"
#include "palStringable.h"
#include <iosfwd>
#include "pal/palException.h"

/** The type of link
 */
typedef enum {
	PAL_LINK_NONE = 0,
	PAL_LINK_SPHERICAL = 1, //!< Spherical link, (ball&socket) 3d rotation
	PAL_LINK_REVOLUTE = 2, //!< Revolute link, (hinge) 1d rotation
	PAL_LINK_PRISMATIC = 3, //!< Prismatic link, (slider) 1d translation
	PAL_LINK_GENERIC = 4, //!< Generic 6DOF link
	PAL_LINK_RIGID = 5, //!< Immovable link
} palLinkType;

typedef enum {
	// Start at a high number so you can put in low numbers for real engine numbers not shown in this list if the engine supports them.
	PAL_LINK_PARAM_BASE = 0x800,
	PAL_LINK_PARAM_ERP, //!< Error reduction parameter.  The amount a joint error is reduced each tick.  Values 0.2 to 0.8 are recommended
	PAL_LINK_PARAM_STOP_ERP, //!< Error reduction parameter, but for the stop of a joint.
	PAL_LINK_PARAM_CFM, //!< Constraint force mixing.  0 allows for no error, > 0 .. 1 allows for some error, and it cause the joints to be more stable.
	PAL_LINK_PARAM_STOP_CFM, //!< Constraint force mixing, but for the stop on a joint.
	PAL_LINK_PARAM_BREAK_IMPULSE, //!< The impulse value that will cause the constraint to break.
	/*
	 * Joint degrees of freedom, also called Limits.  Min < Max means the degree is limited.  Min = Max means it is locked.  Min > Max means free movement.
	 * The Axis is passed into the function separately. Angular values are in radians.
	 */
	PAL_LINK_PARAM_DOF_MIN,
	PAL_LINK_PARAM_DOF_MAX,

	/**
	 * The offset of the bodies relative to the joint origin. As usual, axes 0 - 2 are translation x-z and 3-5 are rotation x-z.
	 */
	PAL_LINK_RELATIVE_BODY_POS_OR_ANGLE
} palLinkParam;

//corkscrew?
//universal ( 2d)

class palSpringDesc {
public:
	palSpringDesc();
	/// The damping coefficient
	Float m_fDamper;
	/// The spring coefficient
	Float m_fSpringCoef;
	/// The length at which the spring forces are 0;
	Float m_fTarget;
};

class palLinkFeedback : public palStringable {
public:
	virtual bool IsEnabled() const = 0;
	virtual bool SetEnabled(bool enable) = 0;
	virtual Float GetValue() const = 0;
	virtual std::string toString() const;
};

/** The base link class.
	Connects two bodies together via a given constraint.
	All links coordinates are specified in world space unless otherwise indicated.
	Although the direction of link connections does not matter for most physics engine implementaions, it is often optimal to specify connections steming from one central body out to all the ending body links.
 */
class palLink : public palFactoryObject {
public:
	/** Any joint must be possible to initialize from frame matrices.
	\param parent The "parent" body to connect
	\param child The "child" body to connect
	\param parentFrame the matrix to move from the space of the parent body to the joint position and rotation.
	\param childFrame the matrix to move from the space of the parent body to the joint position and rotation.
	 */
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies = true) = 0;
	/** Initializes the link.
	\param parent The "parent" body to connect
	\param child The "child" body to connect
	\param pos the xyz position of the link
	\param axis the axis of the joint.  The interpretation of the axis varies per joint.  On the prismatic it could be the default X axis and Z on the revolute.
	 */
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies = true) = 0;

	palLinkType GetLinkType() const { return m_Type; }

	palBodyBase *GetParentBody() const { return m_pParent; }
	palBodyBase *GetChildBody() const { return m_pChild; }

	/**
	 * \return true if this link is enabled, i.e. limiting velocity and relative position
	 */
	virtual bool GetEnabled() const { return true; }

	/**
	 * \enable true if this link should be enabled, or false if not.
	 * \return true if changing the value was successful.  By default, this is not implemented, return false if you try to disable it.
	 */
	virtual bool SetEnabled(bool enable)
	{
		if (enable)
			return true;
		return false;
	}

	/**
	 * @param posOut Fills this vec3 with the current world position of the joint according to the parent body.
	 */
	void GetPosition(palVector3& posOut) const;

	/**
	 * @param frameOut output parameter that will be filled with the frame matrix for the parent.  In many cases, this
	 *                 Won't need to be computed
	 */
	virtual void ComputeFrameParent(palMatrix4x4& frameOut) const = 0;
	/**
	 * @param frameOut output parameter that will be filled with the frame matrix for the parent.  In many cases, this
	 *                 Won't need to be computed
	 */
	virtual void ComputeFrameChild(palMatrix4x4& frameOut) const = 0;

	/**
	 * Sets the value of a joint parameter.  The code is defined by the engine to which it applies.
	 * Some engines allow the value to be set per axis.  Axes are defined as x,y,z,Rx,Ry,Rz, though what is available
	 * to configure is not defined.  Passing -1 means to set the default axis, which usually means the logical axis such
	 * as the main axis of rotation for a revolute link.  On other links, it means all axes.  Some links require an axis
	 * to be provided.
	 * If anything passed in is not supported, it should return false.
	 * Some engines don't support any joint parameters at all.
	 */
	virtual bool SetParam(int parameterCode, Float value, int axis = -1);
	/**
	 * @return the value of a joint parameter or -1 if the value is not supported.
	 */
	virtual Float GetParam(int parameterCode, int axis = -1);
	// @return true if this joint/the current engine supports joint parameters of any kind.
	virtual bool SupportsParameters() const;
	// @return true if some joint parameters can be set per axis, i.e. if the axis parameter on SetParam should be used.
	virtual bool SupportsParametersPerAxis() const;

	virtual std::string toString() const;

	virtual palLinkFeedback* GetFeedback() const throw(palIllegalStateException);
	/// It has a virtual destructor, so there is no reason not be able to delete one.
	virtual ~palLink();
protected:
	palLink(); // to accomodate the FACTORY_CLASS macro
	palLink(palLinkType linkType);
	/** Set the bodies this link is connected to. (This method
	exists so subclasses can override Init(palBody,palBody)
	without causing an infinite loop.)
	\param parent The "parent" body to connect
	\param child The "child" body to connect
	 */

	/**
	 * This is for implementers.
	 * Utility function that computes frame matrices from the a pivot and axis.
	 * Some joints on some engines will need this to implement the ComputeFrame functions.
	 * Others will need this to compute the frames it uses internally.
	 */
	void ComputeFramesFromPivot(palMatrix4x4& frameAOut, palMatrix4x4& frameBOut, const palVector3& pivot, const palVector3& axis, const palVector3& constraintDefaultAxis) const;

	/**
	 * This is for implementers.
	 * This calls the anchor axis version of init with the frames.  This is useful in engines that don't use frames.  This will be forced to use
	 * the parent's current position and move the child.
	 */
	void CallAnchorAxisInitWithFrames(const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, unsigned axisRow, bool disableCollisionsBetweenLinkedBodies);

	void SetBodies(palBodyBase *parent, palBodyBase *child);
private:
	palLinkType m_Type;
	palBodyBase *m_pParent;
	palBodyBase *m_pChild;
};

/** A Spherical link.
	A spherical link, (also know as a ball and socket link) provides 3 degrees of rotational freedom for the constraint. That is, it can twist about its axis, as well as rotate freely about its central point.
	The twist axis is the Z axis for most physics engines (at least for: Bullet, Novodex/Physx). However, it's possible this is not the case for others.
	The link connects two bodies, at a given position.
	<img src="../pictures/sphericallink.png">
	The diagram indicates the central point of the spherical link.

        Note that not all physics engines support arbitrary limits on the angles.
 */
class palSphericalLink: virtual public palLink {
public:
	palSphericalLink();
	virtual ~palSphericalLink();
};

/** A Revolute Link
	A revolute link (also known as a hinge) provides one degree of rotational freedom for the constraint.
	The link connects two bodies, at a given position, and rotates around a specified axis.

	When using a frame, the revolute link uses the z-axis as the axis of rotation.
	<img src="../pictures/hinge.jpg">
	The diagram illustrates two geometries central positions, and the central pivot point of the revolute link.
	The arrow in the diagram illustrates the axis about which the link can rotate.
 */
class palRevoluteLink: virtual public palLink {
public:
	palRevoluteLink();
	virtual ~palRevoluteLink();

	//	virtual void GenericInit(palBody *pb0, palBody *pb1, void *paramarray);

	/** Gets the current angle (in radians) between the two connected bodies.
	\return Angle (radians) between the two connected bodies.
	 */
	virtual Float GetAngle() const; //current rotation angle

	/** Gets the current angular velocity
	 */
	virtual Float GetAngularVelocity() const;

	/** Applies a torque to act on the link
	 */
	virtual void ApplyTorque(Float torque);

	/** Applies a torque to act on the link
	 */
	virtual void ApplyAngularImpulse(Float torque);

	/**
	 * The axis data members are local to the parent body.  This method returns that axis in
	 * world space by applying the transform of the parent body to it.
	 * @return axis of revolution in world space.
	 */
	virtual palVector3 GetAxis() const;

	virtual std::string toString() const;

};

/** A Revolute Link
	This works just like the revolute link if you don't set a spring on it. If you do set the spring, then the spring
	constraints will cause torques to be applied attempting to move the joint to the target defined in the palSpringDesc
	@see palSpringDesc
 */
class palRevoluteSpringLink: virtual public palRevoluteLink {
public:
	virtual void SetSpring(const palSpringDesc& springDesc) = 0;
	virtual void GetSpring(palSpringDesc& springDescOut) const = 0;
};


/** A Prismatic Link
	A prismatic link (also known as a slider) provides one degree of translational freedom for the constraint.
	The link connects two bodies, at a given position, and extends along a specified axis.
	<img src="../pictures/prismatic.jpg">
	The diagram indicates the central point of two geometries.	The arrow indicates the axis about which the link extends. The point the arrow extends from indicates the starting position for the slider.
 */
class palPrismaticLink : virtual public palLink {
public:
	palPrismaticLink();
	virtual ~palPrismaticLink();
};


class palGenericLink : virtual public palLink {
public:
	palGenericLink();
	virtual ~palGenericLink();
};

class palRigidLink  : virtual public palLink {
public:
	palRigidLink();
	virtual ~palRigidLink();
	virtual void Init(palBodyBase *parent, palBodyBase *child, bool disableCollisionsBetweenLinkedBodies);
	virtual void Init(palBodyBase *parent, palBodyBase *child,
			const palMatrix4x4& parentFrame, const palMatrix4x4& childFrame, bool disableCollisionsBetweenLinkedBodies = true) = 0;
	/** This constructor is kept to maintain consistency.  It may have some effect in some engines, in fact, it would in bullet, but in others, the parameters
	 * Since they are related to relatively unmovable bodies, they just don't matter.
	 */
	virtual void Init(palBodyBase *parent, palBodyBase *child, const palVector3& pos, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies = true) = 0;
};

#endif
