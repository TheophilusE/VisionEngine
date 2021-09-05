#ifndef PALFACTORY_H
#define PALFACTORY_H
//(c) Adrian Boeing 2004, see liscence.txt (BSD liscence)
#include "pal.h"
#include "palException.h"
#include <iosfwd>

/** \file palFactory.h
	\brief
		PAL Factory -	Physics Abstraction Layer.
						The factory required to create all objects

	\author
		Adrian Boeing
	\version
	<pre>
	Revision History:
		Version 0.2.14: 29/10/08 - Cleanup bugfix
		Version 0.2.13: 10/10/08 - Cleanup update to remove constraints first
		Version 0.2.12: 30/09/08 - PAL API Versioning
		Version 0.2.11: 15/07/08 - Convex object create method
		Version 0.2.1 : 05/07/08 - Notification added.
		Version 0.2.01: 13/12/07 - lib fix
		Version 0.2   : 06/12/07 - DLL singleton cleanup
		Version 0.1.33: 28/06/07 - DLL support
		Version 0.1.32: 28/08/04 - Diagrams
		Version 0.1.31: 07/08/04 - Materials fix
		Version 0.1.3 : 04/08/04 - Internal changes (parenting)
		Version 0.1.2 : 27/07/04 - Doxygen documentation
		Version 0.1.1 : 25/07/04 - Physics state management
		Version 0.1.0 : 05/07/04 - Compound body (geom), velocimeter
		Version 0.0.99: 28/06/04 - Gyroscope create
		Version 0.0.98: 24/06/04 - Singleton, contact sensor create
		Version 0.0.95: 12/06/04 - safe memory cleanup
		Version 0.0.9 : 11/04/04 - terrain functions, prismatic link, and generic 'create object'
		Version 0.0.8 : 06/04/04
	</pre>
	\todo
 */

#define PF palFactory::GetInstance()

/**	The PAL factory class.
	This singelton class is responsible for the construction, and removal of all objects in PAL.

	The factory allows you to select any existing physics implementation system at runtime, and create whichever objects you require.
	Custom objects and extended implementations are automatically imported by the factory.
 */
#ifndef INTERNAL_DEBUG
class palFactory : private myFactory {
#else
	class palFactory : public myFactory {
#endif
	public:
		palFactory();

		/** Returns the version of the PAL API
		 */
		unsigned int GetPALAPIVersion();

		/**	Selects the underlying physics engine to be used when construction objects. (eg: ODE).
	This function must be called before any objects are created.

	If the call was succesfull then subsequent create calls should succeed, else, they will return null.

	\param name The name of the physics engine to be used
	\return whether the requested engine was able to be selected
		 */
		bool SelectEngine(const PAL_STRING& name);

		/**
	Removes all the objects created - regardless of which engine they were constructed with.
		 */
		void Cleanup();

		/** Creates the physics class.
	This should be created and initialized before any other objects are created for the current physics engine
	\return A newly constructed physics class, specified by the select method
		 */
		palPhysics *CreatePhysics();
		//
		/** Creates a static ground plane
	\return A newly constructed terrain plane class, specified by the select method
		 */
		palTerrainPlane *CreateTerrainPlane();
		/** Creates a static ground height map
	\return A newly constructed terrain heightmap class, specified by the select method
		 */
		palTerrainHeightmap *CreateTerrainHeightmap();
		/** Creates a static environment
	\return A newly constructed terrain mesh class, specified by the select method
		 */
		palTerrainMesh *CreateTerrainMesh();
		//
		/** Creates a box.
	<img src="../pictures/cube.jpg" alt="box">
	\return A newly constructed box class, specified by the select method
		 */
		palBox *CreateBox();
		/** Creates a sphere.
	<img src="../pictures/sphere.jpg" alt="sphere">
	\return A newly constructed sphere class, specified by the select method
		 */
		palSphere *CreateSphere();
		/** Creates a convex object.
	\return A newly constructed convex object class, specified by the select method
		 */
		palConvex *CreateConvex();
		/** Creates a capped cylinder.
	<img src="../pictures/capsule.jpg" alt="cylinder">
	\return A newly constructed capped cylinder class, specified by the select method
		 */
		palCapsule *CreateCapsule();
		/** Creates a compound body.
	<img src="../pictures/compoundbody.jpg" alt="compound">
	\return A newly constructed compound body class, specified by the select method
		 */
		palCompoundBody *CreateCompoundBody();
		/** Creates a generic body.
	\return A newly constructed generic body class, specified by the select method
		 */
		palGenericBody *CreateGenericBody();
		palGenericBody *CreateGenericBody(palMatrix4x4& pos);

		palStaticConvex *CreateStaticConvex();

		/** Creates a box geometry.  This can be added to a compound or generic body
	 \return A new constructed box geometry
		 */
		palBoxGeometry *CreateBoxGeometry();

		/** Creates a sphere geometry.  This can be added to a compound or generic body
	 \return A new constructed sphere geometry
		 */
		palSphereGeometry *CreateSphereGeometry();

		/** Creates a capped cylinder geometry.  This can be added to a compound or generic body
	 \return A new constructed capped cylinder geometry
		 */
		palCapsuleGeometry *CreateCapsuleGeometry();

		/** Creates a cylinder geometry.  This can be added to a compound or generic body
    \return A new constructed cylinder geometry
		 */
		palCylinderGeometry *CreateCylinderGeometry();

		/** Creates a convex mesh geometry.  This can be added to a compound or generic body
	 It will need to be given a set of vertices from which to create a convex hull.
	 \return A new constructed convex hull geometry
		 */
		palConvexGeometry *CreateConvexGeometry();
		palConvexGeometry *CreateConvexGeometry(palMatrix4x4 &pos,
				const Float *pVertices,
				int nVertices, Float mass);
		palConvexGeometry *CreateConvexGeometry(palMatrix4x4 &pos,
				const Float *pVertices,
				int nVertices,
				const int *pIndices,
				int nIndices,
				Float mass);

		/** Creates a concave mesh geometry.  This can be added to a compound or generic body
	 A concave mesh is essentially any triangle mesh that cannot be optimized into a convex hull.
	 \return A new constructed potentially concave triangle mesh geometry
		 */
		palConcaveGeometry *CreateConcaveGeometry();
		palConcaveGeometry *CreateConcaveGeometry(palMatrix4x4 &pos,
				const Float *pVertices,
				int nVertices,
				const int *pIndices,
				int nIndices,
				Float mass);

		/**
		 * Creates a concave mesh geometry that is agnostic of the datastructure behind it. This is a means
		 * to create a custom concave shape.
		 */
		palCustomConcaveGeometry *CreateCustomConcaveGeometry();


		/**
		 * Creates an uninitialized link with the given type.
		 */
		palLink *CreateLink(palLinkType type);

		/**
		 * Creates and initializes any sort of link using the frames passed in.
		 * A frame is the relative transformation matrix from the position and rotation of a body to the
		 * joint pivot point.  You can easily create one with a translation and rotation of some sort. Look at palMath.h at
		 * the mat_ functions like mat_translate, mat_set_rotation, and mat_rotate.
		 * \note To set joint limits, after creation, look at the SetParam/GetParam functions on palLink.
		 *
		 * \see #mat_set_rotation
		 * \see #mat_rotate
		 */
		palLink *CreateLink(palLinkType type, palBodyBase *parent, palBodyBase *child,
				const palMatrix4x4& parentFrame,
				const palMatrix4x4& childFrame,
				bool disableCollisionsBetweenLinkedBodies = true);

		/**
		 * Creates and initializes any sort of link using an pivot and an axis.  This method seems simpler
		 * than the frame version at first, but the complexity in this method is that the pivot and axis are in world space, and therefore
		 * depend on the current position of the two bodies, meaning you may have to transform these yourself before seting the values.
		 * Also, the axis frame of reference depends on the type of joint, that is, it's the Z Axis on the generic, revolute, and fixed joint, and the X axis on
		 * the prismatic.  This isn't too much of a problem in many cases because the axis is the one you care about on the revolute and fixed and it doesn't matter that
		 * much on the fixed.
		 *
		 * \note To set joint limits, after creation, look at the SetParam/GetParam functions on palLink.
		 *
		 * \see #mat_set_rotation
		 * \see #mat_rotate
		 */
		palLink *CreateLink(palLinkType type, palBodyBase *parent, palBodyBase *child,
				const palVector3& pivot, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies = true);

		//
		/** Creates a spherical link
	A spherical link has three degress of freedom. It is also known as a ball-and-socket joint. (example: hip-leg joint)
	<img src="../pictures/sphericallink.png">
	\return A newly constructed spherical link class, specified by the select method
	\see CreateLink
		 */
		palSphericalLink *CreateSphericalLink();

		/** Creates a revolute link
	A revolute link has one degree of rotational freedom. It is also know as a hinge joint. (example: door)
	<img src="../pictures/hinge.jpg">
	\return A newly constructed revolute link class, specified by the select method
	\see CreateLink
		 */
		palRevoluteLink	*CreateRevoluteLink();

		/** Creates a revolute spring link
	A revolute link has one degree of rotational freedom. It is also know as a hinge joint. (example: door)
	This version also allows applying sping constants to the hinge.  This version is not creatable from the CreateLink
	function, so it has its own create and init function.
	<img src="../pictures/hinge.jpg">
	\return A newly constructed revolute link class, specified by the select method
	\see CreateLink
		 */
		palRevoluteSpringLink *CreateRevoluteSpringLink();
		palRevoluteSpringLink *CreateRevoluteSpringLink(palBodyBase *parent, palBodyBase *child,
				const palVector3& pivot, const palVector3& axis, bool disableCollisionsBetweenLinkedBodies = true);

		/** Creates a prismatic link
	A prismatic link has one degree of translational freedom. It is also know as a slider joint. (example: slide rule, hydrolic ram)
	<img src="../pictures/prismatic.jpg">
	\return A newly constructed prismatic link class, specified by the select method
	\see CreateLink
		 */
		palPrismaticLink *CreatePrismaticLink();

		/** Creates a generic link
	A generic link may have up to 3 translational and 3 rotational degrees of freedom.
	\return A newly constructed generic link
		 */
		palGenericLink *CreateGenericLink();

		/** Creates a rigid link
	A rigid link may have up to 3 translational and 3 rotational degrees of freedom.
	\return A newly constructed rigid link
		 */
		palRigidLink *CreateRigidLink();
		/**
		 * This version of CreateRigidLink is a convenience function where you can just rigidly link two bodies
		 * together at their current relative positions.
		 */
		palRigidLink *CreateRigidLink(palBodyBase *parent, palBodyBase *child,
				bool disableCollisionsBetweenLinkedBodies = true);
		//
		/** Creates a PSD sensor
	This sensor tells you the distance from one object to another. This is also called raycasting.
	<img src="../pictures/psdsensor.jpg">
	\return A newly constructed PSD sensor class, specified by the select method
		 */
		palPSDSensor *CreatePSDSensor();

		/** Creates a contact sensor
	This sensor tells you whether the object has collided with another object. This is sometimes called collision detection/querying.
	<img src="../pictures/contact.jpg">
	\return A newly constructed contact sensor class, specified by the select method
		 */
		palContactSensor *CreateContactSensor();

		/** Creates an inclinometer sensor
	This sensor tells you the angle between its starting orientation, and the bodies current orientation. (Angle sensor)
	\return A newly constructed inclinometer sensor class.
		 */
		palInclinometerSensor *CreateInclinometerSensor();

		/** Creates a compass sensor
	This sensor tells you the angle between its starting orientation, and the bodies current orientation. (Angle sensor)
	\return A newly constructed compass sensor class.
		 */
		palCompassSensor *CreateCompassSensor();

		//remove this sensor to seperate DLL?
		/** Creates an gyroscope sensor
	This sensor tells you the change in the angular velocity of a body.
	\return A newly constructed gyroscope sensor class.
		 */
		palGyroscopeSensor *CreateGyroscopeSensor();

		//remove this sensor to seperate DLL?
		palVelocimeterSensor *CreateVelocimeterSensor();

		//remove this sensor to seperate DLL?
		palGPSSensor *CreateGPSSensor();

		/** Creates an angular motor
		 * \return a newly created angular motor
		 * \param pLink the link that will use the motor.  It accepts a revolute link or, in some engines, a 6DOF.
		 * \param axis The axis is only used if the link is a 6-DOF/GenericLink X=0,Y=1,or Z=2.
		 */
		palMotor* CreateMotor(palLink *pLink = 0, int axis = -1);

		//
		//low-level creations, standard user shouldn't use these:
		/** Creates any PAL object
	This will return the most suitable class that matches the currently selected engine, and name. This function can be used to construct objects which are not part of the standard PAL implementation. (eg: custom plug-ins)
	\return A newly constructed PAL object
		 */
		palFactoryObject *CreateObject(const PAL_STRING& name); //this is only to be used for user add-on functionality

		/// Handy template CreateObject that can be used to create and cast an object for you.
		template<typename T>
		T* CreateObject(const PAL_STRING& name)
		{
			palFactoryObject* pfo = CreateObject(name);
			T* result = dynamic_cast<T*>(pfo);
			if (result == NULL)
			{
				delete pfo;
			}
			return result;
		}

		palPhysics *GetActivePhysics();
		void SetActivePhysics(palPhysics *physics);
		void LoadPALfromDLL(const char *szPath = NULL) throw(palException);

		static const char* PAL_PLUGIN_PATH;
		/**
		 * Loads available physics engine libraries from specified directory. If no directory is given, the environment variable PAL_PLUGIN_PATH is used.
		 * @param dirName where to look for physics engine libraries
		 */
		void LoadPhysicsEngines(const char* dirName = NULL);

		void DumpObjects(const PAL_STRING& separator = "\n");
		void DumpObjects(std::ostream& out, const PAL_STRING& separator = "\n");
	protected:
		typedef MemoryObjectManager<StatusObject>::MMOType MMOType;
	private:
		palPhysics *m_active;
	public:
		static palFactory *GetInstance();
		static void SetInstance(palFactory *pf);
	};

#endif
