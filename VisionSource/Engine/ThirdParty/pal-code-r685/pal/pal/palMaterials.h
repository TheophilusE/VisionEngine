#ifndef PALMATERIALS_H
#define PALMATERIALS_H

#include "palBase.h"

//(c) Adrian Boeing 2007, see liscence.txt (BSD liscence)

/** The Material definition structure.
   The use of the friction information is dependent on the phyisics implementation used.
   Friction is represented by the Coulomb model of friction.
   Static friction represents the ammount of friction in a material when an object first begins to move.
   Kinetic friction is the friction of a material for an object already in motion.
   Resitution is used to represent an elastic collision (ie: bounce), it is the height an object will bounce when droped onto a material.

   \f[ c = \frac{s_2-v_2}{v_1-s_1} \f]
   Where,
   c = coefficient of restitution
   \f$v_1\f$ = linear velocity of bodyA mass center before impact
   \f$s_1\f$ = linear velocity of bodyB before impact (will be negative according to our convention that away from the player is positive)
   \f$v_2\f$ = linear velocity of bodyA mass center after impact
   \f$s_2\f$ = linear velocity of bodyB ball after impact

   or, in the case of a falling object bouncing off the ground:
   \f[ c = \sqrt{\frac{h}{H} } \f]
   Where,
   h = bounce height
   H = drop height
   */
struct palMaterialDesc {
   palMaterialDesc();
	virtual ~palMaterialDesc() {}

	/*
	 Sets the member variables.
	*/
	virtual void SetParameters(const palMaterialDesc& matDesc);

	Float m_fStatic; //!< Static friction coefficient, defaults to 0.0
   Float m_fKinetic; //!< Kinetic friction coefficient for isotropic or inline with the direction of anisotropy, defaults to 0.0
   Float m_fRestitution; //!< Restitution coefficient defaults to 0.5
   palVector3 m_vStaticAnisotropic; //!< Anistropic friction vector 0-1 based on the friction coefficient.  Defaults to 1,1,1.
   palVector3 m_vKineticAnisotropic; //!< Anistropic friction vector 0-1 based on the friction coefficient. Defaults to 1,1,1.
   palVector3 m_vDirAnisotropy; //!< A direction vector defining the main direction of anisotropy, which allows rotating the anistrophic vector. Defaults 1,0,0.
   bool m_bEnableAnisotropicFriction; //!< defaults to false;
   bool m_bDisableStrongFriction; //!<Some engines accumulate left over friction across frames, this will disable it if the engine supports it. Defaults to false.
};

/*! \file palBase.h
	\brief
		PAL - Physics Abstraction Layer. 
		Materials
	\author
		Adrian Boeing
	\version
	<pre>
		Version 0.1   : 11/12/07 - Original
	</pre>
	\todo
*/

class palMaterial;
class palMaterialInteraction;

/** This is the base material class.
	This class is only neccessary when constructing a new PAL physics implementation.
	To obtain a pointer to a Material you need to access it via palMaterials::GetMaterial().
*/
class palMaterial : public palFactoryObject, public palMaterialDesc {
public:
	palMaterial();
	palMaterial(const palMaterial& m);
	virtual ~palMaterial();

	virtual void Init(const PAL_STRING& name, const palMaterialDesc& desc); //api version 3

	/// The id is used internally in the material system.
	unsigned GetId() const { return m_Id; }
	void SetId(unsigned newId) { m_Id = newId; }

	const PAL_STRING& GetName() const;

	// This is set when you create a custom interaction.
	void SetHasCustomInteractions(bool enabled);
	// Inline because it could be called for each contact.
	bool GetHasCustomInteractions() const { return m_bHasCustomMaterialInteractions; }
protected:
	FACTORY_CLASS(palMaterial,palMaterial,*,1);
private:
	virtual palMaterial& operator=(const palMaterial& m) { return *this; };
	PAL_STRING m_Name;//!< The name for this material. (eg:"wood")
	unsigned m_Id;
	bool m_bHasCustomMaterialInteractions; //!< material interactions are ignored by default.  This value
};

class palContactPoint;

class palMaterialInteractionCollisionCallback
{
public:
	virtual ~palMaterialInteractionCollisionCallback();
	/**
	 * This is a custom collision callback for a specific material interaction.
	 * @param pmi The material interaction, provided for your convenience if you need to use data from it.
	 * @param materialToAdjust The material description.  You can change these as you wish.
	 * @param contactToAdjust The contact that combines the two materials.  You may adjust this contact before the results are applied.
	 * @return true if you actually changed the data.  if you return false, the changes won't be used.
	 */
	virtual bool operator()(const palMaterialInteraction& pmi, palMaterialDesc& materialToAdjust, palContactPoint& contactToAdjust) = 0;
};

/** This class represents two material interactions (eg: wood/metal).  
	This class is only used internally by palMaterials, and should not be manually created 
 */
class palMaterialInteraction : public palFactoryObject, public palMaterialDesc {
public:
	palMaterialInteraction();

	/*
	Initializes the material
	\param pM1 a pointer to a unique material
	\param pM2 a pointer to a unique material
   \param desc the material description
	*/
	virtual void Init(palMaterial *pM1, palMaterial *pM2, const palMaterialDesc& matDesc); //api version 2
	virtual palMaterialInteraction& operator=(const palMaterialInteraction& pmi);

	palMaterial* getMaterial1() { return m_pMaterial1; }
	palMaterial* getMaterial2() { return m_pMaterial2; }

	/**
	 * This sets a custom collision interaction callback.
	 * This callback allows one to compute custom material interactions and modify the collision contact.
	 * Only some engines will actually use material interactions.
	 */
	void SetCollisionCallback(palMaterialInteractionCollisionCallback* callback) {m_pCollsionCallback = callback;}
	palMaterialInteractionCollisionCallback* GetCollisionCallback() {return m_pCollsionCallback;}
protected:
	FACTORY_CLASS(palMaterialInteraction,palMaterialInteraction,*,1);
private:
	palMaterialInteraction(const palMaterialInteraction& pmi);

	palMaterial* m_pMaterial1;	//!< Pointers to the unique materials which interact
	palMaterial* m_pMaterial2;	//!< Pointers to the unique materials which interact
	palMaterialInteractionCollisionCallback* m_pCollsionCallback;
};


/** The materials management class.
	This class allows you to add materials into the physics engine. The class maintains a library of all materials created and generates the appropriate underlying data structures. 
	A Material can be extracted from the Materials library using the GetMaterial() function.

	<br>
	Friction is represented by the Coulomb model of friction.
	Static friction represents the ammount of friction in a material when an object first begins to move.
	Kinetic friction is the friction of a material for an object already in motion.
	Resitution (elasticity) is used to represent an elastic collision (ie: bounce), it is the height an object will bounce when droped onto a material.

	\f[ c = \frac{s_2-v_2}{v_1-s_1} \f]
	Where,
	c = coefficient of restitution
	\f$v_1\f$ = linear velocity of bodyA mass center before impact
	\f$s_1\f$ = linear velocity of bodyB before impact (will be negative according to our convention that away from the player is positive)
	\f$v_2\f$ = linear velocity of bodyA mass center after impact
	\f$s_2\f$ = linear velocity of bodyB ball after impact

	or, in the case of a falling object bouncing off the ground:
	\f[ c = \sqrt{\frac{h}{H} } \f]
	Where,
	h = bounce height
	H = drop height

	<br>
	Developer Notes:
	The materials management class employes the palMaterialInteraction and palMaterial classes behind the scenes.
	You should only need to implement the aforementioned classes. 
*/
class palMaterials : public palFactoryObject {
public:
	palMaterials();
	~palMaterials();
	/**
	Creates a new material.
	\param name The materials name (eg:"wood")
	\param static_friction Static friction coefficient
	\param kinetic_friction Kinetic friction coefficient
	\param restitution Restitution coefficient
	*/
	virtual palMaterial* NewMaterial(const PAL_STRING& name, const palMaterialDesc& matDesc);

	/**
	Retrievies a unique material from the materials database with a given name.
	The Material inherits from the Material class.
	\param name The material's name (eg:"wood")
	\return A pointer to the material
	*/
	virtual palMaterial* GetMaterial(const PAL_STRING& name);

	/**
	Sets parameters for one materials interaction with another
	\param name1 The first materials name (eg:"wood")
	\param name2 The second materials name (eg:"metal")
	\param matDesc the material data.
	*/
	virtual void SetMaterialInteraction(const PAL_STRING& name1, const PAL_STRING& name2, const palMaterialDesc& matDesc);

	/**
	Sets parameters for one materials interaction with another
	\param id1 The index of the first material
	\param id2 The index of the second material
	\param matDesc the material data.
	*/
	virtual void SetMaterialInteraction(palMaterial* pm1, palMaterial* pm2, const palMaterialDesc& matDesc);

	/**
	 * This is to be called from the Physics Engine impl.
	 * Does some sanity checking, tests for NULL, conversions to unique materials types, checks to see if interactions setup for each material, then
	 * it looks up the interaction and returns it.  If either body has no material, it will return NULL.
	 * If combine is true and no interaction is defined, it will combine them.  if combine is false and no interaction is defined
	 * it will return NULL.
	 * \param pm1 the first material.  You may pass NULL.
	 * \param pm2 the second material.  You may pass NULL.
	 */
	bool HandleCustomInteraction(palMaterial* pm1, palMaterial* pm2, palMaterialDesc& matToAdjust, palContactPoint& contactToAdjust, bool combine);

	/**
	Retrievies an interaction for the two named materials, if it exists.
	This function is not particularly fast.  The other version that takes two materials is O(1)
	\param name1 The first material's name (eg:"wood")
	\param name2 The second material's name (eg:"steel")
	\return A pointer to the material interaction
	*/
	virtual palMaterialInteraction* GetMaterialInteraction(const PAL_STRING& name1, const PAL_STRING& name2);

	/**
	Retrievies an interaction for the two named materials, if it exists.
	\param pm1 The first material
	\param pm2 The second material
	\return A pointer to the material interaction
	*/
	virtual palMaterialInteraction* GetMaterialInteraction(palMaterial* pm1, palMaterial* pm2);

	// When defaulting the material interactions, this method combines them.
	virtual void CombineMaterials(const palMaterialDesc& one, const palMaterialDesc& two, palMaterialDesc& result);

protected:

	PAL_VECTOR<palMaterial*> m_Materials;
	struct InteractionData
	{
		InteractionData(): m_pMatInteration(0) {}
		palMaterialInteraction* m_pMatInteration;
	};
	std_matrix<InteractionData> m_MaterialInteractions;

	virtual unsigned GetIndex(const PAL_STRING& name) const;

	FACTORY_CLASS(palMaterials,palMaterials,*,1);
};


#endif
