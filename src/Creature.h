
#ifndef CREATURE_H
#define CREATURE_H

#include "btBulletDynamicsCommon.h"

class Creature {

public:
	Creature (btDynamicsWorld* ownerWorld, const btVector3& positionOffset); // Constructor

	virtual	~Creature(); // Destructor

	void update(int elapsedTime);		// Update the creature state
	bool hasFallen();					// Return if the creature has fallen down
	void switchCOM();					// Activate / Deactivate the visualization of the COM
	bool m_showCOM;						// Show COM

	btVector3 getCOM() {return m_positionCOM;}	// Return the position of the COM

protected:

	enum {BODYPART_FOOT,BODYPART_LOWER_LEG,BODYPART_UPPER_LEG,BODYPART_UPPER_ARM, BODYPART_LOWER_ARM, BODYPART_COUNT}; // Body parts of the creature

	enum {JOINT_ANKLE, JOINT_KNEE, JOINT_ELBOW, JOINT_SHOULDER, JOINT_COUNT}; // Joints of the creature

	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btCollisionShape	*	m_shapes[BODYPART_COUNT];	// The primitive shape of each body part used in collision
	btRigidBody			*	m_bodies[BODYPART_COUNT];	// The array of body parts
	btTypedConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint

	int lastChange;										// Time of last change of balance controller

	bool	m_hasFallen;		// Indicates if the creature has already fallen down

	btCollisionShape	*	m_COMShape;		// Shape for COM
	btRigidBody			*	m_COM;			// Body COM
	btVector3				m_positionCOM;	// Position COM
	btVector3 computeCenterOfMass();		// Compute the COM of the creature in world coordinate system

};

#endif
