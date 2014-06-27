#ifndef CREATURE_H
#define CREATURE_H

#include "btBulletDynamicsCommon.h"

class Creature {

public:
	Creature();
	Creature (btDynamicsWorld* ownerWorld, const btVector3& positionOffset); // Constructor

	virtual	~Creature(); // Destructor

	void update(int elapsedTime);		// Update the creature state
	bool hasFallen();					// Return if the creature has fallen down
	void switchCOM();					// Activate / Deactivate the visualization of the COM
	bool m_showCOM;						// Show COM
	const char* name;					// Name of the player
	btRigidBody*	m_ball;				// Ball

	btVector3 getCOM() {return m_positionCOM;}	// Return the position of the COM

public:

	enum Part {BODYPART_FOOT,BODYPART_LOWER_LEG,BODYPART_UPPER_LEG,BODYPART_COUNT}; // Body parts of the creature

	enum Joint {JOINT_ANKLE,JOINT_KNEE,JOINT_COUNT}; // Joints of the creature

	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btCollisionShape	*	m_shapes[BODYPART_COUNT];	// The primitive shape of each body part used in collision
	btRigidBody			*	m_bodies[BODYPART_COUNT];	// The array of body parts
	btTypedConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint

	int lastChange;										// Time of last change of balance controller

	bool	m_hasFallen;		// Indicates if the creature has already fallen down

	btCollisionShape	*	m_COMShape;		// Shape for COM
	btRigidBody			*	m_COM;			// Body COM
	btVector3				m_positionCOM;	// Position COM
	btVector3				computeCenterOfMass(Part from = (Part)0);		// Compute the COM of the creature in world coordinate system from the given part up
	float					computeTotalMass(Part from = (Part)0);		// Compute the mass of the creature in world coordinate system from the given part up

	btVector3				computeCenterOfMassBelow(Part to);		// Compute the COM of the creature in world coordinate system from the given part up
	float					computeTotalMassBelow(Part to);		// Compute the mass of the creature in world coordinate system from the given part up

private:
	btScalar				prevError[JOINT_COUNT];
};

#endif
