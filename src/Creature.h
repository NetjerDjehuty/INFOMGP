#ifndef CREATURE_H
#define CREATURE_H

#include "btBulletDynamicsCommon.h"


#define	GAINSP 10.0f
#define GAINSD 100.0f

#define EXTRAPARTS 4
#define EXTRAPARTSLENGTH 0.1f
#define EXTRAPARTSWEIGHT 0.5f





#define BODYPART_FOOT 0
#define BODYPART_LOWER_LEG 1
#define BODYPART_UPPER_LEG 2
#define BODYPART_COUNT (3 + EXTRAPARTS)

#define JOINT_ANKLE 0
#define JOINT_KNEE 1
#define JOINT_COUNT (2 + EXTRAPARTS)

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

	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btCollisionShape	*	m_shapes[BODYPART_COUNT];	// The primitive shape of each body part used in collision
	btRigidBody			*	m_bodies[BODYPART_COUNT];	// The array of body parts
	btTypedConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint

	int lastChange;										// Time of last change of balance controller

	bool	m_hasFallen;		// Indicates if the creature has already fallen down

	btCollisionShape	*	m_COMShape;		// Shape for COM
	btRigidBody			*	m_COM;			// Body COM
	btVector3				m_positionCOM;	// Position COM
	btVector3				computeCenterOfMass(int from = 0);		// Compute the COM of the creature in world coordinate system from the given part up
	float					computeTotalMass(int from = 0);		// Compute the mass of the creature in world coordinate system from the given part up

	btVector3				computeCenterOfMassBelow(int to);		// Compute the COM of the creature in world coordinate system from the given part up
	float					computeTotalMassBelow(int to);		// Compute the mass of the creature in world coordinate system from the given part up

private:
	btScalar				prevError[BODYPART_COUNT];
};

#endif
