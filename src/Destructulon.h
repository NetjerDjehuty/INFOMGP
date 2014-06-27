
#ifndef DESTRUCTULON_H
#define DESTRUCTULON_H

//#include "Creature.h"
#include "Environment.h"
#include "Cape.h"
#include "btBulletDynamicsCommon.h"

class Destructulon
{
public:
	Destructulon (btSoftRigidDynamicsWorld* ownerWorld, const btVector3& positionOffset, Environment *environment); // Constructor
	virtual	~Destructulon(); // Destructor

	void update(int elapsedTime);		// Update the creature state
	bool hasFallen();					// Return if the creature has fallen down
	void switchCOM();					// Activate / Deactivate the visualization of the COM
	bool m_showCOM;						// Show COM
	const char* name;					// player name
	bool firstLoop;						// To start with arms facing downwards
	btRigidBody*	m_ball;				// Ball
	Destructulon*	opponent;			// Ball
	bool cape;							// Boolean for cape

	btVector3 getCOM() {return m_positionCOM;}	// Return the position of the COM
	void toggleCape();							// Change visibility of the cape

public:

	enum {BODYPART_FOOT,BODYPART_LOWER_LEG,BODYPART_UPPER_LEG, BODYPART_UPPER_ARM, /*BODYPART_LOWER_ARM,*/ BODYPART_UPPER_L_ARM, /*BODYPART_LOWER_L_ARM, BODYPART_HEAD,*/ BODYPART_COUNT}; // Body parts of the creature

	enum {JOINT_ANKLE,JOINT_KNEE, JOINT_SHOULDER, /*JOINT_ELBOW,*/ JOINT_L_SHOULDER, /*JOINT_L_ELBOW, JOINT_NECK,*/ JOINT_COUNT}; // Joints of the creature

	const btVector3             m_positionOffset;           // Initial position
	btSoftRigidDynamicsWorld *  m_ownerWorld;				// The physics world of the simulation
	btCollisionShape		 *	m_shapes[BODYPART_COUNT];	// The primitive shape of each body part used in collision
	btRigidBody				 *	m_bodies[BODYPART_COUNT];	// The array of body parts
	btTypedConstraint		 *	m_joints[JOINT_COUNT];		// The type of each joint constraint
	Environment				 *  m_environment;              // Environmental Forces
	Cape					 *  m_cape;                     // Destructolon wears a cape 

	int lastChange;										// Time of last change of balance controller
	int lastChangeFight;

	bool	m_hasFallen;		// Indicates if the creature has already fallen down

	btCollisionShape	*	m_COMShape;		// Shape for COM
	btRigidBody			*	m_COM;			// Body COM
	btVector3				m_positionCOM;	// Position COM
	btVector3 computeCenterOfMass();		// Compute the COM of the creature in world coordinate system

protected:
	void addCape();
	void removeCape();
};

#endif