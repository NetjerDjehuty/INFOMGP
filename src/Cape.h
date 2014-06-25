#ifndef CAPE_H
#define CAPE_H

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "Environment.h"

// Cape

class Cape {

public:
	Cape (btSoftRigidDynamicsWorld* ownerWorld, Environment *environment); // Constructor
	virtual ~Cape(); // Destructor
	
	void bindRigidBody(btRigidBody *body);  // Bind the corners of the cape to some rigidbody
	void update();                          // Taking new environment forces into account

protected:
	btSoftRigidDynamicsWorld  *	  m_ownerWorld;	  // The physics world of the simulation
	Environment               *   m_environment;  // Calculating the wind force
	btSoftBody                *   m_softBody;     // Soft body representing the cape
};




#endif