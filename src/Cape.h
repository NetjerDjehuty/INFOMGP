#ifndef CAPE_H
#define CAPE_H

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "Environment.h"

// Cape

class Cape {

public:
	Cape (btSoftRigidDynamicsWorld* ownerWorld, Environment *environment); // Constructor
	virtual ~Cape(); // Destructor
	
	void bindRigidBody(btRigidBody *body);
	void update();

protected:
	btSoftRigidDynamicsWorld  *	m_ownerWorld;				// The physics world of the simulation
	Environment               *   m_environment;
	btSoftBody                *   m_softBody;     // soft body of the cape
};




#endif