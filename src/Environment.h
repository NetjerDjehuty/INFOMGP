#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"


// Environmental Forces

class Environment {

public:
	Environment(btSoftRigidDynamicsWorld* ownerWorld); // Constructor
	virtual	~Environment(); // Destructor

	void resetScene();
	void update();
    
	btSoftBodyWorldInfo	m_softBodyWorldInfo;

protected:
	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation

};


#endif