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
	btVector3 m_windVelocity;

protected:
	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btVector3               m_windForce;
};


#endif