#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"


// Environmental Forces (wind)

class Environment {

public:
	Environment(btSoftRigidDynamicsWorld* ownerWorld); // Constructor
	virtual	~Environment(); // Destructor

	void resetScene();  // Reset to default environment values
	void update();      // Update forces
    
	btSoftBodyWorldInfo	m_softBodyWorldInfo;  // Holding environment configuration
	btVector3 m_windVelocity;                 // Holding wind strength and direction

protected:
	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	float                   m_timer;                    // Timer used to dynamic change wind forces
};


#endif