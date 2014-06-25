#include "Environment.h"
#include <math.h>       /* sin */
#define PI 3.14159265

// Environmental Forces
Environment::Environment(btSoftRigidDynamicsWorld* ownerWorld) : m_ownerWorld (ownerWorld) { // Constructor
	
	m_softBodyWorldInfo.air_density		    =	(btScalar)1.2;
    m_softBodyWorldInfo.water_density	    =	0;
    m_softBodyWorldInfo.water_offset		=	0;
    m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
    m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	// setup wind force
	m_timer = 0.0;
	m_windVelocity = btVector3(0, 0, 0);

}

Environment::~Environment(){ // Destructor
    /* empty */
}

void Environment::resetScene() {
	
	// reset to default configuration
	m_softBodyWorldInfo.m_sparsesdf.Reset();
	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
    m_softBodyWorldInfo.water_density	    =	0;
    m_softBodyWorldInfo.water_offset		=	0;
    m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
    m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
}

void Environment::update() {
	
	// calculate wind strength
	m_timer += 0.01f;
	if (m_timer > 2*PI)
		m_timer = 0.0;
	float strength = sin(m_timer);

	// update wind velocity
	m_windVelocity = btVector3(0, strength * 50 + 120, strength * 50 + 120);
}