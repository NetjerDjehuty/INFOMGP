#include "Environment.h"


// Environmental Forces
Environment::Environment(btSoftRigidDynamicsWorld* ownerWorld) : m_ownerWorld (ownerWorld) { // Constructor
	
	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
    m_softBodyWorldInfo.water_density	    =	0;
    m_softBodyWorldInfo.water_offset		=	0;
    m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
    m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

	// setup wind force
	m_windForce = btVector3(0, 0.5, 0);
	m_windVelocity = btVector3(0, 120, 150);

}

Environment::~Environment(){ // Destructor
    /* empty */
}

void Environment::resetScene() {
	m_softBodyWorldInfo.m_sparsesdf.Reset();
	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
    m_softBodyWorldInfo.water_density	    =	0;
    m_softBodyWorldInfo.water_offset		=	0;
    m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
    m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);
}

void Environment::update() {
	
	// update wind force
	// XXX TODO

	// update wind velocity
	m_windVelocity += m_windForce;
}