#include "Environment.h"


// Environmental Forces
Environment::Environment(btSoftRigidDynamicsWorld* ownerWorld) : m_ownerWorld (ownerWorld) { // Constructor
	
	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
    m_softBodyWorldInfo.water_density	    =	0;
    m_softBodyWorldInfo.water_offset		=	0;
    m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
    m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);

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
	/* empty */
}