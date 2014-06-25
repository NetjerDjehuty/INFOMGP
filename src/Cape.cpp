#include "Cape.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"


// CAPE

Cape::Cape (btSoftRigidDynamicsWorld* ownerWorld, Environment *environment) : m_ownerWorld (ownerWorld), m_environment (environment) { // Constructor
 
	const btScalar	s=0.1;
	const btScalar	h=1.35;
	const int		r=20;
	
	int fixed=0;//4+8;
	btSoftBody*	psb = btSoftBodyHelpers::CreatePatch(
		m_environment->m_softBodyWorldInfo,
		btVector3(-s,0,-s),
		btVector3(+s,0,-s),
		btVector3(-s,0,+s*3),
		btVector3(+s,0,+s*3),
		r,r,
		fixed,
		true);
	
	psb->m_cfg.kLF			=	0.05;
	psb->m_cfg.kDG			=	0.001;
	psb->m_cfg.kAHR         =   1;
	psb->m_cfg.kDF          =   0;
	psb->m_cfg.piterations  =   10;
	
	psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	psb->setWindVelocity(btVector3(0, 115, 125.0));
	

	btTransform		trs;
	btQuaternion	rot;
	btVector3 pos(0.01, h, 0);
	rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs.setIdentity();
	trs.setOrigin(pos);
	trs.setRotation(rot);
	psb->transform(trs);
	psb->setTotalMass(0.1);

	psb->getCollisionShape()->setMargin(0.05);

	m_ownerWorld->addSoftBody(psb);
	m_softBody = psb;
}

Cape::~Cape(){ // Destructor
    
	m_ownerWorld->removeSoftBody(m_softBody);
	delete m_softBody;

}


void Cape::bindRigidBody(btRigidBody *body) {

	m_softBody->appendAnchor(0,body);
	m_softBody->appendAnchor(18,body);

}

