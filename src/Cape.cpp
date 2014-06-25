#include "Cape.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;


// CAPE

Cape::Cape (btSoftRigidDynamicsWorld* ownerWorld, Environment *environment) : m_ownerWorld (ownerWorld), m_environment (environment) { // Constructor
 
	const btScalar	s=0.1;
	const btScalar	h=1.5;
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
	
	
	//psb->m_cfg.piterations = 10;
	//psb->m_cfg.citerations = 10;
	//psb->m_cfg.diterations = 10;

	//psb->appendAnchor(0,body);
	//psb->appendAnchor(-(r-1),body);


	//psb->getCollisionShape()->setMargin(0.5);
	//btSoftBody::Material* pm=psb->appendMaterial();
	//pm->m_kLST		=	0.0004;
	//pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	//psb->generateBendingConstraints(2,pm);
	//psb->m_cfg.kLF			=	0.05;
	//psb->m_cfg.kDG			=	0.01;
	psb->m_cfg.piterations = 10;
	//	psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	
	//psb->setWindVelocity(btVector3(4, -10, -25.0));
	

	btTransform		trs;
	btQuaternion	rot;
	btVector3 pos(0.01, h, 0.1);
	rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs.setIdentity();
	trs.setOrigin(pos);
	trs.setRotation(rot);
	psb->transform(trs);
	psb->setTotalMass(0.000001);


	m_ownerWorld->addSoftBody(psb);
	m_softBody = psb;

	//cout << "add cape";
}

Cape::~Cape(){ // Destructor
    
	m_ownerWorld->removeSoftBody(m_softBody);
	delete m_softBody;

}


void Cape::bindRigidBody(btRigidBody *body1, btRigidBody *body2) {

	m_softBody->appendAnchor(0,body1);
	m_softBody->appendAnchor(18,body2);
	//m_softBody->appendAnchor(-(r-1),body);

}

