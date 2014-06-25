#include "Cape.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"


// CAPE

Cape::Cape (btSoftRigidDynamicsWorld* ownerWorld, Environment *environment, const btVector3& offset) : m_ownerWorld (ownerWorld), m_environment (environment) { // Constructor
 
	const btScalar	halfWidth=0.1f;
	const btScalar  halfHeight=halfWidth*2;
	const int		resolution=20;
	btVector3 pos(0.01f, 1.25f, offset.z());
	
	m_softBody = btSoftBodyHelpers::CreatePatch(
		m_environment->m_softBodyWorldInfo,
		btVector3(-halfWidth,0,-halfHeight),
		btVector3(+halfWidth,0,-halfHeight),
		btVector3(-halfWidth,0,+halfHeight),
		btVector3(+halfWidth,0,+halfHeight),
		resolution, resolution,
		0, //4+8
		true);
	
	// soft body configuration
	m_softBody->m_cfg.kLF		   =   0.05f;
	m_softBody->m_cfg.kDG		   =   0.001f;
	m_softBody->m_cfg.kAHR         =   1.0f;
	m_softBody->m_cfg.kDF          =   0.0f;
	m_softBody->m_cfg.piterations  =   10;
	m_softBody->m_cfg.aeromodel	   =   btSoftBody::eAeroModel::V_TwoSidedLiftDrag;
	
	// positioning
	btTransform		trs;
	btQuaternion	rot;
	rot.setRotation(btVector3(1, 0, 0), btScalar(SIMD_PI/2));
	trs.setIdentity();
	trs.setOrigin(pos);
	trs.setRotation(rot);
	m_softBody->transform(trs);
	m_softBody->setTotalMass(0.1f);

	// collission distance
	m_softBody->getCollisionShape()->setMargin(0.05f);

	// add to dynamicsworld
	m_ownerWorld->addSoftBody(m_softBody);
}

Cape::~Cape(){ // Destructor
    
	m_ownerWorld->removeSoftBody(m_softBody);
	delete m_softBody;

}

// bind cape to the given rigidbody
void Cape::bindRigidBody(btRigidBody *body) {

	m_softBody->appendAnchor(0,body);
	m_softBody->appendAnchor(18,body);

}

void Cape::update() {
	
	// set updated wind force
	m_softBody->setWindVelocity(m_environment->m_windVelocity);

}