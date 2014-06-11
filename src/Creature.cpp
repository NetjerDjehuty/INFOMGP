
#include "Creature.h"

// TO DEBUG
#include <iostream>
#include <fstream>

#define CONSTRAINT_DEBUG_SIZE 0.2f

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

Creature::Creature (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld), m_hasFallen(false), lastChange(0), m_showCOM(false) { // Constructor

	// Setup the rigid bodies
	// ======================

	// Setup the collision shapes
	m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(0.1),btScalar(0.025),btScalar(0.12)));
	m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
	m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.50));
	m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
	m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.40));
	m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));

	// Setup the body properties
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); // absolute initial starting position
	btTransform transform;

	// FOOT
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
	m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(5.0), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

	// LOWER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
	m_bodies[Creature::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Creature::BODYPART_LOWER_LEG]);

	// UPPER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
	m_bodies[Creature::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Creature::BODYPART_UPPER_LEG]);

	// Add damping to the rigid bodies
	for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
		m_bodies[i]->setDeactivationTime(btScalar(0.01));
		m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
	}
	m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(0.8), btScalar(0.01)); // Higher friction for foot

	// Setup the joint constraints
	// ===========================

	btHingeConstraint* hingeJoint;
	//FYI, another type of joint is for example: btConeTwistConstraint* coneJoint;
	btTransform localA, localB;

	// ANKLE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.025), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.25), btScalar(0.0)));
	hingeJoint =  new btHingeConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LOWER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Creature::JOINT_ANKLE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE], true);

	// KNEE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.25), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.20), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Creature::JOINT_KNEE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);

}

Creature::~Creature() { // Destructor
	// Remove all joint constraints
	for (int i = 0; i < Creature::JOINT_COUNT; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = NULL;
	}		
	// Remove all bodies and shapes
	for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
		m_ownerWorld->removeRigidBody(m_bodies[i]);			
		delete m_bodies[i]->getMotionState();
		delete m_bodies[i]; m_bodies[i] = NULL;
		delete m_shapes[i]; m_shapes[i] = NULL;
	}
	if (m_showCOM) {
		m_ownerWorld->removeRigidBody(m_COM);
		delete m_COM->getMotionState();
		delete m_COM; m_COM = NULL;
		delete m_COMShape; m_COMShape = NULL;
	}
}

void Creature::switchCOM() {
	m_showCOM = !m_showCOM;
	if (m_showCOM) {
		// Shape
		m_COMShape = new btSphereShape(btScalar(0.05));
		m_COMShape->setColor(btVector3(btScalar(0.6),btScalar(1.0),btScalar(0.6)));
		// Body
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		m_COM = m_ownerWorld->localCreateRigidBody(btScalar(0.0), transform, m_COMShape);
		m_COM->setCollisionFlags(m_COM->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_COM->setActivationState(DISABLE_DEACTIVATION);
	}
	else {
		m_ownerWorld->removeRigidBody(m_COM);
		delete m_COM->getMotionState();
		delete m_COM; m_COM = NULL;
		delete m_COMShape; m_COMShape = NULL;	
	}	
}

void Creature::update(int elapsedTime) {

	// BALANCE CONTROLLER
	// ==================

	// Step 1.1: Compute the COM in world coordinate system
	btVector3 comInWorld = computeCenterOfMass();
	m_positionCOM = comInWorld;
	if (m_showCOM) { // Visualize COM
		btTransform transform;
		m_COM->getMotionState()->getWorldTransform(transform);
		transform.setOrigin(comInWorld);
		m_COM->getMotionState()->setWorldTransform(transform);
	}

	// Step 1.2: Update pose only if creature did not fall
	if (m_hasFallen) {
		if (((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->getEnableAngularMotor()) { // ragdoll is fallen
			((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->enableMotor(false);
			((btHingeConstraint*)m_joints[Creature::JOINT_KNEE])->enableMotor(false);

			((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->setEnabled(false);
			((btHingeConstraint*)m_joints[Creature::JOINT_KNEE])->setEnabled(false);
		}
		return;
	}			

	if (elapsedTime - lastChange > 10) { // Update balance control only every 10 ms
		lastChange = elapsedTime;

		btVector3 COM, COA;
		COA = m_bodies[BODYPART_FOOT]->getCenterOfMassPosition();
		COM = computeCenterOfMass();

		btTransform trFoot = m_bodies[Creature::BODYPART_FOOT]->getWorldTransform();
		btQuaternion fRot = trFoot.getRotation();
		btScalar mag = sqrt(fRot.w()*fRot.w() + fRot.y()*fRot.y());
		btScalar yaw = 2*acos(fRot.w()/mag); //... I think...

		btTransform yawRot; yawRot.setIdentity();
		yawRot.setRotation(btQuaternion(yaw,0,0));
		yawRot = yawRot.inverse();

		COM = yawRot * COM;
		COA = yawRot * COA;

		((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->setMotorTarget( btScalar( COM.z() - COA.z() ) * 20.0f );
		((btHingeConstraint*)m_joints[Creature::JOINT_KNEE])->setMotorTarget( btScalar( COA.x() - COM.x() ) * 25.0f );

		//=================== TODO ===================//

		// Step 2: Describe the ground projected CSP in world coordinate system

		// ANKLE
		// -----

		// Step 3.1: Describe the ground projected CSP in foot coordinate system
		// Step 3.2: Describe the ground projected COM in foot coordinate system
		// Step 3.3: Calculate the balance error solveable by an ankle rotation (inverted pendulum model)
		// Step 3.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
		// (Conversion between error to torque/motor velocity done by gains in PD controller)

		// KNEE
		// ----

		// Step 4.1: Describe the ground projected CSP in lower leg coordinate system
		// Step 4.2: Describe the ground projected COM in lower leg coordinate system
		// Step 4.3: Calculate the balance error solveable by a knee rotation (inverted pendulum model)
		// Step 4.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
		// (Conversion between error to torque/motor velocity done by gains in PD controller)

		//===========================================//

	}
}

bool Creature::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // true if already down (cannot get back up here)
	if (m_bodies[BODYPART_LOWER_LEG]->getActivationState() == ISLAND_SLEEPING) m_hasFallen = true; // true if enters in sleeping mode
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() < 0.15) m_hasFallen = true; // true if a creature has fallen from platform
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	if (m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	return m_hasFallen;
}

btVector3 Creature::computeCenterOfMass() {

	btVector3 ret(0,0,0);
	float totMass = 0.0f;

	for (int i = 0; i < BODYPART_COUNT; i++)
	{
		totMass += (1.0f/m_bodies[i]->getInvMass());
		ret +=	m_bodies[i]->getCenterOfMassPosition() / m_bodies[i]->getInvMass();
	}

	//=================== TODO ==================//
	return ret/totMass;
	//===========================================//

}
