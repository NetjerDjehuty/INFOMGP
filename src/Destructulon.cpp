#include "Destructulon.h"

// TO DEBUG
#include <iostream>
#include <fstream>

#define CONSTRAINT_DEBUG_SIZE 0.2f

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

Destructulon::Destructulon(btSoftRigidDynamicsWorld* ownerWorld, const btVector3& positionOffset, Environment* environment) : m_ownerWorld (ownerWorld), m_environment(environment), m_hasFallen(false), lastChange(0), m_showCOM(false), firstLoop(true)
{
	this->m_ball = NULL;
	this->opponent = NULL;
	name = "Destructulon";

#pragma region

	m_shapes[Destructulon::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(0.1),btScalar(0.025),btScalar(0.12)));
	m_shapes[Destructulon::BODYPART_FOOT]->setColor(btVector3(btScalar(0.0),btScalar(0.0),btScalar(0.0)));
	m_shapes[Destructulon::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.50));
	m_shapes[Destructulon::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.0),btScalar(0.0),btScalar(0.0)));
	m_shapes[Destructulon::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.40));
	m_shapes[Destructulon::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.0),btScalar(0.0),btScalar(0.0)));
	m_shapes[Destructulon::BODYPART_UPPER_ARM] = new btCapsuleShape(btScalar(0.03), btScalar(0.6));
	m_shapes[Destructulon::BODYPART_UPPER_ARM]->setColor(btVector3(1,0,0));/*
																		   m_shapes[Destructulon::BODYPART_LOWER_ARM] = new btCapsuleShape(btScalar(0.03), btScalar(0.30));
																		   m_shapes[Destructulon::BODYPART_LOWER_ARM]->setColor(btVector3(1,1,1));*/
	m_shapes[Destructulon::BODYPART_UPPER_L_ARM] = new btCapsuleShape(btScalar(0.03), btScalar(0.6));
	m_shapes[Destructulon::BODYPART_UPPER_L_ARM]->setColor(btVector3(0,1,0));/*
																			 m_shapes[Destructulon::BODYPART_LOWER_L_ARM] = new btCapsuleShape(btScalar(0.03), btScalar(0.3));
																			 m_shapes[Destructulon::BODYPART_LOWER_L_ARM]->setColor(btVector3(1,1,1));*/
	/*m_shapes[Destructulon::BODYPART_HEAD] = new btSphereShape(0.3);
	m_shapes[Destructulon::BODYPART_HEAD]->setColor(btVector3(0.0,0.0,0.0));*/
	
#pragma endregion Setup the collision shapes

	// Setup the body properties
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); // absolute initial starting position
	btTransform transform;

#pragma region 

	// FOOT
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(115.0), offset*transform, m_shapes[Destructulon::BODYPART_FOOT]);

	// LOWER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Destructulon::BODYPART_LOWER_LEG]);

	// UPPER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Destructulon::BODYPART_UPPER_LEG]);
	
	// UPPER_ARM
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.1), btScalar(0.8), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_UPPER_ARM] = m_ownerWorld->localCreateRigidBody(btScalar(1.0), offset*transform, m_shapes[Destructulon::BODYPART_UPPER_ARM]);

	/*// LOWER_ARM
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.1), btScalar(0.375), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_LOWER_ARM] = m_ownerWorld->localCreateRigidBody(btScalar(1.0), offset*transform, m_shapes[Destructulon::BODYPART_LOWER_ARM]);*/

	// UPPER_L_ARM
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.1), btScalar(0.8), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_UPPER_L_ARM] = m_ownerWorld->localCreateRigidBody(btScalar(1.0), offset*transform, m_shapes[Destructulon::BODYPART_UPPER_L_ARM]);

	/*// LOWER_L_ARM
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.1), btScalar(0.375), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_LOWER_L_ARM] = m_ownerWorld->localCreateRigidBody(btScalar(1.0), offset*transform, m_shapes[Destructulon::BODYPART_LOWER_L_ARM]);*/

	/*
	// HEAD
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.8), btScalar(0.0)));
	m_bodies[Destructulon::BODYPART_HEAD] = m_ownerWorld->localCreateRigidBody(btScalar(5.0), offset*transform, m_shapes[Destructulon::BODYPART_HEAD]);
	*/

#pragma endregion initialization of bodyparts

	// Add damping to the rigid bodies
	for (int i = 0; i < Destructulon::BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
		m_bodies[i]->setDeactivationTime(btScalar(0.01));
		m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
	}
	m_bodies[Destructulon::BODYPART_FOOT]->setDamping(btScalar(0.8), btScalar(0.01)); // Higher friction for foot

	// Add damping to the rigid bodies
	for (int i = 0; i < Destructulon::BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
		m_bodies[i]->setDeactivationTime(btScalar(0.01));
		m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
	}
	m_bodies[Destructulon::BODYPART_FOOT]->setDamping(btScalar(0.8), btScalar(0.01)); // Higher friction for foot
	btHingeConstraint* hingeJoint; btConeTwistConstraint*  socketJoint;
	//btPoint2PointConstraint* socketJoint;
	//FYI, another type of joint is for example: btConeTwistConstraint* coneJoint;
	btTransform localA, localB;

#pragma region

	// ANKLE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.025), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.25), btScalar(0.0)));
	hingeJoint =  new btHingeConstraint(*m_bodies[Destructulon::BODYPART_FOOT], *m_bodies[Destructulon::BODYPART_LOWER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Destructulon::JOINT_ANKLE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[Destructulon::JOINT_ANKLE], true);

	// KNEE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.25), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.20), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Destructulon::BODYPART_LOWER_LEG], *m_bodies[Destructulon::BODYPART_UPPER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Destructulon::JOINT_KNEE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);

	//////
	////// m_shapes[Destructulon::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.40));
	////// m_shapes[Destructulon::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.0),btScalar(0.0),btScalar(0.0)));
	//////

	// SHOULDER
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(-0.075), btScalar(.15), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(0.3), btScalar(0.0)));
	//old stuff: socketJoint = new btPoint2PointConstraint(*m_bodies[BODYPART_UPPER_LEG], *m_bodies[Destructulon::BODYPART_UPPER_ARM], btVector3(-.06,0.09,0.0), btVector3(0.0,0.0,0.0));
	socketJoint = new btConeTwistConstraint(*m_bodies[BODYPART_UPPER_LEG], *m_bodies[Destructulon::BODYPART_UPPER_ARM], localA, localB);

	socketJoint ->enableMotor(true);
	socketJoint ->setLimit(M_PI_2, M_PI_2, 0);
	socketJoint ->setMaxMotorImpulse(btScalar(50.0));

	m_joints[Destructulon::JOINT_SHOULDER] = socketJoint;
	socketJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_SHOULDER], true);

	/*
	// ELBOW
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(0.15), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Destructulon::BODYPART_UPPER_ARM], *m_bodies[Destructulon::BODYPART_LOWER_ARM], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Destructulon::JOINT_ELBOW] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_ELBOW], true);*/

	// SHOULDER_L
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.075), btScalar(.15), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(0.3), btScalar(0.0)));
	socketJoint = new btConeTwistConstraint(*m_bodies[BODYPART_UPPER_LEG], *m_bodies[Destructulon::BODYPART_UPPER_L_ARM], localA, localB);
	//old stuff: socketJoint = new btPoint2PointConstraint(*m_bodies[BODYPART_UPPER_LEG], *m_bodies[Destructulon::BODYPART_UPPER_L_ARM], btVector3(0.06,.09,0.0), btVector3(0.0,0.0,0.0));

	socketJoint ->enableMotor(true);
	socketJoint->setLimit(M_PI_2, M_PI_2, 0);
	socketJoint ->setMaxMotorImpulse(btScalar(50.0));

	m_joints[Destructulon::JOINT_L_SHOULDER] = socketJoint;
	socketJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_L_SHOULDER], true);

	/*
	// ELBOW_L
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(0.15), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Destructulon::BODYPART_UPPER_L_ARM], *m_bodies[Destructulon::BODYPART_LOWER_L_ARM], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Destructulon::JOINT_L_ELBOW] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_L_ELBOW], true);*/

	/*
	// NECK
	socketJoint = new btPoint2PointConstraint(*m_bodies[Destructulon::BODYPART_HEAD], *m_bodies[Destructulon::BODYPART_UPPER_LEG], btVector3(0.0,0.0,0.0), btVector3(0.0,0.5,0.0));

	m_joints[Destructulon::JOINT_NECK] = socketJoint;
	socketJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_NECK], true);
	*/

#pragma endregion intialization of joints

	// Initialize Cape
	m_cape = new Cape(m_ownerWorld, m_environment, positionOffset);
	m_cape->bindRigidBody(m_bodies[Destructulon::BODYPART_UPPER_LEG]);
}

Destructulon::~Destructulon(){ // Destructor
	// Remove all joint constraints
	for (int i = 0; i < Destructulon::JOINT_COUNT; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = NULL;
	}		
	// Remove all bodies and shapes
	for (int i = 0; i < Destructulon::BODYPART_COUNT; ++i) {
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
	// Delete Cape
	if(m_cape) delete m_cape;
}

void Destructulon::switchCOM() {
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

void Destructulon::update(int elapsedTime) {

	// update cape
	if(m_cape)
		m_cape->update();

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

	// Step 1.2: Update pose only if Destructulon did not fall
	if (m_hasFallen) {
		if (((btHingeConstraint*)m_joints[Destructulon::JOINT_ANKLE])->getEnableAngularMotor()) { // ragdoll is fallen

			for(int i = 0; i < Destructulon::JOINT_COUNT; i++)
			{
				m_joints[i]->setEnabled(false);
			}
			((btHingeConstraint*)m_joints[Destructulon::JOINT_ANKLE])->enableMotor(false);
			((btHingeConstraint*)m_joints[Destructulon::JOINT_KNEE])->enableMotor(false);
			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])->enableMotor(false);
			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])->enableMotor(false);
		}
		return;
	}			

	if (elapsedTime - lastChange > 10) { // Update balance control only every 10 ms
		lastChange = elapsedTime;

		btVector3 COM, COA;
		COA = m_bodies[BODYPART_FOOT]->getCenterOfMassPosition();
		COM = computeCenterOfMass();

		btTransform trFoot = m_bodies[Destructulon::BODYPART_FOOT]->getWorldTransform();
		btQuaternion fRot = trFoot.getRotation();
		btScalar mag = sqrt(fRot.w()*fRot.w() + fRot.y()*fRot.y());
		btScalar yaw = 2*acos(fRot.w()/mag); //... I think...

		btTransform yawRot; yawRot.setIdentity();
		yawRot.setRotation(btQuaternion(yaw,0,0));
		yawRot = yawRot.inverse();

		COM = yawRot * COM;
		COA = yawRot * COA;

		((btHingeConstraint*)m_joints[Destructulon::JOINT_ANKLE])->setMotorTarget( btScalar( COM.z() - COA.z() ) * 20.0f );
		((btHingeConstraint*)m_joints[Destructulon::JOINT_KNEE])->setMotorTarget( btScalar( COA.x() - COM.x() ) * 25.0f );

		if(firstLoop)
		{
			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])-> setMotorTarget( btQuaternion( btScalar(0.0),  btScalar( COA.z() - COM.z() ) * 10.0f ,  btScalar( COA.x() - COM.x() ) * 15.0f )); // last one: positive is right, negative left
			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])-> setMotorTarget( btQuaternion( btScalar(0.0) ,  btScalar( COM.z() - COA.z() ) * 10.0f ,  btScalar( COM.x() - COA.x() ) * 15.0f ));
			firstLoop = false;
		}

#pragma region
		if(this->opponent != NULL)
		{
			btVector3 opponentPos = this->opponent->m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition();
			btVector3 arm = this->m_bodies[BODYPART_UPPER_ARM]->getCenterOfMassPosition();
			btVector3 up = btVector3(0, 0.3, 0);
			btTransform armOrient = m_bodies[BODYPART_UPPER_ARM]->getWorldTransform();
			btVector3 shoulder = armOrient * up;

			btVector3 shoulderToOpponent = shoulder - opponentPos;
			btVector3 shoulderToArm = shoulder - arm;

			btVector3 temp = btCross(shoulderToOpponent,shoulderToArm);

			btScalar w = sqrt((shoulderToOpponent.length() * shoulderToOpponent.length()) * (shoulderToArm.length() * shoulderToArm.length())) + btDot(shoulderToArm, shoulderToOpponent);


			btQuaternion Quat = btQuaternion(temp.getX(), temp.getY(), temp.getZ(), w);

			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])-> setMotorTarget(Quat.normalize());
			//((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])->setMaxMotorImpulse(btScalar(90));
			((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])-> setMotorTarget(Quat.normalize());
			//((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])->setMaxMotorImpulse(btScalar(90));
		}
#pragma endregion  BATTLE!

#pragma region
		if(this->m_ball != NULL)
		{
			btVector3 ballPos = this->m_ball->getCenterOfMassPosition();
			btScalar threshold = 3;
			if(ballPos.distance2(m_bodies[BODYPART_UPPER_ARM]->getCenterOfMassPosition()) < threshold)
			{
				btVector3 ballPos = this->m_ball->getCenterOfMassPosition();
				btVector3 arm = this->m_bodies[BODYPART_UPPER_ARM]->getCenterOfMassPosition();
				btVector3 up = btVector3(0, 0.3, 0);
				btTransform armOrient = m_bodies[BODYPART_UPPER_ARM]->getWorldTransform();
				btVector3 shoulder = armOrient * up;

				btVector3 shoulderToBall = shoulder - ballPos;
				btVector3 shoulderToArm = shoulder - arm;

				btVector3 temp = btCross(shoulderToBall, shoulderToArm);

				btScalar w = sqrt((shoulderToBall.length() * shoulderToBall.length()) * (shoulderToArm.length() * shoulderToArm.length())) + btDot(shoulderToBall,shoulderToArm);


				btQuaternion Quat = btQuaternion(temp.getX(), temp.getY(), temp.getZ(), w);

				if(temp.getZ() > -.1)
				{
					((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])-> setMotorTarget(Quat.normalize());
					((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])->setMaxMotorImpulse(btScalar(90));
				}
				else if(temp.getZ() < 0.1)
				{
					((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])-> setMotorTarget(Quat.normalize());
					((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])->setMaxMotorImpulse(btScalar(90));
				}
			}
			else
			{
				((btConeTwistConstraint*)m_joints[Destructulon::JOINT_SHOULDER])-> setMotorTarget( btQuaternion( btScalar(0.0),  btScalar( COA.z() - COM.z() ) * 10.0f ,  btScalar( COA.x() - COM.x() ) * 15.0f )); // last one: positive is right, negative left
				((btConeTwistConstraint*)m_joints[Destructulon::JOINT_L_SHOULDER])-> setMotorTarget( btQuaternion( btScalar(0.0) ,  btScalar( COM.z() - COA.z() ) * 10.0f ,  btScalar( COM.x() - COA.x() ) * 15.0f ));
			}
		}
#pragma endregion BALL SLAPPER

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

bool Destructulon::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // true if already down (cannot get back up here)
	if (m_bodies[BODYPART_LOWER_LEG]->getActivationState() == ISLAND_SLEEPING) m_hasFallen = true; // true if enters in sleeping mode
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() < 0.15) m_hasFallen = true; // true if a Destructulon has fallen from platform
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	if (m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	return m_hasFallen;
}

btVector3 Destructulon::computeCenterOfMass() {

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