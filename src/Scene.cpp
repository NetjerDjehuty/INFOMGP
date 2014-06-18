
#include "Scene.h"
#include <time.h>

Scene::Scene (btDynamicsWorld* ownerWorld) : m_ownerWorld(ownerWorld), m_ball(NULL), m_cannonBall(NULL), m_PlatformActive(false), m_BallActive(false), m_CannonActive(false) {

	//Seed random generator
	srand(time(NULL));

	// Setup the collision shapes
	m_platform_shape = new btBoxShape(btVector3(btScalar(0.8),btScalar(0.02),btScalar(0.8)));
	m_platform_shape->setColor(btVector3(btScalar(0.3),btScalar(0.3),btScalar(1.0)));
	m_ball_shape = new btSphereShape(btScalar(0.1));
	m_ball_shape->setColor(btVector3(btScalar(1.0),btScalar(0.2),btScalar(0.2)));
	m_cannon_ball_shape = new btSphereShape(btScalar(0.05));
	m_cannon_ball_shape->setColor(btVector3(0,0,0));

	// Setup the body properties
	btTransform offset; offset.setIdentity();
	btTransform transform;

	// Platform
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.5), btScalar(0.0)));
	m_platform = m_ownerWorld->localCreateRigidBody(btScalar(0.0), offset*transform, m_platform_shape);
	m_platform->setCollisionFlags(m_platform->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	m_platform->setActivationState(DISABLE_DEACTIVATION);
	m_axisToRotatePlatform = rand() % 3;
	m_signToRotatePlatform = rand() % 2;
	m_magnitudeToRotatePlatform = btScalar(0.0);
	lastChangeRotation = 0;
	lastShoot = 0;
	lastCannon = -10;

	// Ball
	// Done when necessary
}

Scene::~Scene() {
	// Remove all bodies and shapes
	m_ownerWorld->removeRigidBody(m_platform);
	delete m_platform->getMotionState();
	delete m_platform;
	if (m_ball != NULL) {
		m_ownerWorld->removeRigidBody(m_ball);
		delete m_ball->getMotionState();
		delete m_ball;
		m_ball = NULL;
	}
	if (m_cannonBall != NULL) {
		m_ownerWorld->removeRigidBody(m_cannonBall);
		delete m_cannonBall->getMotionState();
		delete m_cannonBall;
		m_cannonBall = NULL;
	}
	delete m_platform_shape;
	delete m_ball_shape;
}

void Scene::update(int elapsedTime, const btVector3& creatureCOM) {

	// If creature has fallen, do not move platform or shoot ball anymore
	if (elapsedTime == -1) {
		lastChangeRotation = 0;
		lastShoot = 0;
		return;
	}

	// PLATFORM
	// ========

	if (m_PlatformActive) {

		// Current Transform
		btTransform transform;
		m_platform->getMotionState()->getWorldTransform(transform);
		btQuaternion currentRotation = transform.getRotation();

		// Change direction every second
		if ( elapsedTime - lastChangeRotation > 10) {
			lastChangeRotation = elapsedTime;
			m_axisToRotatePlatform = rand() % 3;
			m_signToRotatePlatform = rand() % 2;
			m_magnitudeToRotatePlatform = btScalar(0.00001 * (rand() % elapsedTime));
		}

		// Apply rotation
		btQuaternion appliedRotation (btVector3( btScalar((m_axisToRotatePlatform==0) ? 1 : 0) , btScalar((m_axisToRotatePlatform==1) ? 1 : 0) , btScalar((m_axisToRotatePlatform==2) ? 1 : 0 )), m_magnitudeToRotatePlatform * btScalar(((m_signToRotatePlatform==0) ? 1 : -1) ));
		//appliedRotation *= btQuaternion(btVector3(0.0f, 1.0f, 0.0f), 0.0001f);
		//btQuaternion appliedRotation = btQuaternion(btVector3(0.0f, 1.0f, 0.0f), 0.0001f);
		currentRotation *= appliedRotation;

		if (abs(currentRotation.getAngle()) < 0.25) {
			transform.setRotation(currentRotation);
			m_platform->getMotionState()->setWorldTransform(transform);
		}
	}

	// BALL
	// ====

	if (m_BallActive) {

		// Shoot a ball every four seconds
		if ( elapsedTime - lastShoot > 40) {

			// Delete old ball
			if (m_ball != NULL) {		
				m_ownerWorld->removeRigidBody(m_ball);
				delete m_ball->getMotionState();
				delete m_ball;
				m_ball = NULL;
			}

			// Decide new shooting position
			lastShoot = elapsedTime;
			btTransform startTransform;
			startTransform.setIdentity();
			int shootDirection = rand() % 4;
			btVector3 shootFrom;
			switch (shootDirection) {
			case 0 : { shootFrom = btVector3(btScalar(-0.8),btScalar(1.2),btScalar(0.0)); break; }
			case 1 : { shootFrom = btVector3(btScalar(0.8),btScalar(1.2),btScalar(0.0)); break; }
			case 2 : { shootFrom = btVector3(btScalar(0.0),btScalar(1.2),btScalar(0.8)); break; }
			case 3 : { shootFrom = btVector3(btScalar(0.0),btScalar(1.2),btScalar(-0.8)); break; }
			default: break;	
			}
			startTransform.setOrigin(shootFrom);

			// Create new ball
			m_ball = m_ownerWorld->localCreateRigidBody(1.0,startTransform,m_ball_shape);
			m_ball->setLinearFactor(btVector3(1,1,1));
			btScalar height = btScalar(creatureCOM.getY() + 0.5  + ((rand() % 9) - 4) / 10.0);
			btVector3 linVel(creatureCOM.getX()-shootFrom[0],height-shootFrom[1],creatureCOM.getZ()-shootFrom[2]); linVel.normalize();
			linVel *= btScalar(3.5 + 0.001 * (rand() % elapsedTime));
			m_ball->getWorldTransform().setOrigin(shootFrom);
			m_ball->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
			m_ball->setLinearVelocity(linVel);
			m_ball->setAngularVelocity(btVector3(0,0,0));
			m_ball->setCcdMotionThreshold(0.5);
			m_ball->setCcdSweptSphereRadius(0.9f);
		}
	}

	m_CannonActive = m_CannonActive && (elapsedTime - lastCannon) > 5;

	if(m_CannonActive)
	{
		// Delete old ball
		if (m_cannonBall != NULL) {		
			m_ownerWorld->removeRigidBody(m_cannonBall);
			delete m_cannonBall->getMotionState();
			delete m_cannonBall;
			m_cannonBall = NULL;
		}

		// Decide new shooting position
		lastCannon = elapsedTime;
		btTransform startTransform;
		startTransform.setIdentity();
		int shootDirection = rand() % 4;
		btVector3 shootFrom;
		switch (shootDirection) {
		case 0 : { shootFrom = btVector3(btScalar(-0.8),btScalar(1.2),btScalar(0.0)); break; }
		case 1 : { shootFrom = btVector3(btScalar(0.8),btScalar(1.2),btScalar(0.0)); break; }
		case 2 : { shootFrom = btVector3(btScalar(0.0),btScalar(1.2),btScalar(0.8)); break; }
		case 3 : { shootFrom = btVector3(btScalar(0.0),btScalar(1.2),btScalar(-0.8)); break; }
		default: break;	
		}
		startTransform.setOrigin(shootFrom);

		// Create new ball
		m_cannonBall = m_ownerWorld->localCreateRigidBody(10.0,startTransform,m_cannon_ball_shape);
		m_cannonBall->setLinearFactor(btVector3(1,1,1));
		btScalar height = btScalar(creatureCOM.getY() + 0.5  + ((rand() % 9) - 4) / 10.0);
		btVector3 linVel(creatureCOM.getX()-shootFrom[0],height-shootFrom[1],creatureCOM.getZ()-shootFrom[2]); linVel.normalize();
		linVel *= btScalar(5.5 + 0.01 * (rand() % elapsedTime));
		m_cannonBall->getWorldTransform().setOrigin(shootFrom);
		m_cannonBall->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		m_cannonBall->setLinearVelocity(linVel);
		m_cannonBall->setAngularVelocity(btVector3(0,0,0));
		m_cannonBall->setCcdMotionThreshold(0.5);
		m_cannonBall->setCcdSweptSphereRadius(0.9f);
	}
}



