
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include <iostream>
#include <sstream>
#include <string>

#include "Application.h"
#include "Creature.h"
//#include "destructulon.h"
#include "Scene.h"

void Application::initPhysics() {

	this->creatureCreated = false;
	
	// Setup the basic world
	// =====================
	setTexturing(true);
	setShadows(true);
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	// Setup Environmental Forces
	m_environment = new Environment((btSoftRigidDynamicsWorld*) m_dynamicsWorld);
    m_environment->m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
    m_environment->m_softBodyWorldInfo.m_broadphase = m_broadphase;
    m_environment->m_softBodyWorldInfo.m_sparsesdf.Initialize();

	// Setup a big ground box
	// ======================
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.0),btScalar(10.0),btScalar(200.0)));
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));
	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(groundShape);
	fixedGround->setWorldTransform(groundTransform);
	m_dynamicsWorld->addCollisionObject(fixedGround);

	// Init Scene
	// ==========

	m_destructulon = NULL;
	m_creature = NULL;
	m_destructulon2 = NULL;

	btVector3 startOffset(0,0.55,0);
	resetScene(startOffset);
	clientResetScene();
	m_startTime = GetTickCount();
	setCameraDistance(1.5);
}

void Application::resetScene(const btVector3& startOffset) {

	this->strtOffset = startOffset;

	if(m_destructulon != NULL)
	{
		delete m_destructulon;
		m_destructulon = NULL;
		if(m_destructulon == NULL)
			m_destructulon = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, startOffset, m_environment);
	}
	if(m_destructulon2 != NULL)
	{
		delete m_destructulon;
		m_destructulon = NULL;
		if(m_destructulon == NULL)
			m_destructulon = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, btVector3(0, 0.55, 0.25), m_environment);

		delete m_destructulon2;
		m_destructulon2 = NULL;
		if(m_destructulon2 == NULL)
			m_destructulon2 = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, btVector3(0, 0.55, -0.25), m_environment);

		if(m_destructulon != NULL)
			m_destructulon2->opponent = m_destructulon;
		if(m_destructulon2 != NULL)
			m_destructulon->opponent = m_destructulon2;
	}
	if(m_creature != NULL)
	{
		delete m_creature;
		m_creature = NULL;
		if(m_creature == NULL)
			m_creature = new Creature(m_dynamicsWorld, startOffset);
	}



	if (m_scene != NULL) delete m_scene;
	m_scene = new Scene(m_dynamicsWorld);
	m_startTime = GetTickCount();

	// reset environmental forces
	m_environment->resetScene();
}

void Application::clientMoveAndDisplay() {

	// Update the simulator
	// ====================
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	float ms = getDeltaTimeMicroseconds();
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS) ms = minFPS;
	if (m_dynamicsWorld) {
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		m_dynamicsWorld->debugDrawWorld();
	}

	// Update the Scene
	// ================
	update();

	// Garbage collecting Environmental Forces
	if (m_environment)
		m_environment->m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

	// Render the simulation
	// =====================
	renderme(); 
	glFlush();
	glutSwapBuffers();
}

void Application::displayCallback() {

	// Render the simulation
	// =====================
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	renderme();
	if (m_dynamicsWorld) m_dynamicsWorld->debugDrawWorld();
	glFlush();
	glutSwapBuffers();
}

void Application::keyboardCallback(unsigned char key, int x, int y) {
	// You can add your key bindings here.
	// Be careful to not use a key already listed in DemoApplication::keyboardCallback
	switch (key) {
	case 'a': // move left arm for player 1
		{
			if (m_creature != NULL)
			{
				delete m_creature;
				if(m_destructulon != NULL)
					if(m_destructulon->opponent != NULL)
						m_destructulon->opponent = NULL;
				if(m_destructulon2 != NULL)
					if(m_destructulon2->opponent != NULL)
						m_destructulon2->opponent = NULL;
				m_creature = NULL;
			}
			if (m_destructulon2 != NULL)
			{
				delete m_destructulon2;
				if(m_destructulon2->opponent != NULL)
					m_destructulon2->opponent = NULL;
				m_destructulon2 = NULL;
			}
			if(m_destructulon != NULL)
			{
				delete m_destructulon;
				if(m_destructulon->opponent != NULL)
					m_destructulon->opponent = NULL;
				m_destructulon = NULL;
			}
			m_creature = new Creature(m_dynamicsWorld, this->strtOffset);
			Application::resetScene(this->strtOffset);
			break;
		}
	case 'd' : // Create destructulon and delete Creature
		{
			if (m_destructulon != NULL)
			{
				delete m_destructulon;
				m_destructulon = NULL;
			}
			if (m_destructulon2 != NULL)
			{
				delete m_destructulon2;
				m_destructulon2 = NULL;
			}
			if (m_creature != NULL)
			{
				delete m_creature;
				m_creature = NULL;
			}
			m_destructulon = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, this->strtOffset, m_environment);
			Application::resetScene(this->strtOffset);
			break;
		}
	case 'f':
		{
			if (m_destructulon != NULL)
			{
				delete m_destructulon;
				m_destructulon = NULL;
			}
			if (m_destructulon2 != NULL)
			{
				delete m_destructulon2;
				m_destructulon2 = NULL;
			}
			if (m_creature != NULL)
			{
				delete m_creature;
				m_creature = NULL;
			}
			m_destructulon = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, btVector3(0, 0.55, 0.25), m_environment);
			m_destructulon2 = new Destructulon((btSoftRigidDynamicsWorld*)m_dynamicsWorld, btVector3(0, 0.55, -0.25), m_environment);
			m_destructulon->opponent = m_destructulon2;
			m_destructulon2->opponent = m_destructulon;
			//Application::resetScene(this->strtOffset);
			break;
		}
	case 'e':
		{
			btVector3 startOffset(0,0.55,0);
			resetScene(startOffset);
			break;
		}
	case 'r':
		{
			m_scene->switchPlatform();
			break;
		}
	case 't':
		{
			m_scene->switchBall();
			break;
		}
	case 'y':
		{
			if(m_creature != NULL)
				m_creature->switchCOM();
			else if (m_destructulon != NULL)
				m_destructulon->switchCOM();
			else if (m_destructulon2 != NULL)
				m_destructulon2->switchCOM();
			break;
		}
	case 'c':
		{
			m_scene->shootCannon();
			break;
		}
	case 'p':
		{
			// toggle cape
			if(m_destructulon != NULL) m_destructulon->toggleCape();
			if(m_destructulon2 != NULL) m_destructulon2->toggleCape();
			break;
		}
	default :
		DemoApplication::keyboardCallback(key, x, y);
	}	
}

void Application::exitPhysics() {
	delete m_creature;
	delete m_scene;
	//remove the rigidbodies from the dynamics world and delete them	
	for (int i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) delete body->getMotionState();
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
	//delete ground
	delete m_ground;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	//delete collision configuration
	delete m_collisionConfiguration;

	//delete environment
	delete m_environment;
}

void Application::update() {	

	if(m_creature != NULL)
	{
		// Always draw the COM
		if(!m_creature->m_showCOM)
			m_creature->switchCOM();

		// Do not update time if creature fallen
		if (!m_creature->hasFallen()) m_currentTime = GetTickCount();
		m_elapsedTime = (int)(((double) m_currentTime - m_startTime)/100.0);

		// Move the platform if not fallen
		m_scene->update( (!m_creature->hasFallen()) ? m_elapsedTime : -1, m_creature->getCOM());

		// Control the creature movements
		m_creature->update((int)(m_currentTime - m_startTime));
	}

	if(m_destructulon != NULL)
	{
		if(m_scene->m_ball != NULL)
			m_destructulon->m_ball = m_scene->m_ball;

		// Always draw the COM
		if(!m_destructulon->m_showCOM)
			m_destructulon->switchCOM();

		// Do not update time if creature fallen
		if (!m_destructulon->hasFallen()) m_currentTime = GetTickCount();
		m_elapsedTime = (int)(((double) m_currentTime - m_startTime)/100.0);

		// Move the platform if not fallen
		m_scene->update( (!m_destructulon->hasFallen()) ? m_elapsedTime : -1, m_destructulon->getCOM());

		// Control the creature movements
		m_destructulon->update((int)(m_currentTime - m_startTime));
	}

	if(m_destructulon2 != NULL)
	{
		if(m_scene->m_ball != NULL)
			m_destructulon2->m_ball = m_scene->m_ball;

		// Always draw the COM
		if(!m_destructulon2->m_showCOM)
			m_destructulon2->switchCOM();

		// Do not update time if creature fallen
		if (!m_destructulon2->hasFallen()) m_currentTime = GetTickCount();
		m_elapsedTime = (int)(((double) m_currentTime - m_startTime)/100.0);

		// Move the platform if not fallen
		m_scene->update( (!m_destructulon2->hasFallen()) ? m_elapsedTime : -1, m_destructulon2->getCOM());

		// Control the creature movements
		m_destructulon2->update((int)(m_currentTime - m_startTime));
	}

	// Update environment
	m_environment->update();

	// Display info
	DemoApplication::displayProfileString(10,20,"Q=quit E=reset R=platform T=ball Y=COM U=switch I=pause");
	DemoApplication::displayProfileString(10,40,"A=Creature D=destructulon F=fight C=canon P=cape");


	// Display time elapsed
	std::ostringstream osstmp;
	osstmp << m_elapsedTime;
	std::string s_elapsedTime = osstmp.str();
	std::ostringstream oss;
	if (m_elapsedTime < 10)
		oss << "Time under balance: 0." << s_elapsedTime << " seconds";
	else
		oss << "Time under balance: " << s_elapsedTime.substr(0,s_elapsedTime.size()-1) << "." << s_elapsedTime.substr(s_elapsedTime.size()-1,s_elapsedTime.size()) << " seconds";
	DemoApplication::displayProfileString(10,60,const_cast<char*>(oss.str().c_str()));	

	// Display wind
	std::ostringstream ss; 
	ss << "Wind: " << m_environment->m_windVelocity.z();
	DemoApplication::displayProfileString(10,80,const_cast<char*>(ss.str().c_str()));
}


// overwrite default GLUT renderme
// to also draw the softbodies
void Application::renderme() {

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	//m_dynamicsWorld->debugDrawWorld();

	btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	//btIDebugDraw*	sdraw = softWorld ->getDebugDrawer();
	
	for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
	{
		btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
		if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
		}
	}

	GlutDemoApplication::renderme();
}