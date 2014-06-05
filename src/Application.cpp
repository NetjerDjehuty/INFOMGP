
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include <iostream>
#include <sstream>
#include <string>

#include "Application.h"
#include "Creature.h"
#include "Scene.h"

void Application::initPhysics() {
	
	// Setup the basic world
	// =====================
	setTexturing(true);
	setShadows(true);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

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
	btVector3 startOffset(0,0.55,0);
	resetScene(startOffset);
	clientResetScene();
	m_startTime = GetTickCount();
	setCameraDistance(1.5);
}

void Application::resetScene(const btVector3& startOffset) {
	if (m_creature != NULL) delete m_creature;
	m_creature = new Creature(m_dynamicsWorld, startOffset);
	if (m_scene != NULL) delete m_scene;
	m_scene = new Scene(m_dynamicsWorld);
	m_startTime = GetTickCount();
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
				m_creature->switchCOM();
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
	
}

void Application::update() {	

	// Do not update time if creature fallen
	if (!m_creature->hasFallen()) m_currentTime = GetTickCount();
	m_elapsedTime = (int)(((double) m_currentTime - m_startTime)/100.0);

	// Move the platform if not fallen
	m_scene->update( (!m_creature->hasFallen()) ? m_elapsedTime : -1, m_creature->getCOM());

	// Control the creature movements
	m_creature->update((int)(m_currentTime - m_startTime));

	// Display info
	DemoApplication::displayProfileString(10,20,"Q=quit E=reset R=platform T=ball Y=COM U=switch I=pause");

	// Display time elapsed
	std::ostringstream osstmp;
	osstmp << m_elapsedTime;
	std::string s_elapsedTime = osstmp.str();
	std::ostringstream oss;
	if (m_elapsedTime < 10)
		oss << "Time under balance: 0." << s_elapsedTime << " seconds";
	else
		oss << "Time under balance: " << s_elapsedTime.substr(0,s_elapsedTime.size()-1) << "." << s_elapsedTime.substr(s_elapsedTime.size()-1,s_elapsedTime.size()) << " seconds";
	DemoApplication::displayProfileString(10,40,const_cast<char*>(oss.str().c_str()));	

}
