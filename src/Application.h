
#ifndef APPLICATION_H
#define APPLICATION_H

//------------- include/declaration -------------
#include "GlutDemoApplication.h"
#include "Destructulon.h"
class btBroadphaseInterface;
class btCollisionShape;
class btCollisionDispatcher;
class btConstraintSolver;
class btDefaultCollisionConfiguration;

// User include
class Creature;
class Scene;
//-----------------------------------------------

class Application : public GlutDemoApplication {

public:

	Application() : m_creature(NULL), m_destructulon(NULL), m_scene(NULL), m_elapsedTime(0) {}
	virtual ~Application() {	exitPhysics(); } // Destructor

	bool creatureCreated;
	btVector3 strtOffset;			// Start offset 
	void initPhysics();				// Initialize the simulation
	void exitPhysics();				// End the simulation

protected:

	virtual void clientMoveAndDisplay();		// Update the simulation
	virtual void displayCallback();				// Render the simulation

	virtual void keyboardCallback(unsigned char key, int x, int y); // Input management

	void resetScene(const btVector3& startOffset);	// Reset the creature
	void update();									// Update objects and display the time elapsed under balance

	Destructulon					*	m_destructulon;	// The first Destructulon
	Destructulon					*	m_destructulon2;// The Second Destructulon
	Creature						*	m_creature;		// The creature
	Scene							*	m_scene;		// The scene
	btCollisionShape				*	m_ground;		// The ground
	btBroadphaseInterface			*	m_broadphase;	// The broadphase
	btCollisionDispatcher			*	m_dispatcher;	// The displatcher
	btConstraintSolver				*	m_solver;		// The solver
	btDefaultCollisionConfiguration	*	m_collisionConfiguration;	// The collision configuration

	DWORD m_startTime;		// Time starter
	DWORD m_currentTime;	// Time counter
	int m_elapsedTime;		// Time elapsed in 10e-1 sec

};

#endif
