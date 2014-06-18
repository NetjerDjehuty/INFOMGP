
#ifndef SCENE_H
#define SCENE_H

#include "btBulletDynamicsCommon.h"

class Scene {

public:
	Scene (btDynamicsWorld* ownerWorld); // Constructor

	virtual	~Scene(); // Destructor

	void update(int elapsedTime, const btVector3& creatureCOM);		// Update the scene (platform and ball)
	void switchPlatform() {m_PlatformActive = !m_PlatformActive;}	// Activate / Deactivate platform
	void switchBall(){m_BallActive = !m_BallActive;}				// Activate / Deactivate ball
	void shootCannon(){m_CannonActive = true;}						// FIRE!

protected:

	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btCollisionShape	*	m_platform_shape;			// The primitive shape of the platform
	btCollisionShape	*	m_ball_shape;				// The primitive shape of the ball
	btCollisionShape	*	m_cannon_ball_shape;		// The primitive shape of the cannon ball
	btRigidBody			*	m_platform;					// The platform body
	btRigidBody			*	m_ball;						// The ball body
	btRigidBody			*	m_cannonBall;				// The cannon ball body

	//Platform movements
	int m_axisToRotatePlatform;				// current axis of rotation of the platform
	int m_signToRotatePlatform;				// current direction of rotation of the platform
	btScalar m_magnitudeToRotatePlatform;	// current magnitude of the rotation

	//Timer
	int lastChangeRotation;		// Time of last change of rotation
	int lastShoot;				// Time of last ball shooting
	int lastCannon;				// Time of last cannon ball shooting

	bool m_PlatformActive;		// Platform is active
	bool m_BallActive;			// Ball is active
	bool m_CannonActive;		// Cannon is active

};

#endif
