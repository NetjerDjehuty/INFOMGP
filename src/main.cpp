
#include "Application.h"

#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        Application app;

        app.initPhysics();
		app.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,1024,768,"INFOMGP - Assignment",&app);
}
