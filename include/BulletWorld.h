#ifndef __BulletWorld_H__
#define __BulletWorld_H__

#include "btBulletDynamicsCommon.h"
#include "CinderBulletDebugDrawer.h"
#include "PParams.h"
#include "cinder/app/MouseEvent.h"
#include "cinder/Camera.h"

class BulletWorld
{
public:
	BulletWorld();
	~BulletWorld();

	void setup();
	void update();
	void draw();

	void mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam );
	void mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam );
	void mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam );

	void initPhysics();
	void donePhysics();

	void spawnBulletRagdoll( const ci::Vec3f &pos );

protected:
	btRigidBody *createRigidBody( btDynamicsWorld *world, btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

	void setupParams();

protected:

	btAlignedObjectArray< class BulletRagdoll *>     mBulletRagdolls;

	btDefaultCollisionConfiguration           *mCollisionConfiguration;
	btCollisionDispatcher                     *mDispatcher;
	btBroadphaseInterface                     *mBroadphase;
	btSequentialImpulseConstraintSolver       *mSolver;
	btDiscreteDynamicsWorld                   *mDynamicsWorld;

	btAlignedObjectArray< btCollisionShape* >  mCollisionShapes;

	CinderBulletDebugDrawer                   *mDebugDrawer;

	ci::Vec3f                                  mGravity;

	double                                     mTime;
	ci::params::PInterfaceGl                   mParams;
	static const int                           DEBUG_DRAW_NUM = 15;
	bool                                       mDebugDrawActive[ DEBUG_DRAW_NUM ];
};

#endif // __BulletWorld_H__
