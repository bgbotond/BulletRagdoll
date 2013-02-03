#ifndef __BulletWorld_H__
#define __BulletWorld_H__

#include "btBulletDynamicsCommon.h"
#include "CinderBulletDebugDrawer.h"
#include "BulletConstraint.h"
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

	bool checkIntersects( const ci::Ray &ray, float farClip, BulletConstraint *constraint );
	void addConstraint( const BulletConstraint &constraint, float clamping, float tau );
	void removeConstraint( const BulletConstraint &constraint );

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
	static const int                           DEBUG_DRAW_NUM = 16;
	bool                                       mDebugDrawActive[ DEBUG_DRAW_NUM ];

	BulletConstraint                           mBulletConstraint;
	bool                                       mDragging;
};

#endif // __BulletWorld_H__
