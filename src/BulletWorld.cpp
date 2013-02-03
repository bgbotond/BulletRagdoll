#include "cinder/app/App.h"

#include "CinderBullet.h"
#include "BulletWorld.h"
#include "BulletRagdoll.h"

BulletWorld::BulletWorld()
: mCollisionConfiguration( NULL )
, mDispatcher( NULL )
, mBroadphase( NULL )
, mSolver( NULL )
, mDynamicsWorld( NULL )
, mDebugDrawer( NULL )
, mDragging( false )
{
	mBulletConstraint.init();
}

BulletWorld::~BulletWorld()
{
	donePhysics();
}

void BulletWorld::setup()
{
	setupParams();

	mTime = ci::app::App::get()->getElapsedSeconds();
	initPhysics();
}

void BulletWorld::update()
{
	if( mGravity != CinderBullet::convert( mDynamicsWorld->getGravity()))
		mDynamicsWorld->setGravity( CinderBullet::convert( mGravity ));

	for( int i = 0; i < DEBUG_DRAW_NUM; ++i )
	{
		if( mDebugDrawActive[ i ] != mDebugDrawer->getDrawEnable( (CinderBulletDebugDrawer::DrawType)i ))
			mDebugDrawer->setDrawEnable( (CinderBulletDebugDrawer::DrawType)i, mDebugDrawActive[ i ] );
	}

	// simple dynamics world doesn't handle fixed-time-stepping
	double time = ci::app::App::get()->getElapsedSeconds();
	float  ellapsedTime = ( float )( time - mTime ) * 1000000;
	time = mTime;

	float minFPS = 1000000.f/60.f;
	if( ellapsedTime > minFPS )
		ellapsedTime = minFPS;

	if( mDynamicsWorld )
	{
		mDynamicsWorld->stepSimulation( ellapsedTime / 1000000.f );

		//optional but useful: debug drawing
		mDynamicsWorld->debugDrawWorld();
	}
}

void BulletWorld::draw()
{
	if( /*mDrawBulletDebug && */mDynamicsWorld != NULL )
		mDynamicsWorld->debugDrawWorld();
}

void BulletWorld::initPhysics()
{
	mCollisionConfiguration = new btDefaultCollisionConfiguration();
	mDispatcher             = new btCollisionDispatcher( mCollisionConfiguration );

	btVector3 worldAabbMin( -10000, -10000, -10000 );
	btVector3 worldAabbMax(  10000,  10000,  10000 );
	mBroadphase = new btAxisSweep3( worldAabbMin, worldAabbMax );

	mSolver     = new btSequentialImpulseConstraintSolver;

	mDynamicsWorld = new btDiscreteDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
//	mDynamicsWorld->setGravity( btVector3( 0, 0, 0 ) );
	mDynamicsWorld->setGravity( btVector3( 0, -9.81, 0 ) );

	mDebugDrawer = new CinderBulletDebugDrawer();
	mDynamicsWorld->setDebugDrawer( mDebugDrawer );

	// Setup a big ground box
	{
		btCollisionShape *groundShape = new btBoxShape( btVector3( btScalar( 200. ), btScalar( 10. ), btScalar( 200. )));

		mCollisionShapes.push_back( groundShape );
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin( btVector3( 0, -10, 0 ));
		createRigidBody( mDynamicsWorld, btScalar( 0 ), groundTransform, groundShape );
	}
}

void BulletWorld::donePhysics()
{
	for( int i = 0; i < mBulletRagdolls.size(); ++i )
	{
		BulletRagdoll *bulletRagdoll = mBulletRagdolls[ i ];
		delete bulletRagdoll;
	}

	for( int i = mDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i )
	{
		btCollisionObject *obj = mDynamicsWorld->getCollisionObjectArray()[ i ];
		btRigidBody *body = btRigidBody::upcast( obj );
		if( body && body->getMotionState())
		{
			delete body->getMotionState();
		}

		mDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	for( int i = 0; i < mCollisionShapes.size(); ++i )
	{
		btCollisionShape *shape = mCollisionShapes[ i ];
		delete shape;
	}

	delete mDynamicsWorld;
	delete mSolver;
	delete mBroadphase;
	delete mDispatcher;
	delete mCollisionConfiguration;
}

btRigidBody *BulletWorld::createRigidBody( btDynamicsWorld *world, btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if( isDynamic )
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState *myMotionState = new btDefaultMotionState( startTransform );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	rbInfo.m_additionalDamping = true;
	btRigidBody *body = new btRigidBody( rbInfo );

	world->addRigidBody( body );

	return body;
}

void BulletWorld::spawnBulletRagdoll( const ci::Vec3f &pos )
{
	BulletRagdoll *bulletRagdoll = new BulletRagdoll( mDynamicsWorld, CinderBullet::convert( pos ));
	mBulletRagdolls.push_back( bulletRagdoll );
}

void BulletWorld::setupParams()
{
	mParams = ci::params::PInterfaceGl( "Bullet", ci::Vec2i( 250, 300 ), ci::Vec2i( 300, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mParams.addText( "World" );
	mParams.addPersistentParam( "Gravity", &mGravity, ci::Vec3f( 0.0f, -9.81f, 0.0f ) );

	mParams.addText( "DebugDraw" );
	const char *text[DEBUG_DRAW_NUM] = { "DrawWireframe", "DrawAabb", "DrawFeaturesText", "DrawContactPoints", "NoDeactivation", "NoHelpText", "DrawText", "ProfileTimings", "EnableSatComparison", "DisableBulletLCP", "EnableCCD", "DrawConstraints", "DrawConstraintLimits", "FastWireframe", "DrawNormals" };

	for( int i = 0; i < DEBUG_DRAW_NUM; ++i )
	{
		mParams.addPersistentParam( text[i], &mDebugDrawActive[i], true );
	}
}

void BulletWorld::mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	ci::Vec2f pos  = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
	pos.y          = 1.0f - pos.y;
	ci::Ray ray    = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
	mDragging      = checkIntersects( ray, cam.getFarClip(), &mBulletConstraint );
	if( mDragging )
	{
		addConstraint( mBulletConstraint, 0.0f, 0.01f );
	}
}

void BulletWorld::addConstraint( const BulletConstraint &constraint, float clamping, float tau )
{
	mDynamicsWorld->addConstraint( constraint.mConstraint );
	constraint.mConstraint->m_setting.m_impulseClamp	= clamping;
	constraint.mConstraint->m_setting.m_tau				= tau;
}

void BulletWorld::removeConstraint( const BulletConstraint &constraint )
{
	mDynamicsWorld->removeConstraint( constraint.mConstraint );
}

bool BulletWorld::checkIntersects( const ci::Ray &ray, float farClip, BulletConstraint *constraint )
{
	btVector3 rayFrom = CinderBullet::convert( ray.getOrigin() );
	btVector3 rayTo   = CinderBullet::convert( ray.calcPosition( farClip ) );

	btCollisionWorld::ClosestRayResultCallback rayCallback( rayFrom, rayTo );
	mDynamicsWorld->rayTest( rayFrom, rayTo, rayCallback );

	if( rayCallback.hasHit() )
	{
		btRigidBody* collisionBody = const_cast<btRigidBody*>( btRigidBody::upcast( rayCallback.m_collisionObject ));
		if( collisionBody )
		{
			btVector3 position = rayCallback.m_hitPointWorld;
			btVector3 pivot    = collisionBody->getCenterOfMassTransform().inverse() * position;

			constraint->mConstraint = new btPoint2PointConstraint( *collisionBody, pivot );
			constraint->mDistance   = ( position - rayFrom ).length();
			constraint->mPosition   = CinderBullet::convert( rayTo );

			return true;
		}
	}
	return false;
}

void BulletWorld::mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	if( mDragging )
	{
		ci::Vec2f pos = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
		pos.y         = 1.0f - pos.y;
		ci::Ray ray   = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
		mBulletConstraint.update( ray );
	}
}

void BulletWorld::mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	if ( mDragging )
	{
		removeConstraint( mBulletConstraint );
		mBulletConstraint.reset();
		mDragging = false;
	}
}
