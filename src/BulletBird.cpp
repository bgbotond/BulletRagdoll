#include "cinder/CinderMath.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "CinderBullet.h"
#include "BulletBird.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f

#ifndef M_PI
#define M_PI                  3.14159265358979323846
#endif

#define RADIAN( x ) ( (float)(x) * M_PI / 180  )
#define DEGREE( x ) ( (float)(x) * 180  / M_PI )

BulletBird::BulletBird( btDynamicsWorld *ownerWorld, const btVector3 &positionOffset )
: mOwnerWorld( ownerWorld )
{
	mBeckSize = .1;
	mHeadSize = .2;
	mNeckSize = .1;
	mBodySize = .3;
	mLegSize  = .1;
	mFootSize = .03;
	mHangSize = .1;

	mNeckPart = 4;
	mLegPart  = 4;
	mHangPart = 4;

	mStringSize = 2;

	mShapes.push_back( new btCylinderShape( btVector3( mBeckSize / 2, mBeckSize, mBeckSize / 2 )));
	mShapes.push_back( new btSphereShape  ( btScalar ( mHeadSize                               )));
	mShapes.push_back( new btSphereShape  ( btScalar ( mNeckSize                               )));
	mShapes.push_back( new btSphereShape  ( btScalar ( mBodySize                               )));
	mShapes.push_back( new btSphereShape  ( btScalar ( mLegSize                                )));
	mShapes.push_back( new btCylinderShape( btVector3( mFootSize * 5, mFootSize, mFootSize * 5 )));
	mShapes.push_back( new btBoxShape     ( btVector3( mHangSize    , mHangSize, mHangSize     )));

	float neck      = mBodySize + mNeckPart * 2 * mNeckSize + mHeadSize;
	float leg       = mBodySize + ( mLegPart - 1 ) * 2 * mLegSize + mLegSize;
	float stick     = neck / ci::math<float>::sqrt( 8 ); 
	float neckRotX  = 45.0f;
	float neckRotY  =  0.0f;
	float neckRotZ  =  0.0f;
	float legRotX   =  DEGREE( ci::math<float>::asin( stick / ci::math<float>::sqrt( leg * leg - stick * stick )));
	float legRotY   =  0.0f;
	float legRotZ   =  DEGREE( ci::math<float>::asin( stick / leg ));

 	float neckY = 2 * stick;                                              // y axe distance betweeen body center to head center
// 	float bodyY = 0;
	float legY  = ci::math<float>::sqrt( leg * leg - 2 * stick * stick ); // y axe distance betweeen body center to lower leg center

	// Setup all the rigid bodies
	btTransform transform;
	btTransform offset;
	offset.setIdentity();
	offset.setOrigin( positionOffset );

	ci::Vec3f pos = ci::Vec3f::zero();

	// body
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mBodySize * 10, offset * transform, mShapes[ BODYPART_BODY ] ));

	// neck
	ci::Quaternion<float> quaternion;
	quaternion = ci::Quaternion<float>( RADIAN( neckRotX ), RADIAN( neckRotY ), RADIAN( neckRotZ ));
	for( int i = 0; i < mNeckPart; ++i )
	{
		float dist = 0.0;

		if( i == 0 )
			dist = mBodySize + mNeckSize;
		else
			dist = 2 * mNeckSize;

		pos += ci::Vec3f( 0, dist, 0 );
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( quaternion * pos ));
		mBodies.push_back( localCreateRigidBody( mNeckSize * 10, offset * transform, mShapes[ BODYPART_NECK ] ));
	}

	// head
	pos += ci::Vec3f( 0, mNeckSize + mHeadSize, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( quaternion * pos ));
	mBodies.push_back( localCreateRigidBody( mHeadSize * 10, offset * transform, mShapes[ BODYPART_HEAD ] ));

	// beck
	pos = quaternion * pos + ci::Vec3f( 0, 0, mBeckSize + mHeadSize );
	transform.setRotation( btQuaternion( 0.0, RADIAN( 90.0 ), 0.0 ));
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mBeckSize * 10, offset * transform, mShapes[ BODYPART_BECK ] ));

	// left leg
	pos = ci::Vec3f::zero();
	quaternion = ci::Quaternion<float>( -RADIAN( legRotX ), RADIAN( legRotY ), -RADIAN( legRotZ ));
	for( int i = 0; i < mLegPart; ++i )
	{
		float dist = 0.0;

		if( i == 0 )
			dist = mBodySize + mLegSize;
		else
			dist = 2 * mLegSize;

		pos -= ci::Vec3f( 0, dist, 0 );
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( quaternion * pos ));
		mBodies.push_back( localCreateRigidBody( mLegSize * 10, offset * transform, mShapes[ BODYPART_LEG ] ));
	}

	// foot
	pos = quaternion * pos + ci::Vec3f( 0, - mLegSize - mFootSize, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mFootSize * 10, offset * transform, mShapes[ BODYPART_FOOT ] ));

	// right leg
	pos = ci::Vec3f::zero();
	quaternion = ci::Quaternion<float>( -RADIAN( legRotX ), RADIAN( legRotY ), RADIAN( legRotZ ));
	for( int i = 0; i < mLegPart; ++i )
	{
		float dist = 0.0;

		if( i == 0 )
			dist = mBodySize + mLegSize;
		else
			dist = 2 * mLegSize;

		pos -= ci::Vec3f( 0, dist, 0 );
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( quaternion * pos ));
		mBodies.push_back( localCreateRigidBody( mLegSize * 10, offset * transform, mShapes[ BODYPART_LEG ] ));
	}

	// foot
	pos = quaternion * pos + ci::Vec3f( 0, - mLegSize - mFootSize, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mFootSize * 10, offset * transform, mShapes[ BODYPART_FOOT ] ));

	// hang
	pos = ci::Vec3f( 0, 2 * stick + mStringSize, 2 * stick );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mHangSize * 10, offset * transform, mShapes[ BODYPART_HANG ] ));

	pos = ci::Vec3f( 0, 2 * stick + mStringSize, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mHangSize * 10, offset * transform, mShapes[ BODYPART_HANG ] ));

	pos = ci::Vec3f( -stick, 2 * stick + mStringSize, stick );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mHangSize * 10, offset * transform, mShapes[ BODYPART_HANG ] ));

	pos = ci::Vec3f(  stick, 2 * stick + mStringSize, stick );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mHangSize * 10, offset * transform, mShapes[ BODYPART_HANG ] ));

	// Setup some damping on the mBodies
	for( Bodies::iterator it = mBodies.begin(); it != mBodies.end(); ++it )
	{
		btRigidBody *body = *it;
		body->setDamping( 0.85, 0.85 );
		body->setDeactivationTime( 0.8 );
		body->setSleepingThresholds( 1.6, 2.5 );
	}

	// Now setup the constraints
	btConeTwistConstraint   *coneC;
	btPoint2PointConstraint *pointC;

	pos = ci::Vec3f::zero();
	quaternion = ci::Quaternion<float>( RADIAN( neckRotX ), RADIAN( neckRotY ), RADIAN( neckRotZ ));

	btTransform localA, localB;
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localA.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mBodySize, 0 ))));
	localB.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localB.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mNeckSize, 0 ))));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_BODY ), *getBody( BODYPART_NECK, 0 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	for( int i = 0; i < mNeckPart - 1; ++i )
	{
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localA.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mNeckSize, 0 ))));
		localB.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localB.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mNeckSize, 0 ))));
		coneC = new btConeTwistConstraint( *getBody( BODYPART_NECK, i ), *getBody( BODYPART_NECK, i + 1 ), localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localA.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mNeckSize, 0 ))));
	localB.getBasis().setEulerZYX(0,-RADIAN( 45 ),RADIAN( 90 )); localB.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mHeadSize, 0 ))));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_NECK, mNeckPart - 1 ), *getBody( BODYPART_HEAD ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	btVector3 pivotA = btVector3( 0, -mBeckSize, 0         );
	btVector3 pivotB = btVector3( 0,          0, mHeadSize );
	pointC = new btPoint2PointConstraint( *getBody( BODYPART_BECK ), *getBody( BODYPART_HEAD ), pivotA, pivotB );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( pointC, false );

	// left leg
	quaternion = ci::Quaternion<float>( -RADIAN( legRotX ), RADIAN( legRotY ), -RADIAN( legRotZ ));
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, RADIAN( legRotX ), RADIAN( legRotZ ) ); localA.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mBodySize, 0 ))));
	localB.getBasis().setEulerZYX( 0, RADIAN( legRotX ), RADIAN( legRotZ ) ); localB.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mLegSize , 0 ))));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_BODY ), *getBody( BODYPART_LEG, 0 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	for( int i = 0; i < mLegPart - 1; ++i )
	{
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX( 0, RADIAN( legRotX ), RADIAN( legRotZ ) ); localA.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mLegSize, 0 ))));
		localB.getBasis().setEulerZYX( 0, RADIAN( legRotX ), RADIAN( legRotZ ) ); localB.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mLegSize, 0 ))));
		coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, i ), *getBody( BODYPART_LEG, i + 1 ), localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, RADIAN( 90 ) ); localA.setOrigin( CinderBullet::convert( - ci::Vec3f( 0, mLegSize , 0 )));
	localB.getBasis().setEulerZYX( 0, 0, RADIAN( 90 ) ); localB.setOrigin( CinderBullet::convert(   ci::Vec3f( 0, mFootSize, 0 )));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, mLegPart - 1 ), *getBody( BODYPART_FOOT, 0 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	// right leg
	quaternion = ci::Quaternion<float>( -RADIAN( legRotX ), RADIAN( legRotY ), RADIAN( legRotZ ));
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, -RADIAN( 180+legRotX ), -RADIAN( legRotZ ) ); localA.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mBodySize, 0 ))));
	localB.getBasis().setEulerZYX( 0, -RADIAN( 180+legRotX ), -RADIAN( legRotZ ) ); localB.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mLegSize , 0 ))));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_BODY ), *getBody( BODYPART_LEG, mLegPart ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	for( int i = 0; i < mLegPart - 1; ++i )
	{
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX( 0, -RADIAN( 180+legRotX ), -RADIAN( legRotZ ) ); localA.setOrigin( CinderBullet::convert( quaternion * ( - ci::Vec3f( 0, mLegSize, 0 ))));
		localB.getBasis().setEulerZYX( 0, -RADIAN( 180+legRotX ), -RADIAN( legRotZ ) ); localB.setOrigin( CinderBullet::convert( quaternion * (   ci::Vec3f( 0, mLegSize, 0 ))));
		coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, mLegPart + i ), *getBody( BODYPART_LEG, mLegPart + i + 1 ), localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX( 0, 0, RADIAN( 90 ) ); localA.setOrigin( CinderBullet::convert( - ci::Vec3f( 0, mLegSize , 0 )));
	localB.getBasis().setEulerZYX( 0, 0, RADIAN( 90 ) ); localB.setOrigin( CinderBullet::convert(   ci::Vec3f( 0, mFootSize, 0 )));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, mLegPart + mLegPart - 1 ), *getBody( BODYPART_FOOT, 1 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);


	// hang
	pivotA = btVector3( 0, -mStringSize, 0 );
	pivotB = btVector3( 0, 0, 0 );
	pointC = new btPoint2PointConstraint( *getBody( BODYPART_HANG, 0 ), *getBody( BODYPART_HEAD ), pivotA, pivotB );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	mOwnerWorld->addConstraint( pointC, false );

	pivotA = btVector3( 0, -mStringSize - neckY, 0 );
	pivotB = btVector3( 0,  0, 0 );
	pointC = new btPoint2PointConstraint( *getBody( BODYPART_HANG, 1 ), *getBody( BODYPART_BODY ), pivotA, pivotB );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	mOwnerWorld->addConstraint( pointC, false );

	pivotA = btVector3( 0, -mStringSize - neckY - legY, 0 );
	pivotB = btVector3( 0,  0, 0 );
	pointC = new btPoint2PointConstraint( *getBody( BODYPART_HANG, 2 ), *getBody( BODYPART_LEG, mLegPart - 1 ), pivotA, pivotB );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	mOwnerWorld->addConstraint( pointC, false );

	pivotA = btVector3( 0, -mStringSize - neckY - legY, 0 );
	pivotB = btVector3( 0,  0, 0 );
	pointC = new btPoint2PointConstraint( *getBody( BODYPART_HANG, 3 ), *getBody( BODYPART_LEG, 2 * mLegPart - 1 ), pivotA, pivotB );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	mOwnerWorld->addConstraint( pointC, false );
}

BulletBird::~BulletBird()
{
	// Remove all constraints
	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( *it );
		delete *it;
	}

	for( Bodies::iterator it = mBodies.begin(); it != mBodies.end(); ++it )
	{
		btRigidBody *rigidBody = *it;

		mOwnerWorld->removeRigidBody( rigidBody );

		if( rigidBody->getMotionState())
			delete rigidBody->getMotionState();

		delete rigidBody;
	}

	for( Shapes::iterator it = mShapes.begin(); it != mShapes.end(); ++it )
	{
		delete *it;
	}
}

btRigidBody *BulletBird::localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if (isDynamic)
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	btRigidBody* body = new btRigidBody( rbInfo );

	mOwnerWorld->addRigidBody( body );

	return body;
}

btCollisionShape *BulletBird::getShape( BodyPart bodyPart )
{
	return mShapes[ bodyPart ];
}

btRigidBody *BulletBird::getBody( BodyPart bodyPart, int count /* = 0 */ )
{
	switch( bodyPart )
	{
	case BODYPART_BECK : return mBodies[ 2 + mNeckPart                    ]; break;
	case BODYPART_HEAD : return mBodies[ 1 + mNeckPart                    ]; break;
	case BODYPART_NECK : return mBodies[ 1 + count                        ]; break;
	case BODYPART_BODY : return mBodies[ 0                                ]; break;
	case BODYPART_LEG  : 
		if( count < mLegPart )
			return mBodies[ 3 + mNeckPart + count ];
		else 
			return mBodies[ 4 + mNeckPart + count ];
		break;
	case BODYPART_FOOT :
		if( count == 0 )
			return mBodies[ 3 + mNeckPart + mLegPart ];
		else
			return mBodies[ 4 + mNeckPart + 2 * mLegPart ];
		break;
	case BODYPART_HANG :
		return mBodies[ 5 + mNeckPart + 2 * mLegPart + count ];
		break;
	}

	return 0;
}
