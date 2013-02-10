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

using namespace ci;

/*
      hb      head beck
      n       neck
      n       neck
      n       neck
      n       neck
      b       body
     l l      leg
    l   l     leg
   l     l    leg
  l       l   leg
 f         f  foot
*/

mndl::kit::params::PInterfaceGl   BulletBird::mParams;
float                             BulletBird::mBeckSize   = 1.0; // cylinder shape
float                             BulletBird::mHeadSize   = 2.0; // sphere shape
float                             BulletBird::mNeckSize   = 1.0; // sphere shape
float                             BulletBird::mBodySize   = 3.0; // sphere shape
float                             BulletBird::mLegSize    = 1.0; // cylinder shape
float                             BulletBird::mFootSize   = 0.3; // sphere shape
float                             BulletBird::mStickSize  = 4.0; // control cross size

int                               BulletBird::mNeckPart   = 4  ; // count of neck sphere
int                               BulletBird::mLegPart    = 4  ; // count of leg  sphere
int                               BulletBird::mStringSize = 0.0; // size of string from head

float                             BulletBird::mTau          = 0.01;
float                             BulletBird::mDamping      = 1.0;
float                             BulletBird::mImpulseClamp = 0.0;


BulletBird::BulletBird( btDynamicsWorld *ownerWorld, const ci::Vec3f &worldOffset )
: mOwnerWorld( ownerWorld )
{
	float hangSize = 1;

	mShapes.push_back( new btConeShape    ( mBeckSize / 2, 2 * mBeckSize                        )); // -> BODYPART_BECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mHeadSize                               ))); // -> BODYPART_HEAD
	mShapes.push_back( new btSphereShape  ( btScalar ( mNeckSize                               ))); // -> BODYPART_NECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mBodySize                               ))); // -> BODYPART_BODY
	mShapes.push_back( new btSphereShape  ( btScalar ( mLegSize                                ))); // -> BODYPART_LEG
	mShapes.push_back( new btCylinderShape( btVector3( mFootSize * 8, mFootSize, mFootSize * 8 ))); // -> BODYPART_FOOT

	float neckLength = mBodySize + mNeckPart * 2 * mNeckSize + mHeadSize;
	float legLength  = mBodySize + mLegPart  * 2 * mLegSize  - mLegSize;

	Vec3f offset       = Vec3f( 0, 2 * mFootSize + mLegSize, 0 ) +  worldOffset;
	Vec3f bodyPos      = Vec3f( 0, ci::math<float>::sqrt( ( legLength  * legLength  ) - 2 * ( mStickSize * mStickSize ) )            ,   mStickSize );
	Vec3f headPos      = Vec3f( 0, ci::math<float>::sqrt( ( neckLength * neckLength ) - 4 * ( mStickSize * mStickSize ) ) + bodyPos.y, - mStickSize );
	Vec3f beckPos      = Vec3f( 0, 0, - mHeadSize - mBeckSize ) + headPos;
	Vec3f leftFootPos  = Vec3f( -mStickSize, mFootSize, 0 );
	Vec3f rightFootPos = Vec3f(  mStickSize, mFootSize, 0 );
	Vec3f leftLegPos   = Vec3f( -mStickSize, 0, 0 );
	Vec3f rightLegPos  = Vec3f(  mStickSize, 0, 0 );

	mPosHangCenter     = Vec3f( 0, headPos.y + mStringSize, 0 );
	mPosHangFront      = mPosHangCenter + offset + Vec3f( 0, 0, -mStickSize );
	mPosHangBack       = mPosHangCenter + offset + Vec3f( 0, 0,  mStickSize );
	mPosHangLeft       = mPosHangCenter + offset + Vec3f( -mStickSize, 0, 0 );
	mPosHangRight      = mPosHangCenter + offset + Vec3f(  mStickSize, 0, 0 );

	Vec3f pos;
	Vec3f orient;
	Quatf rotate;
	btTransform transform;

	// left foot
	pos = leftFootPos + worldOffset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mFootSize , transform, mShapes[ BODYPART_FOOT ] ));

	// right foot
	pos = rightFootPos + worldOffset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mFootSize , transform, mShapes[ BODYPART_FOOT ] ));

	// left legs
	orient = bodyPos - leftLegPos;
	orient.normalize();
	pos = leftLegPos + offset;
	for( int i = 0; i < mLegPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mLegSize * orient ));
		mBodies.push_back( localCreateRigidBody( mLegSize , transform, mShapes[ BODYPART_LEG ] ));
	}

	// right legs
	orient = bodyPos - rightLegPos;
	orient.normalize();
	pos = rightLegPos + offset;
	for( int i = 0; i < mLegPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mLegSize * orient ));
		mBodies.push_back( localCreateRigidBody( mLegSize , transform, mShapes[ BODYPART_LEG ] ));
	}

	// body
	pos = bodyPos + offset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mBodySize , transform, mShapes[ BODYPART_BODY ] ));

	// neck
	orient = headPos - bodyPos;
	orient.normalize();
	pos = bodyPos + offset + orient * ( mBodySize + mNeckSize );
	for( int i = 0; i < mNeckPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mNeckSize * orient ));
		mBodies.push_back( localCreateRigidBody( mNeckSize , transform, mShapes[ BODYPART_NECK ] ));
	}

	// head
	pos = headPos + offset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mHeadSize , transform, mShapes[ BODYPART_HEAD ] ));

	// beck
	pos    = beckPos + offset;
	rotate = Quatf( RADIAN( -90 ), 0, 0 );
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mBeckSize , transform, mShapes[ BODYPART_BECK ] ));

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
	btTransform              localA, localB;


	// left foot
	rotate = Quatf( 0, 0, RADIAN( 90 ));
	localA.setIdentity(); localB.setIdentity();
	localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert( - ci::Vec3f( 0, mLegSize , 0 )));
	localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert(   ci::Vec3f( 0, mFootSize, 0 )));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, 0 ), *getBody( BODYPART_FOOT, 0 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	// right foot
	rotate = Quatf( 0, 0, RADIAN( 90 ));
	localA.setIdentity(); localB.setIdentity();
	localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert( - ci::Vec3f( 0, mLegSize , 0 )));
	localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert(   ci::Vec3f( 0, mFootSize, 0 )));
	coneC = new btConeTwistConstraint( *getBody( BODYPART_LEG, mLegPart ), *getBody( BODYPART_FOOT, 1 ), localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	btRigidBody *rigidBodyA;
	btRigidBody *rigidBodyB;
	float sizeA;
	float sizeB;

	// left leg
	orient = bodyPos - leftLegPos;
	orient.normalize();
	rotate = Quatf( Vec3f::xAxis(), orient );
	for( int i = 0; i < mLegPart; ++i )
	{
		if( i == mLegPart - 1 )
		{
			rigidBodyA = getBody( BODYPART_LEG, i     );
			rigidBodyB = getBody( BODYPART_BODY       );
			sizeA = mLegSize;
			sizeB = mBodySize;
		}
		else
		{
			rigidBodyA = getBody( BODYPART_LEG, i     );
			rigidBodyB = getBody( BODYPART_LEG, i + 1 );
			sizeA = mLegSize;
			sizeB = mLegSize;
		}

		localA.setIdentity(); localB.setIdentity();
		localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert(   orient * sizeA ));
		localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert( - orient * sizeB ));
		coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	// right leg
	orient = bodyPos - rightLegPos;
	orient.normalize();
	rotate = Quatf( Vec3f::xAxis(), orient );
	for( int i = 0; i < mLegPart; ++i )
	{
		if( i == mLegPart - 1 )
		{
			rigidBodyA = getBody( BODYPART_LEG , mLegPart + i );
			rigidBodyB = getBody( BODYPART_BODY               );
			sizeA = mLegSize;
			sizeB = mBodySize;
		}
		else
		{
			rigidBodyA = getBody( BODYPART_LEG, mLegPart + i     );
			rigidBodyB = getBody( BODYPART_LEG, mLegPart + i + 1 );
			sizeA = mLegSize;
			sizeB = mLegSize;
		}

		localA.setIdentity(); localB.setIdentity();
		localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert(   orient * sizeA ));
		localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert( - orient * sizeB ));
		coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	// neck1
	orient = headPos - bodyPos;
	orient.normalize();
	rotate = Quatf( Vec3f::xAxis(), orient );
	rigidBodyA = getBody( BODYPART_BODY    );
	rigidBodyB = getBody( BODYPART_NECK, 0 );
	sizeA = mBodySize;
	sizeB = mNeckSize;
	localA.setIdentity(); localB.setIdentity();
	localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert(   orient * sizeA ));
	localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert( - orient * sizeB ));
	coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	// neck
	orient = headPos - bodyPos;
	orient.normalize();
	rotate = Quatf( Vec3f::xAxis(), orient );
	for( int i = 0; i < mNeckPart; ++i )
	{
		if( i == mNeckPart - 1 )
		{
			rigidBodyA = getBody( BODYPART_NECK,  i );
			rigidBodyB = getBody( BODYPART_HEAD     );
			sizeA = mNeckSize;
			sizeB = mHeadSize;
		}
		else
		{
			rigidBodyA = getBody( BODYPART_NECK, i     );
			rigidBodyB = getBody( BODYPART_NECK, i + 1 );
			sizeA = mNeckSize;
			sizeB = mNeckSize;
		}

		localA.setIdentity(); localB.setIdentity();
		localA.setRotation( CinderBullet::convert( rotate )); localA.setOrigin( CinderBullet::convert(   orient * sizeA ));
		localB.setRotation( CinderBullet::convert( rotate )); localB.setOrigin( CinderBullet::convert( - orient * sizeB ));
		coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
		coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
		mConstraints.push_back( coneC );
		coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		mOwnerWorld->addConstraint( coneC, false);
	}

	// beck
	rigidBodyA = getBody( BODYPART_HEAD );
	rigidBodyB = getBody( BODYPART_BECK );
	sizeA = mHeadSize;
	sizeB = mBeckSize;
	localA.setIdentity(); localB.setIdentity();
	localA.setRotation( CinderBullet::convert( Quatf( Vec3f::xAxis(), -Vec3f::zAxis()))); localA.setOrigin( CinderBullet::convert( Vec3f( 0, 0     , -sizeA ) ) );
	localB.setRotation( CinderBullet::convert( Quatf( -Vec3f::zAxis(),  Vec3f::yAxis()) * Quatf( Vec3f::xAxis(),  Vec3f::yAxis()))); localB.setOrigin( CinderBullet::convert( Vec3f( 0, -sizeB, 0      ) ) );
	coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);

	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		btConeTwistConstraint *constraint = static_cast< btConeTwistConstraint * >( *it );

		constraint->setDamping( .1 );
	}

	HangConstraint *hangConstraint;
	Vec3f hangPos;

	pos     = headPos + offset;
	hangPos = mPosHangFront;
	hangConstraint = new HangConstraint( hangPos, getBody( BODYPART_HEAD ), pos );
	mHangConstraints.push_back( hangConstraint );
	mOwnerWorld->addConstraint( hangConstraint->mConstraint, false );

	pos     = bodyPos + offset;
	hangPos = mPosHangBack;
	hangConstraint = new HangConstraint( hangPos, getBody( BODYPART_BODY ), pos );
	mHangConstraints.push_back( hangConstraint );
	mOwnerWorld->addConstraint( hangConstraint->mConstraint, false );

	pos     = leftLegPos + offset;
	hangPos = mPosHangLeft;
	hangConstraint = new HangConstraint( hangPos, getBody( BODYPART_LEG, 0 ), pos );
	mHangConstraints.push_back( hangConstraint );
	mOwnerWorld->addConstraint( hangConstraint->mConstraint, false );

	pos     = rightLegPos + offset;
	hangPos = mPosHangRight;
	hangConstraint = new HangConstraint( hangPos, getBody( BODYPART_LEG, mLegPart ), pos );
	mHangConstraints.push_back( hangConstraint );
	mOwnerWorld->addConstraint( hangConstraint->mConstraint, false );
}

BulletBird::~BulletBird()
{
	// Remove all constraints
	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( *it );
		delete *it;
	}

	for( HangConstraints::iterator it = mHangConstraints.begin(); it != mHangConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( (*it)->mConstraint );
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

void BulletBird::update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm )
{
	if( pos  == Vec3f::zero()
	 || dir  == Vec3f::zero()
	 || norm == Vec3f::zero())
		return;

	ci::Vec3f dir2 = dir.normalized();
	ci::Vec3f cross = dir2.cross( norm.normalized() );
	cross.normalize();

	ci::Vec3f hangPos[4];

	hangPos[ 0 ] = mPosHangFront + (   dir2  * mStickSize ) - ( - Vec3f::zAxis() * mStickSize );
	hangPos[ 1 ] = mPosHangBack  + ( - dir2  * mStickSize ) - (   Vec3f::zAxis() * mStickSize );
	hangPos[ 2 ] = mPosHangLeft  + (   cross * mStickSize ) - ( - Vec3f::xAxis() * mStickSize );
	hangPos[ 3 ] = mPosHangRight + ( - cross * mStickSize ) - (   Vec3f::xAxis() * mStickSize );

	for( int i = 0; i < 4; ++i )
	{
		mHangConstraints[i]->update( hangPos[i] );
		mHangConstraints[i]->mConstraint->m_setting.m_damping      = mDamping;
		mHangConstraints[i]->mConstraint->m_setting.m_impulseClamp = mImpulseClamp;
		mHangConstraints[i]->mConstraint->m_setting.m_tau          = mTau;
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
	case BODYPART_FOOT : return mBodies[ 0 + count                            ]; break;
	case BODYPART_LEG  : return mBodies[ 2 + count                            ]; break;
	case BODYPART_BODY : return mBodies[ 2 + 2 * mLegPart                     ]; break;
	case BODYPART_NECK : return mBodies[ 3 + 2 * mLegPart + count             ]; break;
	case BODYPART_HEAD : return mBodies[ 3 + 2 * mLegPart + mNeckPart         ]; break;
	case BODYPART_BECK : return mBodies[ 4 + 2 * mLegPart + mNeckPart         ]; break;
	}

	return 0;
}

void BulletBird::setupParams()
{
	mParams = mndl::kit::params::PInterfaceGl( "Bird", ci::Vec2i( 250, 350 ), ci::Vec2i( 500, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mParams.addText( "Body" );
	mParams.addPersistentParam( "Beck size" , &mBeckSize , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParams.addPersistentParam( "Head size" , &mHeadSize , 2.0f, "min=0.5 max=5.0 step=0.1" );
	mParams.addPersistentParam( "Neck size" , &mNeckSize , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParams.addPersistentParam( "Body size" , &mBodySize , 3.0f, "min=0.5 max=5.0 step=0.1" );
	mParams.addPersistentParam( "Leg size"  , &mLegSize  , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParams.addPersistentParam( "Foot size" , &mFootSize , 0.3f, "min=0.1 max=3.0 step=0.1" );
	mParams.addPersistentParam( "Stick size", &mStickSize, 4.0f, "min=0.5 max=5.0 step=0.1" );

	mParams.addPersistentParam( "Neck part" , &mNeckPart , 4   , "min=2 max=8 step=1" );
	mParams.addPersistentParam( "Leg part"  , &mLegPart  , 4   , "min=2 max=8 step=1" );

	mParams.addPersistentParam( "String size", &mStringSize, 0.0f, "min=0.0 max=5.0 step=0.1" );

	mParams.addText( "Hang constraints" );
	mParams.addPersistentParam( "Tau"          , &mTau         , 0.01f,  "min=0.0 max=1.0 step=0.01" );
	mParams.addPersistentParam( "Damping"      , &mDamping     , 1.0f,  "min=0.0 max=1.0 step=0.01" );
	mParams.addPersistentParam( "Impulse clamp", &mImpulseClamp, 0.0f,  "min=0.0 max=10.0 step=0.1" );
}
