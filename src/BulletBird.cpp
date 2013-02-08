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

BulletBird::BulletBird( btDynamicsWorld *ownerWorld, const ci::Vec3f &worldOffset )
: mOwnerWorld( ownerWorld )
{
	mBodySize = 3;
	mNeckSize = 1;
	mHeadSize = 2;
	mBeckSize = 1;

	mLegSize  = 1;
	mFootSize = 0.3;

	mNeckPart = 4;
	mLegPart  = 4;

	mStringSize = 0;

	mStickSize  = 3;

//	mShapes.push_back( new btCylinderShape( btVector3( mBeckSize / 2, mBeckSize, mBeckSize / 2 ))); // -> BODYPART_BECK
	mShapes.push_back( new btConeShape    ( mBeckSize / 2, 2 * mBeckSize                         )); // -> BODYPART_BECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mHeadSize                               ))); // -> BODYPART_HEAD
	mShapes.push_back( new btSphereShape  ( btScalar ( mNeckSize                               ))); // -> BODYPART_NECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mBodySize                               ))); // -> BODYPART_BODY
	mShapes.push_back( new btSphereShape  ( btScalar ( mLegSize                                ))); // -> BODYPART_LEG
	mShapes.push_back( new btCylinderShape( btVector3( mFootSize * 5, mFootSize, mFootSize * 5 ))); // -> BODYPART_FOOT

	float neckLength = mBodySize + mNeckPart * 2 * mNeckSize + mHeadSize;
	float legLength  = mBodySize + mLegPart  * 2 * mLegSize  - mLegSize;

	Vec3f offset       = Vec3f( 0, 2 * mFootSize + mLegSize, 0 );
	Vec3f bodyPos      = Vec3f( 0, ci::math<float>::sqrt( ( legLength  * legLength  ) - 2 * ( mStickSize * mStickSize ) )            , - mStickSize );
	Vec3f headPos      = Vec3f( 0, ci::math<float>::sqrt( ( neckLength * neckLength ) - 4 * ( mStickSize * mStickSize ) ) + bodyPos.y,   mStickSize );
	Vec3f beckPos      = Vec3f( 0, 0, mHeadSize + mBeckSize ) + headPos;
	Vec3f leftFootPos  = Vec3f(  mStickSize, mFootSize, 0 );
	Vec3f rightFootPos = Vec3f( -mStickSize, mFootSize, 0 );
	Vec3f leftLegPos   = Vec3f(  mStickSize, 0, 0 );
	Vec3f rightLegPos  = Vec3f( -mStickSize, 0, 0 );

	Vec3f pos;
	Vec3f orient;
	Quatf rotate;
	btTransform transform;

	// left foot
	pos = leftFootPos;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mBodies.push_back( localCreateRigidBody( mFootSize , transform, mShapes[ BODYPART_FOOT ] ));

	// right foot
	pos = rightFootPos;
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
	rotate = Quatf( RADIAN( 90 ), 0, 0 );
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
	btPoint2PointConstraint *pointC;
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
/*
	rigidBodyA = getBody( BODYPART_HEAD );
	rigidBodyB = getBody( BODYPART_BECK );
	sizeA = mHeadSize;
	sizeB = mBeckSize;
	pointC = new btPoint2PointConstraint( *rigidBodyA, *rigidBodyB, CinderBullet::convert( Vec3f( 0, 0, sizeA ) ), CinderBullet::convert( Vec3f( 0, -sizeB, 0 ) ) );
	mConstraints.push_back( pointC );
	pointC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	pointC->m_setting.m_impulseClamp = 0;
	pointC->m_setting.m_tau          = 0.01;

	mOwnerWorld->addConstraint( pointC, false );
*/
	rotate = Quatf( Vec3f::xAxis(), Vec3f::yAxis());
	rigidBodyA = getBody( BODYPART_HEAD );
	rigidBodyB = getBody( BODYPART_BECK );
	sizeA = mHeadSize;
	sizeB = mBeckSize;
	localA.setIdentity(); localB.setIdentity();
	localA.setRotation( CinderBullet::convert( Quatf( Vec3f::xAxis(), Vec3f::zAxis()))); localA.setOrigin( CinderBullet::convert( Vec3f( 0, 0     , sizeA  ) ) );
	localB.setRotation( CinderBullet::convert( Quatf( Vec3f::xAxis(), Vec3f::yAxis()))); localB.setOrigin( CinderBullet::convert( Vec3f( 0, -sizeB, 0      ) ) );
	coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	coneC->setLimit( RADIAN( 45 ), RADIAN( 45 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	mOwnerWorld->addConstraint( coneC, false);


	btVector3 position;
	btVector3 pivot;

	mHangPivot[0] = headPos + offset;
//	pivot    = getBody( BODYPART_HEAD )->getCenterOfMassTransform().inverse() * mHangPivot[0];
	pivot = btVector3( 0, 0, 0 );
	mHangConstraint[0] = new btPoint2PointConstraint( *getBody( BODYPART_HEAD ), pivot );
	mOwnerWorld->addConstraint( mHangConstraint[0], false );
	mHangConstraint[0]->m_setting.m_impulseClamp = 0;
	mHangConstraint[0]->m_setting.m_tau          = 0.01;

	mHangPivot[1] = bodyPos + offset;
//	pivot    = getBody( BODYPART_BODY )->getCenterOfMassTransform().inverse() * mHangPivot[1];
	pivot = btVector3( 0, 0, 0 );
	mHangConstraint[1] = new btPoint2PointConstraint( *getBody( BODYPART_BODY ), pivot );
	mOwnerWorld->addConstraint( mHangConstraint[1], false );
	mHangConstraint[1]->m_setting.m_impulseClamp = 0;
	mHangConstraint[1]->m_setting.m_tau          = 0.01;

	mHangPivot[2] = leftLegPos + offset;
//	pivot    = getBody( BODYPART_LEG, 0 )->getCenterOfMassTransform().inverse() * mHangPivot[2];
	pivot = btVector3( 0, 0, 0 );
	mHangConstraint[2] = new btPoint2PointConstraint( *getBody( BODYPART_LEG, 0 ), pivot );
	mOwnerWorld->addConstraint( mHangConstraint[2], false );
	mHangConstraint[2]->m_setting.m_impulseClamp = 0;
	mHangConstraint[2]->m_setting.m_tau          = 0.01;

	mHangPivot[3] = rightLegPos + offset;
//	pivot    = getBody( BODYPART_LEG, mLegPart )->getCenterOfMassTransform().inverse() * mHangPivot[3];
	pivot = btVector3( 0, 0, 0 );
	mHangConstraint[3] = new btPoint2PointConstraint( *getBody( BODYPART_LEG, mLegPart ), pivot );
	mOwnerWorld->addConstraint( mHangConstraint[3], false );
	mHangConstraint[3]->m_setting.m_impulseClamp = 0;
	mHangConstraint[3]->m_setting.m_tau          = 0.01;

}

BulletBird::~BulletBird()
{
	// Remove all constraints
	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( *it );
		delete *it;
	}

	for( int i = 0; i < 4; ++i )
	{
		mOwnerWorld->removeConstraint( mHangConstraint[i] );
		delete mHangConstraint[i];
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
// 	ci::Vec3f cross = dir.cross( norm );
// 	cross.normalize();

// 	ci::Vec3f posHang0 = pos + dir   * mStickSize;
// 	ci::Vec3f posHang1 = pos - dir   * mStickSize;
// 	ci::Vec3f posHang2 = pos - cross * mStickSize;
// 	ci::Vec3f posHang3 = pos + cross * mStickSize;
// 
// 	posHang0.y -= mHangPivot[0];
// 	posHang1.y -= mHangPivot[1];
// 	posHang2.y -= mHangPivot[2];
// 	posHang3.y -= mHangPivot[3];
// 
// 	mHangConstraint[0]->setPivotB( CinderBullet::convert( posHang0 ) );
// 	mHangConstraint[1]->setPivotB( CinderBullet::convert( posHang1 ) );
// 	mHangConstraint[2]->setPivotB( CinderBullet::convert( posHang2 ) );
// 	mHangConstraint[3]->setPivotB( CinderBullet::convert( posHang3 ) );

// 	setPos( getBody( BODYPART_HANG, 0 ), posHang0 );
// 	setPos( getBody( BODYPART_HANG, 1 ), posHang1 );
// 	setPos( getBody( BODYPART_HANG, 2 ), posHang2 );
// 	setPos( getBody( BODYPART_HANG, 3 ), posHang3 );

	Quatf rotate1 = Quatf( -Vec3f::zAxis(), dir.normalized() );
	Quatf rotate2 = Quatf( -Vec3f::yAxis(), norm.normalized() );

	Quatf rotate = rotate1 * rotate2;

	for( int i = 0; i < 4; ++i )
	{
		Vec3f pos = rotate * mHangPivot[ i ];
// 		rotateConstraint( mHangConstraint[i], pos );
		mHangConstraint[i]->setPivotB( CinderBullet::convert( pos ) );
	}
}

// void BulletBird::rotateConstraint( btPoint2PointConstraint *hangConstraint, Quatf rotate )
// {
// 	Vec3f pos = CinderBullet::convert( hangConstraint->getRigidBodyA().getCenterOfMassTransform()( hangConstraint->getPivotInA() ) );
// 
// 	hangConstraint->setPivotB( CinderBullet::convert( rotate * pos ));
// }

void BulletBird::setPos( btRigidBody *rigidBody, ci::Vec3f &pos )
{
// 	rigidBody->translate( CinderBullet::convert( pos ));
// 	btMotionState *motionState = rigidBody->getMotionState();
// 
// 	if( motionState )
// 	{
// 		btTransform trans;
// 		motionState->getWorldTransform( trans );
// 
// 		trans.setOrigin( CinderBullet::convert( pos ));
// 
// 		motionState->setWorldTransform( trans );
// 
// 		rigidBody->translate( btVector3( CinderBullet::convert( pos ) - trans.getOrigin()));
// 	}
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
	case BODYPART_FOOT : return mBodies[ 0 + count                    ]; break;
	case BODYPART_LEG  : return mBodies[ 2 + count                    ]; break;
	case BODYPART_BODY : return mBodies[ 2 + 2 * mLegPart             ]; break;
	case BODYPART_NECK : return mBodies[ 3 + 2 * mLegPart + count     ]; break;
	case BODYPART_HEAD : return mBodies[ 3 + 2 * mLegPart + mNeckPart ]; break;
	case BODYPART_BECK : return mBodies[ 4 + 2 * mLegPart + mNeckPart ]; break;
	}

	return 0;
}