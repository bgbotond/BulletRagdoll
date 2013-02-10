#include "cinder/CinderMath.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "CinderBullet.h"
#include "BulletBird.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

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

mndl::kit::params::PInterfaceGl   BulletBird::mParamsBird;
float                             BulletBird::mBeckSize   = 1.0; // cylinder shape
float                             BulletBird::mHeadSize   = 2.0; // sphere shape
float                             BulletBird::mNeckSize   = 1.0; // sphere shape
float                             BulletBird::mBodySize   = 3.0; // sphere shape
float                             BulletBird::mLegSize    = 1.0; // cylinder shape
float                             BulletBird::mFootSize   = 0.3; // sphere shape
float                             BulletBird::mStickSize  = 4.0; // control cross size

int                               BulletBird::mNeckPart   = 4  ; // count of neck sphere
int                               BulletBird::mLegPart    = 4  ; // count of leg  sphere
float                             BulletBird::mStringSize = 0.0; // size of string from head


float                             BulletBird::mLinearDamping             = 0.85;
float                             BulletBird::mAngularDamping            = 0.85;
float                             BulletBird::mDeactivationTime          = 0.8;
float                             BulletBird::mLinearSleepingThresholds  = 1.6;
float                             BulletBird::mAngularSleepingThresholds = 2.5;

float                             BulletBird::mDamping = 0.01;
// float                             BulletBird::mLinCFM  = 0.0;
// float                             BulletBird::mLinERP  = 0.7;
// float                             BulletBird::mAngCFM  = 0.0f;


mndl::kit::params::PInterfaceGl   BulletBird::mParamsRope;
int                               BulletBird::mRopePart    = 16;     // rope part count
float                             BulletBird::mRopeMass    = 5.0;    // mass
float                             BulletBird::mKVCF        = 1.0;    // Velocities correction factor (Baumgarte)
float                             BulletBird::mKDP         = 0.0;    // Damping coefficient [0,1]
float                             BulletBird::mKDG         = 0.0;    // Drag coefficient [0,+inf]
float                             BulletBird::mKLF         = 0.0;    // Lift coefficient [0,+inf]
float                             BulletBird::mKPR         = 0.0;    // Pressure coefficient [-inf,+inf]
float                             BulletBird::mKVC         = 0.0;    // Volume conversation coefficient [0,+inf]
float                             BulletBird::mKDF         = 0.2;    // Dynamic friction coefficient [0,1]
float                             BulletBird::mKMT         = 0.0;    // Pose matching coefficient [0,1]
float                             BulletBird::mKCHR        = 1.0;    // Rigid contacts hardness [0,1]
float                             BulletBird::mKKHR        = 0.1;    // Kinetic contacts hardness [0,1]
float                             BulletBird::mKSHR        = 1.0;    // Soft contacts hardness [0,1]
float                             BulletBird::mKAHR        = 0.7;    // Anchors hardness [0,1]
float                             BulletBird::mMaxvolume   = 1.0;    // Maximum volume ratio for pose
float                             BulletBird::mTimescale   = 1.0;    // Time scale
int                               BulletBird::mViterations = 0;      // Velocities solver iterations
int                               BulletBird::mPiterations = 4;      // Positions solver iterations
int                               BulletBird::mDiterations = 0;      // Drift solver iterations
int                               BulletBird::mCiterations = 4;      // Cluster solver iterations


// float                             BulletBird::mTau          = 0.01;
// float                             BulletBird::mDamping      = 1.0;
// float                             BulletBird::mImpulseClamp = 0.0;


BulletBird::BulletBird( btDynamicsWorld *ownerWorld, btSoftBodyWorldInfo *softBodyWorldInfo, const ci::Vec3f &worldOffset )
: mOwnerWorld( ownerWorld )
, mSoftBodyWorldInfo( softBodyWorldInfo )
{
	float hangSize = 0.2;

	mShapes.push_back( new btConeShape    ( mBeckSize / 2, 2 * mBeckSize                          )); // -> BODYPART_BECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mHeadSize                                 ))); // -> BODYPART_HEAD
	mShapes.push_back( new btSphereShape  ( btScalar ( mNeckSize                                 ))); // -> BODYPART_NECK
	mShapes.push_back( new btSphereShape  ( btScalar ( mBodySize                                 ))); // -> BODYPART_BODY
	mShapes.push_back( new btSphereShape  ( btScalar ( mLegSize                                  ))); // -> BODYPART_LEG
	mShapes.push_back( new btCylinderShape( btVector3( mLegSize * 2.5, mFootSize, mLegSize * 2.5 ))); // -> BODYPART_FOOT
	mShapes.push_back( new btCompoundShape(                                                       )); // -> BODYPART_CROSS
	mShapes.push_back( new btBoxShape     ( btVector3( hangSize, hangSize, mStickSize + 2 * hangSize ))); // -> BODYPART_HANG

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

	mPosHangCenter     = Vec3f( 0, headPos.y + mStringSize, 0 ) + offset;
	mPosHangFront      = mPosHangCenter + Vec3f( 0, 0, -mStickSize );
	mPosHangBack       = mPosHangCenter + Vec3f( 0, 0,  mStickSize );
	mPosHangLeft       = mPosHangCenter + Vec3f( -mStickSize, 0, 0 );
	mPosHangRight      = mPosHangCenter + Vec3f(  mStickSize, 0, 0 );

	Vec3f pos;
	Vec3f orient;
	Quatf rotate;
	btTransform transform;

	// left foot
	pos = leftFootPos + worldOffset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( mFootSize , transform, mShapes[ BODYPART_FOOT ] ));

	// right foot
	pos = rightFootPos + worldOffset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( mFootSize , transform, mShapes[ BODYPART_FOOT ] ));

	// left legs
	orient = bodyPos - leftLegPos;
	orient.normalize();
	pos = leftLegPos + offset;
	for( int i = 0; i < mLegPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mLegSize * orient ));
		mRigidBodies.push_back( localCreateRigidBody( mLegSize , transform, mShapes[ BODYPART_LEG ] ));
	}

	// right legs
	orient = bodyPos - rightLegPos;
	orient.normalize();
	pos = rightLegPos + offset;
	for( int i = 0; i < mLegPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mLegSize * orient ));
		mRigidBodies.push_back( localCreateRigidBody( mLegSize , transform, mShapes[ BODYPART_LEG ] ));
	}

	// body
	pos = bodyPos + offset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( mBodySize , transform, mShapes[ BODYPART_BODY ] ));

	// neck
	orient = headPos - bodyPos;
	orient.normalize();
	pos = bodyPos + offset + orient * ( mBodySize + mNeckSize );
	for( int i = 0; i < mNeckPart; ++i )
	{
		transform.setIdentity();
		transform.setOrigin( CinderBullet::convert( pos + i * 2 * mNeckSize * orient ));
		mRigidBodies.push_back( localCreateRigidBody( mNeckSize , transform, mShapes[ BODYPART_NECK ] ));
	}

	// head
	pos = headPos + offset;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( mHeadSize , transform, mShapes[ BODYPART_HEAD ] ));

	// beck
	pos    = beckPos + offset;
	rotate = Quatf( RADIAN( -90 ), 0, 0 );
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( mBeckSize , transform, mShapes[ BODYPART_BECK ] ));

	// hang
	btCompoundShape* compShape = (btCompoundShape*)mShapes[ BODYPART_CROSS ];

	pos = Vec3f( 0, 0, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	compShape->addChildShape( transform, mShapes[ BODYPART_HANG ] );

	pos = Vec3f( 0, 0, 0 );
	rotate = Quatf( Vec3f::zAxis(), Vec3f::xAxis());
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( pos ));
	compShape->addChildShape( transform, mShapes[ BODYPART_HANG ] );

	pos = mPosHangCenter;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.push_back( localCreateRigidBody( hangSize * 4, transform, compShape ));

	getBody( BODYPART_CROSS )->setCollisionFlags( getBody( BODYPART_CROSS )->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );

	// Setup some damping on the mRigidBodies
	for( RigidBodies::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
	{
		btRigidBody *body = *it;
		body->setDamping( mLinearDamping, mAngularDamping );
		body->setDeactivationTime( mDeactivationTime );
		body->setSleepingThresholds( mLinearSleepingThresholds, mAngularSleepingThresholds );
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

		constraint->setDamping( mDamping );
	}

	btSoftBody *rope;
	Vec3f from, to;
	btRigidBody *rigidBodyFrom, *rigidBodyTo;

	from = headPos + offset;
	to   = mPosHangFront;
	rigidBodyFrom = getBody( BODYPART_HEAD    );
	rigidBodyTo   = getBody( BODYPART_CROSS   );
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = bodyPos + offset;
	to   = mPosHangBack;
	rigidBodyFrom = getBody( BODYPART_BODY    );
	rigidBodyTo   = getBody( BODYPART_CROSS   );
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = leftLegPos + offset;
	to   = mPosHangLeft;
	rigidBodyFrom = getBody( BODYPART_LEG , 0 );
	rigidBodyTo   = getBody( BODYPART_CROSS   );
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = rightLegPos + offset;
	to   = mPosHangRight;
	rigidBodyFrom = getBody( BODYPART_LEG , mLegPart );
	rigidBodyTo   = getBody( BODYPART_CROSS   );
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );
}

BulletBird::~BulletBird()
{
	// Remove all constraints
	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( *it );
		delete *it;
	}

	for( RigidBodies::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
	{
		btRigidBody *rigidBody = *it;

		mOwnerWorld->removeRigidBody( rigidBody );

		if( rigidBody->getMotionState())
			delete rigidBody->getMotionState();

		delete rigidBody;
	}

	for( SoftBodies::iterator it = mSoftBodies.begin(); it != mSoftBodies.end(); ++it )
	{
		btSoftBody *softBody = *it;

		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( softBody );

		delete softBody;
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

	ci::Vec3f cross = dir.cross( norm ).normalized();

	Quatf quatZ( -Vec3f::zAxis(), dir );
	Quatf quatY( -Vec3f::yAxis(), norm );
	Quatf rotate = quatZ * quatY;
	Vec3f posCenter    = mPosHangCenter;

	btTransform transform;
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( posCenter ));
	btMotionState *motionState = getBody( BODYPART_CROSS )->getMotionState();
	motionState->setWorldTransform( transform );

	mSoftBodies[0]->m_nodes[mSoftBodies[0]->m_nodes.size() - 1].m_x = CinderBullet::convert( mPosHangCenter + dir   * mStickSize );
	mSoftBodies[1]->m_nodes[mSoftBodies[1]->m_nodes.size() - 1].m_x = CinderBullet::convert( mPosHangCenter - dir   * mStickSize );
	mSoftBodies[2]->m_nodes[mSoftBodies[2]->m_nodes.size() - 1].m_x = CinderBullet::convert( mPosHangCenter + cross * mStickSize );
	mSoftBodies[3]->m_nodes[mSoftBodies[3]->m_nodes.size() - 1].m_x = CinderBullet::convert( mPosHangCenter - cross * mStickSize );
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

btSoftBody *BulletBird::localCreateRope( const ci::Vec3f &from, const ci::Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo )
{
	btSoftBody *rope = btSoftBodyHelpers::CreateRope( *mSoftBodyWorldInfo
	                                                ,  CinderBullet::convert( from )
	                                                ,  CinderBullet::convert( to )
	                                                ,  mRopePart
	                                                ,  2 );

	rope->setTotalMass( mRopeMass );

	rope->m_cfg.kVCF        = mKVCF;
	rope->m_cfg.kDP         = mKDP;
	rope->m_cfg.kDG         = mKDG;
	rope->m_cfg.kLF         = mKLF;
	rope->m_cfg.kPR         = mKPR;
	rope->m_cfg.kVC         = mKVC;
	rope->m_cfg.kDF         = mKDF;
	rope->m_cfg.kMT         = mKMT;
	rope->m_cfg.kCHR        = mKCHR;
	rope->m_cfg.kKHR        = mKKHR;
	rope->m_cfg.kSHR        = mKSHR;
	rope->m_cfg.kAHR        = mKAHR;
	rope->m_cfg.maxvolume   = mMaxvolume;
	rope->m_cfg.timescale   = mTimescale;
	rope->m_cfg.viterations = mViterations;
	rope->m_cfg.piterations = mPiterations;
	rope->m_cfg.diterations = mDiterations;
	rope->m_cfg.citerations = mCiterations;

	rope->appendAnchor( 0                       , rigidBodyFrom );
	rope->appendAnchor( rope->m_nodes.size() - 1, rigidBodyTo   );

	((btSoftRigidDynamicsWorld*)mOwnerWorld)->addSoftBody( rope );

	return rope;
}

btCollisionShape *BulletBird::getShape( BodyPart bodyPart )
{
	return mShapes[ bodyPart ];
}

btRigidBody *BulletBird::getBody( BodyPart bodyPart, int count /* = 0 */ )
{
	switch( bodyPart )
	{
	case BODYPART_FOOT  : return mRigidBodies[ 0 + count                            ]; break;
	case BODYPART_LEG   : return mRigidBodies[ 2 + count                            ]; break;
	case BODYPART_BODY  : return mRigidBodies[ 2 + 2 * mLegPart                     ]; break;
	case BODYPART_NECK  : return mRigidBodies[ 3 + 2 * mLegPart + count             ]; break;
	case BODYPART_HEAD  : return mRigidBodies[ 3 + 2 * mLegPart + mNeckPart         ]; break;
	case BODYPART_BECK  : return mRigidBodies[ 4 + 2 * mLegPart + mNeckPart         ]; break;
	case BODYPART_CROSS : return mRigidBodies[ 5 + 2 * mLegPart + mNeckPart         ]; break;
	case BODYPART_HANG  : return 0;
	}

	return 0;
}

void BulletBird::setupParams()
{
	mParamsBird = mndl::kit::params::PInterfaceGl( "Bird", ci::Vec2i( 250, 350 ), ci::Vec2i( 500, 50 ) );
	mParamsBird.addPersistentSizeAndPosition();

	mParamsBird.addText( "Body" );
	mParamsBird.addPersistentParam( "Beck size" , &mBeckSize , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParamsBird.addPersistentParam( "Head size" , &mHeadSize , 2.0f, "min=0.5 max=5.0 step=0.1" );
	mParamsBird.addPersistentParam( "Neck size" , &mNeckSize , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParamsBird.addPersistentParam( "Body size" , &mBodySize , 3.0f, "min=0.5 max=5.0 step=0.1" );
	mParamsBird.addPersistentParam( "Leg size"  , &mLegSize  , 1.0f, "min=0.5 max=5.0 step=0.1" );
	mParamsBird.addPersistentParam( "Foot size" , &mFootSize , 0.3f, "min=0.1 max=3.0 step=0.1" );
	mParamsBird.addPersistentParam( "Stick size", &mStickSize, 4.0f, "min=0.5 max=5.0 step=0.1" );

	mParamsBird.addPersistentParam( "Neck part" , &mNeckPart , 4   , "min=2 max=8 step=1" );
	mParamsBird.addPersistentParam( "Leg part"  , &mLegPart  , 4   , "min=2 max=8 step=1" );

	mParamsBird.addText( "RigidBody" );
	mParamsBird.addPersistentParam( "Linear damping"             , &mLinearDamping            , 0.85 , "min=0.0 max=1.0 step=0.1" );
	mParamsBird.addPersistentParam( "Angular damping"            , &mAngularDamping           , 0.85 , "min=0.0 max=1.0 step=0.1" );
	mParamsBird.addPersistentParam( "Deactivation time"          , &mDeactivationTime         , 0.8  , "min=0.0         step=0.1" );
	mParamsBird.addPersistentParam( "Linear sleeping thresholds" , &mLinearSleepingThresholds , 1.6  , "min=0.0         step=0.1" );
	mParamsBird.addPersistentParam( "Angular sleeping thresholds", &mAngularSleepingThresholds, 2.5  , "min=0.0         step=0.1" );

	mParamsBird.addText( "Constraint" );
	mParamsBird.addPersistentParam( "Damping"                    , &mDamping                  , 0.01 , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinCFM"                     , &mLinCFM                   , 0.0  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinERP"                     , &mLinERP                   , 0.7  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "AngCFM"                     , &mAngCFM                   , 0.0f , "min=0.0 max=1.0 step=0.01" );

	mParamsRope = mndl::kit::params::PInterfaceGl( "Rope", ci::Vec2i( 250, 350 ), ci::Vec2i( 700, 50 ) );
	mParamsRope.addPersistentSizeAndPosition();
	mParamsRope.addPersistentParam( "String size"                              , &mStringSize, 5.0f, "min=0.0 max=5.0 step=0.1" );
	mParamsRope.addPersistentParam( "Part"                                     , &mRopePart    , 16,   "min=4 max=50 step=1"         );
	mParamsRope.addPersistentParam( "Mass"                                     , &mRopeMass    , 5.0,  "min=0.01 max=20.0 step=0.01" );
	mParamsRope.addPersistentParam( "Velocities correction factor (Baumgarte)" , &mKVCF        , 1.0,  "min=0.0  max=20.0 step=0.1"  );
	mParamsRope.addPersistentParam( "Damping coefficient [0,1]"                , &mKDP         , 0.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Drag coefficient [0,+inf]"                , &mKDG         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Lift coefficient [0,+inf]"                , &mKLF         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Pressure coefficient [-inf,+inf]"         , &mKPR         , 0.0,  "                  step=0.01" );
	mParamsRope.addPersistentParam( "Volume conversation coefficient [0,+inf]" , &mKVC         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Dynamic friction coefficient [0,1]"       , &mKDF         , 0.2,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Pose matching coefficient [0,1]"          , &mKMT         , 0.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Rigid contacts hardness [0,1]"            , &mKCHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Kinetic contacts hardness [0,1]"          , &mKKHR        , 0.1,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Soft contacts hardness [0,1]"             , &mKSHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Anchors hardness [0,1]"                   , &mKAHR        , 0.7,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Maximum volume ratio for pose"            , &mMaxvolume   , 1.0   );
	mParamsRope.addPersistentParam( "Time scale"                               , &mTimescale   , 1.0   );
	mParamsRope.addPersistentParam( "Velocities solver iterations"             , &mViterations , 0     );
	mParamsRope.addPersistentParam( "Positions solver iterations"              , &mPiterations , 4     );
	mParamsRope.addPersistentParam( "Drift solver iterations"                  , &mDiterations , 0     );
	mParamsRope.addPersistentParam( "Cluster solver iterations"                , &mCiterations , 4     );

// 	mParams.addText( "Hang constraints" );
// 	mParams.addPersistentParam( "Tau"          , &mTau         , 0.01f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Damping"      , &mDamping     , 1.0f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Impulse clamp", &mImpulseClamp, 0.0f,  "min=0.0 max=10.0 step=0.1" );
}
