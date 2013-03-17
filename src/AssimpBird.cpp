#include "cinder/app/App.h"
#include "cinder/CinderMath.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "CinderBullet.h"
#include "AssimpBird.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletWorld.h"


#ifndef CONSTRAINT_DEBUG_SIZE
#define CONSTRAINT_DEBUG_SIZE 0.5f
#endif

#ifndef RADIAN
#define RADIAN( x ) ( (float)(x) * M_PI / 180  )
#endif

using namespace ci;
using namespace mndl;
using namespace mndl::assimp;

mndl::kit::params::PInterfaceGl   AssimpBird::mParamsBird;

float                             AssimpBird::mLinearDamping             = 0.85;
float                             AssimpBird::mAngularDamping            = 0.85;
float                             AssimpBird::mDeactivationTime          = 0.8;
float                             AssimpBird::mLinearSleepingThresholds  = 1.6;
float                             AssimpBird::mAngularSleepingThresholds = 2.5;

float                             AssimpBird::mDamping = 0.01;
float                             AssimpBird::mStopCMF = 0.85;
float                             AssimpBird::mStopERP = 0.65;
// float                             AssimpBird::mLinCFM  = 0.0;
// float                             AssimpBird::mLinERP  = 0.7;
// float                             AssimpBird::mAngCFM  = 0.0f;


mndl::kit::params::PInterfaceGl   AssimpBird::mParamsRope;
int                               AssimpBird::mRopePart    = 16;     // rope part count
float                             AssimpBird::mRopeMass    = 5.0;    // mass
float                             AssimpBird::mKVCF        = 1.0;    // Velocities correction factor (Baumgarte)
float                             AssimpBird::mKDP         = 0.0;    // Damping coefficient [0,1]
float                             AssimpBird::mKDG         = 0.0;    // Drag coefficient [0,+inf]
float                             AssimpBird::mKLF         = 0.0;    // Lift coefficient [0,+inf]
float                             AssimpBird::mKPR         = 0.0;    // Pressure coefficient [-inf,+inf]
float                             AssimpBird::mKVC         = 0.0;    // Volume conversation coefficient [0,+inf]
float                             AssimpBird::mKDF         = 0.2;    // Dynamic friction coefficient [0,1]
float                             AssimpBird::mKMT         = 0.0;    // Pose matching coefficient [0,1]
float                             AssimpBird::mKCHR        = 1.0;    // Rigid contacts hardness [0,1]
float                             AssimpBird::mKKHR        = 0.1;    // Kinetic contacts hardness [0,1]
float                             AssimpBird::mKSHR        = 1.0;    // Soft contacts hardness [0,1]
float                             AssimpBird::mKAHR        = 0.7;    // Anchors hardness [0,1]
float                             AssimpBird::mMaxvolume   = 1.0;    // Maximum volume ratio for pose
float                             AssimpBird::mTimescale   = 1.0;    // Time scale
int                               AssimpBird::mViterations = 0;      // Velocities solver iterations
int                               AssimpBird::mPiterations = 4;      // Positions solver iterations
int                               AssimpBird::mDiterations = 0;      // Drift solver iterations
int                               AssimpBird::mCiterations = 4;      // Cluster solver iterations

float                             AssimpBird::MIN_BONE_LENGTH = 2.0f;
float                             AssimpBird::CAPSULE_RADIUS  = 0.1f;

bool                              AssimpBird::mDrawSkin = true;
bool                              AssimpBird::mEnableWireframe = false;

// float                             AssimpBird::mTau          = 0.01;
// float                             AssimpBird::mDamping      = 1.0;
// float                             AssimpBird::mImpulseClamp = 0.0;


AssimpBird::AssimpBird( btDynamicsWorld *ownerWorld, btSoftBodyWorldInfo *softBodyWorldInfo, const Vec3f &worldOffset, const boost::filesystem::path &file )
: mOwnerWorld( ownerWorld )
, mSoftBodyWorldInfo( softBodyWorldInfo )
, mFile( file )
{
	mAssimpLoader = AssimpLoader( file );
	mAssimpLoader.enableSkinning( true );
	mAssimpLoader.enableAnimation( true );

	Vec3f pos = Vec3f( 0, 15, 0 );

	assimp::AssimpNodeRef scene = mAssimpLoader.getAssimpNode( "Scene" );
	scene->setPosition( pos );

	defineBones();

	AssimpNodeRef node = mAssimpLoader.getRootNode();

	buildBones( node );
	buildJoints( node );

	buildHang();
}

void AssimpBird::defineBones()
{
	mBones.insert( std::make_pair( "body"   , "head"   ) );
	mBones.insert( std::make_pair( "head"   , "T_beak" ) );
	mBones.insert( std::make_pair( "T_beak" , ""       ) );
	mBones.insert( std::make_pair( "B_beak" , ""       ) );
	mBones.insert( std::make_pair( "L_wing" , ""       ) );
	mBones.insert( std::make_pair( "R_wing" , ""       ) );
	mBones.insert( std::make_pair( "L_thigh", "L_shin" ) );
	mBones.insert( std::make_pair( "L_shin" , "L_foot" ) );
	mBones.insert( std::make_pair( "L_foot" , ""       ) );
	mBones.insert( std::make_pair( "R_thigh", "R_shin" ) );
	mBones.insert( std::make_pair( "R_shin" , "R_foot" ) );
	mBones.insert( std::make_pair( "R_foot" , ""       ) );
	mBones.insert( std::make_pair( "tail"   , ""       ) );
}

void AssimpBird::buildBones( const NodeRef &node )
{
	if( ! filterNode( node ))
		buildBone( node );

	for( std::vector< NodeRef >::const_iterator it = node->getChildren().begin(); it != node->getChildren().end(); ++it )
	{
		NodeRef child = *it;

		buildBones( child );
	}
}

void AssimpBird::buildJoints( const NodeRef &node )
{
	for( std::vector< NodeRef >::const_iterator it = node->getChildren().begin(); it != node->getChildren().end(); ++it )
	{
		NodeRef child = *it;
		buildJoint( child );
		buildJoints( child );
	}
}

void AssimpBird::updateBones( const NodeRef &node )
{
	if( ! filterNode( node ))
		updateBone( node );

	for( std::vector< NodeRef >::const_iterator it = node->getChildren().begin(); it != node->getChildren().end(); ++it )
	{
		NodeRef child = *it;

		updateBones( child );
	}
}

void AssimpBird::buildBone( const NodeRef &node )
{
	float length = getLength( node );

	Vec3f pos = node->getDerivedPosition();

	if( node->getName() == "tail" )
		pos = node->getParent()->getDerivedPosition();

	Quatf rot = node->getDerivedOrientation();
	btCollisionShape  *shape = new btCapsuleShape( CAPSULE_RADIUS, length /*- 2 * CAPSULE_RADIUS */);
	mShapes.push_back( shape );

	Vec3f move = Vec3f::yAxis() * rot * ( length / 2 /*- CAPSULE_RADIUS */);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos + move ));
	transform.setRotation( CinderBullet::convert( rot ) );
	mRigidBodies.insert( std::make_pair( node->getName(), localCreateRigidBody( 1.0f, transform, shape ) ) );

	btRigidBody *rigidBody = mRigidBodies[ node->getName() ];

	rigidBody->setDamping( mLinearDamping, mAngularDamping );
	rigidBody->setDeactivationTime( mDeactivationTime );
	rigidBody->setSleepingThresholds( mLinearSleepingThresholds, mAngularSleepingThresholds );

	updateBone( node );
}

void AssimpBird::buildJoint( const NodeRef &nodeB )
{
	NodeRef nodeA = nodeB->getParent();

	if( filterNode( nodeA )
	 || filterNode( nodeB ) )
		return;

	btRigidBody *rigidBodyA = mRigidBodies[ nodeA->getName() ];
	btRigidBody *rigidBodyB = mRigidBodies[ nodeB->getName() ];
	float        sizeA      = getLength( nodeA );
	float        sizeB      = getLength( nodeB );

	btConeTwistConstraint   *coneC;
	btTransform              localA, localB;

	localA.setIdentity(); localB.setIdentity();

	int sign = 1;

	if( nodeB->getName() == "tail" )
		sign = -1;

	Quatf rot = Quatf( Vec3f::xAxis(), Vec3f::yAxis() );

	localA.setRotation( CinderBullet::convert( rot ) ); localA.setOrigin( CinderBullet::convert( Vec3f( 0,  sign * sizeA / 2, 0 ) ) );
	localB.setRotation( CinderBullet::convert( rot * nodeB->getOrientation().inverse() ) ); localB.setOrigin( CinderBullet::convert( Vec3f( 0, -sizeB / 2, 0 ) ) );
	coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	/**/ if( nodeB->getName() == "T_beak" )
		coneC->setLimit( RADIAN( 20 ), 0, 0 );
	else if( nodeB->getName() == "B_beak" )
		coneC->setLimit( RADIAN( 20 ), 0, 0 );
	else
		coneC->setLimit( RADIAN( 20 ), RADIAN( 20 ), 0 );
	mConstraints.push_back( coneC );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

	coneC->setDamping( mDamping );
	for( int i = 0; i < 6; ++i )
	{
		coneC->setParam( BT_CONSTRAINT_STOP_CFM, mStopCMF, i );
		coneC->setParam( BT_CONSTRAINT_STOP_ERP, mStopERP, i );
	}

	mOwnerWorld->addConstraint( coneC, true );
}

void AssimpBird::updateBone( const NodeRef &node )
{
	float length = getLength( node );
	btRigidBody *rigidBody = mRigidBodies[ node->getName() ];

	btMotionState *motionState = rigidBody->getMotionState();

	btTransform worldTransform;
	motionState->getWorldTransform( worldTransform );

	Vec3f pos = CinderBullet::convert( worldTransform.getOrigin() );
	Quatf rot = CinderBullet::convert( worldTransform.getRotation() );

	Vec3f move = Vec3f::yAxis() * rot * ( length / 2 );

	pos = node->convertWorldToLocalPosition( pos - move );
	rot = node->convertWorldToLocalOrientation( rot );

	node->setPosition( pos );
	node->setOrientation( rot );
}

float AssimpBird::getLength( const NodeRef &node )
{
	Vec3f pos = node->getDerivedPosition();
	float lenght = MIN_BONE_LENGTH;

	std::string name = mBones[ node->getName() ];

	if( ! name.empty() )
		lenght = ( mAssimpLoader.getAssimpNode( name )->getDerivedPosition() - pos ).length();

	return lenght;
}

bool AssimpBird::filterNode( const NodeRef &node )
{
	if( node->getName() == "Scene"    ) return true;
	if( node->getName() == "madar"    ) return true;
	if( node->getName() == "Armature" ) return true;

	return false;
}

void AssimpBird::buildHang()
{
	float hangSize = 0.2;

	NodeRef nodeFront  = mAssimpLoader.getAssimpNode( "body"   );
	NodeRef nodeBack   = mAssimpLoader.getAssimpNode( "T_beak" );
	NodeRef nodeLeft   = mAssimpLoader.getAssimpNode( "L_foot" );
	NodeRef nodeRight  = mAssimpLoader.getAssimpNode( "R_foot" );
	NodeRef nodeCenter = mAssimpLoader.getAssimpNode( "head"   );

	// hang body
	btCompoundShape *compShape = new btCompoundShape();
	mShapes.push_back( compShape );

	float height = nodeFront->getDerivedPosition().y + 5;

	mHangCenter     = Vec3f( nodeLeft->getDerivedPosition().x, height, 0 );
	mHangSizeFront  = nodeFront->getDerivedPosition().x - nodeLeft->getDerivedPosition().x;
	mHangSizeBack   = nodeLeft->getDerivedPosition().x - nodeBack->getDerivedPosition().x;
	mHangSizeLeft   = nodeLeft ->getDerivedPosition().z;
	mHangSizeRight  = nodeRight->getDerivedPosition().z;
	mHangSizeMiddle = nodeLeft->getDerivedPosition().x - nodeCenter->getDerivedPosition().x;

	float lengthFrontBack = ( nodeFront->getDerivedPosition().x - nodeBack->getDerivedPosition().x ) / 2;
	float lengthLeftRight = ( nodeLeft->getDerivedPosition().z - nodeRight->getDerivedPosition().z ) / 2;
	Vec3f center = Vec3f( nodeLeft->getDerivedPosition().x, height, 0 );

	Vec3f hangPosFront  = Vec3f( nodeFront ->getDerivedPosition().x, height, 0 );
	Vec3f hangPosBack   = Vec3f( nodeBack  ->getDerivedPosition().x, height, 0 );
	Vec3f hangPosLeft   = Vec3f( nodeLeft  ->getDerivedPosition().x, height, nodeLeft ->getDerivedPosition().z );
	Vec3f hangPosRight  = Vec3f( nodeRight ->getDerivedPosition().x, height, nodeRight->getDerivedPosition().z );
	Vec3f hangPosCenter = Vec3f( nodeCenter->getDerivedPosition().x, height, 0 );

	float offsetX = lengthFrontBack - ( nodeFront->getDerivedPosition().x - nodeLeft->getDerivedPosition().x );

	btBoxShape *hangShapeFrontBack = new btBoxShape( btVector3( hangSize, hangSize, lengthFrontBack + 2 * hangSize ));
	btBoxShape *hangShapeLeftRight = new btBoxShape( btVector3( hangSize, hangSize, lengthLeftRight + 2 * hangSize ));

	Vec3f pos;
	Quatf rot;
	btTransform transform;

	pos = Vec3f( -offsetX, 0, 0 );
	rot = Quatf( Vec3f::zAxis(), Vec3f::xAxis());
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rot ));
	transform.setOrigin( CinderBullet::convert( pos ));
	compShape->addChildShape( transform, hangShapeFrontBack );

	pos = Vec3f( 0, 0, 0 );
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	compShape->addChildShape( transform, hangShapeLeftRight );

	pos = center;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mRigidBodies.insert( std::make_pair( "hang", localCreateRigidBody( hangSize * 4, transform, compShape )));

	mRigidBodies[ "hang" ]->setCollisionFlags( mRigidBodies[ "hang" ]->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );


	// hang rope
	btSoftBody *rope;
	Vec3f from, to;
	btRigidBody *rigidBodyFrom, *rigidBodyTo;

	from = nodeFront->getDerivedPosition();
	to   = hangPosFront;
	rigidBodyFrom = mRigidBodies[ "body" ];
	rigidBodyTo   = mRigidBodies[ "hang" ];
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = nodeBack->getDerivedPosition();
	to   = hangPosBack;
	rigidBodyFrom = mRigidBodies[ "head" ];
	rigidBodyTo   = mRigidBodies[ "hang" ];
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = nodeLeft->getDerivedPosition();
	to   = hangPosLeft;
	rigidBodyFrom = mRigidBodies[ "L_foot" ];
	rigidBodyTo   = mRigidBodies[ "hang" ];
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = nodeRight->getDerivedPosition();
	to   = hangPosRight;
	rigidBodyFrom = mRigidBodies[ "R_foot" ];
	rigidBodyTo   = mRigidBodies[ "hang" ];
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );

	from = nodeCenter->getDerivedPosition();
	to = hangPosCenter;
	rigidBodyFrom = mRigidBodies[ "body" ];
	rigidBodyTo   = mRigidBodies[ "hang" ];
	rope = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
	mSoftBodies.push_back( rope );
}

AssimpBird::~AssimpBird()
{
	// Remove all constraints
	for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
	{
		mOwnerWorld->removeConstraint( *it );
		delete *it;
	}

	for( RigidBodies::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
	{
		btRigidBody *rigidBody = it->second;

		mOwnerWorld->removeRigidBody( rigidBody );

		if( rigidBody->getMotionState() )
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

void AssimpBird::update( const Vec3f pos, const Vec3f dir, const Vec3f norm )
{
	AssimpNodeRef node = mAssimpLoader.getRootNode();
	updateBones( node );

	mAssimpLoader.update();

	updateHang( pos, dir, norm );
}

void AssimpBird::updateHang( const Vec3f pos, const Vec3f dir, const Vec3f norm )
{
	if( pos  == Vec3f::zero()
	 || dir  == Vec3f::zero()
	 || norm == Vec3f::zero())
		return;

	return; // disable for test

	Quatf rot = Quatf( -Vec3f::zAxis(), -Vec3f::xAxis() );
	Vec3f pos2  = rot * pos;
	Vec3f dir2  = rot * dir;
	Vec3f norm2 = rot * norm;

	ci::Vec3f cross = dir2.cross( norm2 ).normalized();

	Quatf quatX( -Vec3f::xAxis(), dir2  );
	Quatf quatY( -Vec3f::yAxis(), norm2 );
	Quatf rotate = quatY * quatX;
	Vec3f posCenter = mHangCenter;

	btTransform transform;
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( posCenter ));
	btMotionState *motionState = mRigidBodies[ "hang" ]->getMotionState();
	motionState->setWorldTransform( transform );

// 	mSoftBodies[ 0 ]->m_nodes[ mSoftBodies[ 0 ]->m_nodes.size() - 1 ].m_x = CinderBullet::convert( mHangCenter + dir2   * mHangSizeFront  );
// 	mSoftBodies[ 1 ]->m_nodes[ mSoftBodies[ 1 ]->m_nodes.size() - 1 ].m_x = CinderBullet::convert( mHangCenter - dir2   * mHangSizeBack   );
// 	mSoftBodies[ 2 ]->m_nodes[ mSoftBodies[ 2 ]->m_nodes.size() - 1 ].m_x = CinderBullet::convert( mHangCenter + cross  * mHangSizeLeft   );
// 	mSoftBodies[ 3 ]->m_nodes[ mSoftBodies[ 3 ]->m_nodes.size() - 1 ].m_x = CinderBullet::convert( mHangCenter - cross  * mHangSizeRight  );
// 	mSoftBodies[ 4 ]->m_nodes[ mSoftBodies[ 4 ]->m_nodes.size() - 1 ].m_x = CinderBullet::convert( mHangCenter - dir2   * mHangSizeMiddle );
}

void AssimpBird::draw()
{
	if( mDrawSkin )
	{
		if ( mEnableWireframe )
			gl::enableWireframe();

		mAssimpLoader.draw();

		if ( mEnableWireframe )
			gl::disableWireframe();
	}
}

btRigidBody *AssimpBird::localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if (isDynamic)
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	btRigidBody* body = new btRigidBody( rbInfo );

	mOwnerWorld->addRigidBody( body );///, CT_BONE, CT_GROUND );

	return body;
}

btSoftBody *AssimpBird::localCreateRope( const Vec3f &from, const Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo )
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

void AssimpBird::setupParams()
{
	mParamsBird = mndl::kit::params::PInterfaceGl( "AssimpBird", Vec2i( 250, 350 ), Vec2i( 500, 50 ) );
	mParamsBird.addPersistentSizeAndPosition();

	mParamsBird.addText( "RigidBody" );
	mParamsBird.addPersistentParam( "Linear damping"             , &mLinearDamping            , 0.85 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "Angular damping"            , &mAngularDamping           , 0.85 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "Deactivation time"          , &mDeactivationTime         , 0.8  , "min=0.0         step=0.01" );
	mParamsBird.addPersistentParam( "Linear sleeping thresholds" , &mLinearSleepingThresholds , 1.6  , "min=0.0         step=0.01" );
	mParamsBird.addPersistentParam( "Angular sleeping thresholds", &mAngularSleepingThresholds, 2.5  , "min=0.0         step=0.01" );

	mParamsBird.addText( "Constraint" );
	mParamsBird.addPersistentParam( "Damping"                    , &mDamping                  , 0.01 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "StopCMF"                    , &mStopCMF                  , 0.85 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "StopERP"                    , &mStopERP                  , 0.65 , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinCFM"                     , &mLinCFM                   , 0.0  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinERP"                     , &mLinERP                   , 0.7  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "AngCFM"                     , &mAngCFM                   , 0.0f , "min=0.0 max=1.0 step=0.01" );

	mParamsBird.addText( "Assimp" );
	mParamsBird.addPersistentParam( "DrawSkin"                  , &mDrawSkin, true );
	mParamsBird.addPersistentParam( "EnableWireframe"           , &mEnableWireframe, false );

	mParamsRope = mndl::kit::params::PInterfaceGl( "AssimpRope", Vec2i( 250, 350 ), Vec2i( 700, 50 ) );
	mParamsRope.addPersistentSizeAndPosition();
// 	mParamsRope.addPersistentParam( "String size"                              , &mStringSize, 5.0f, "min=0.0 max=5.0 step=0.1" );
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
	mParamsRope.addPersistentParam( "Time scale"                               , &mTimescale   , 3.0   );
	mParamsRope.addPersistentParam( "Velocities solver iterations"             , &mViterations , 0     );
	mParamsRope.addPersistentParam( "Positions solver iterations"              , &mPiterations , 4     );
	mParamsRope.addPersistentParam( "Drift solver iterations"                  , &mDiterations , 0     );
	mParamsRope.addPersistentParam( "Cluster solver iterations"                , &mCiterations , 4     );

// 	mParams.addText( "Hang constraints" );
// 	mParams.addPersistentParam( "Tau"          , &mTau         , 0.01f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Damping"      , &mDamping     , 1.0f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Impulse clamp", &mImpulseClamp, 0.0f,  "min=0.0 max=10.0 step=0.1" );
}


/*
node: Scene - parent NULL - position [0,0,0] - orientation [-1,0,0] @ 90deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]
node: Armature - parent Scene - position [0,0,0] - orientation [0,0,0] @ 0deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]
node: body - parent Armature - position [4.35206,3.07919e-008,0.511193] - orientation [-0.619478,0.555089,0.555089] @ 116.445deg - scale [1,1,1] - derivedposition [4.35206,0.511193,3.78914e-008] - derivedorientation [-0.744752,0.667342,-1.05367e-007] @ 180deg - derivedscale[1,1,1]
node: head - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.0335814,-0.999385,0.0101043] @ 146.527deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.277732,0.266212,0.923036] @ 96.9984deg - derivedscale[1,1,1]
node: T_beak - parent head - position [1.49012e-008,3.73539,0] - orientation [-0.0124969,-0.998007,0.061863] @ 44.2226deg - scale [1,1,1] - derivedposition [-5.93931,-0.368282,-1.04808e-007] - derivedorientation [0.092481,-0.0845222,0.992121] @ 95.6002deg - derivedscale[1,1,1]
node: B_beak - parent head - position [1.49012e-008,3.73539,0] - orientation [-0.0331506,-0.993761,0.106489] @ 32.5935deg - scale [1,1,1] - derivedposition [-5.93931,-0.368282,-1.04808e-007] - derivedorientation [-0.00952138,0.00857103,0.999918] @ 96.0181deg - derivedscale[1,1,1]
node: R_wing - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.981315,-0.168849,0.0922502] @ 98.3377deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.572018,0.634305,-0.52005] @ 100.679deg - derivedscale[1,1,1]
node: L_wing - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [-0.398123,0.652607,-0.644673] @ 160.161deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.79681,-0.516678,-0.313269] @ 272.286deg - derivedscale[1,1,1]
node: L_thigh - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.424907,0.534927,-0.730279] @ 132.128deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.748106,-0.226505,-0.623725] @ 184.246deg - derivedscale[1,1,1]
node: L_shin - parent L_thigh - position [-1.78814e-007,5.80307,-2.38419e-007] - orientation [-0.0487576,-0.962518,-0.266799] @ 160.472deg - scale [1,1,1] - derivedposition [-0.511278,-5.40265,1.95884] - derivedorientation [-0.723952,-0.187252,0.66395] @ 229.77deg - derivedscale[1,1,1]
node: L_foot - parent L_shin - position [1.19209e-007,4.40979,4.76837e-007] - orientation [0.960339,-0.110405,0.256047] @ 65.7811deg - scale [1,1,1] - derivedposition [2.70793,-7.99624,3.49383] - derivedorientation [-0.762621,0.2896,0.578395] @ 192.031deg - derivedscale[1,1,1]
node: R_thigh - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.686829,0.333574,-0.64575] @ 109.899deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.803286,-0.0107562,-0.595497] @ 152.638deg - derivedscale[1,1,1]
node: R_shin - parent R_thigh - position [-5.96046e-008,5.84806,2.38419e-007] - orientation [0.0471562,0.963878,0.262137] @ 176.829deg - scale [1,1,1] - derivedposition [-0.511278,-5.40264,-2.08835] - derivedorientation [0.556113,0.413624,-0.72087] @ 156.339deg - derivedscale[1,1,1]
node: R_foot - parent R_shin - position [3.57628e-007,4.40979,0] - orientation [0.786126,0.617735,0.0202287] @ 86.6796deg - scale [1,1,1] - derivedposition [2.70793,-7.99624,-3.62334] - derivedorientation [0.852226,-0.0072194,-0.523124] @ 215.685deg - derivedscale[1,1,1]
node: tail - parent body - position [-0.0284997,0.349762,-5.5053e-008] - orientation [0.999473,-0.0193096,-0.0260835] @ 180.058deg - scale [1,1,1] - derivedposition [4.00128,0.501289,7.99921e-008] - derivedorientation [-0.0260765,-0.0302575,-0.999202] @ 81.5561deg - derivedscale[1,1,1]
node: madar - parent Scene - position [0,0,0] - orientation [0,0,0] @ 0deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]


node: Scene - parent NULL
node: Armature - parent Scene
node: body - parent Armature
node: head - parent body
node: T_beak - parent head
node: B_beak - parent head
node: R_wing - parent body
node: L_wing - parent body
node: L_thigh - parent body
node: L_shin - parent L_thigh
node: L_foot - parent L_shin
node: R_thigh - parent body
node: R_shin - parent R_thigh
node: R_foot - parent R_shin
node: tail - parent body
node: madar - parent Scene


Scene
	madar
	Armature
		body
			head
				T_beak
				B_beak
			R_wing
			L_wing
			L_thigh
				L_shin
					L_foot
			R_thigh
				R_shin
					R_foot
			tail
 */