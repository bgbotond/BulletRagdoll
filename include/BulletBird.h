#ifndef __BulletBird_H__
#define __BulletBird_H__

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "mndlkit/params/PParams.h"

class BulletBird
{
	enum BodyPart
	{
		BODYPART_BECK = 0,
		BODYPART_HEAD,
		BODYPART_NECK,
		BODYPART_BODY,
		BODYPART_LEG,
		BODYPART_FOOT,
		BODYPART_CROSS,
		BODYPART_HANG,
	};

typedef std::vector< btCollisionShape  * > Shapes;
typedef std::vector< btRigidBody       * > RigidBodies;
typedef std::vector< btSoftBody        * > SoftBodies;
typedef std::vector< btTypedConstraint * > Constraints;

public:
	BulletBird( btDynamicsWorld *ownerWorld, btSoftBodyWorldInfo *softBodyWorldInfo, const ci::Vec3f &worldOffset );
	~BulletBird();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

	static void setupParams();

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );
	btSoftBody  *localCreateRope( const ci::Vec3f &from, const ci::Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo );

	btCollisionShape  *getShape( BodyPart bodyPart );
	btRigidBody       *getBody ( BodyPart bodyPart, int count = 0 );

protected:
	btDynamicsWorld     *mOwnerWorld;
	btSoftBodyWorldInfo *mSoftBodyWorldInfo;

	static mndl::kit::params::PInterfaceGl            mParamsBird;
	static float       mBeckSize;   // cylinder shape
	static float       mHeadSize;   // sphere shape
	static float       mNeckSize;   // sphere shape
	static float       mBodySize;   // sphere shape
	static float       mLegSize;    // cylinder shape
	static float       mFootSize;   // sphere shape
	static float       mStickSize;  // control cross size

	static int         mNeckPart;   // count of neck sphere
	static int         mLegPart;    // count of leg  sphere
	static float       mStringSize; // size of string from head

	// rigid body
	static float       mLinearDamping;  // [0-1]
	static float       mAngularDamping; // [0-1]
	static float       mDeactivationTime;
	static float       mLinearSleepingThresholds;
	static float       mAngularSleepingThresholds;

	// cone twist constraint
	static float       mDamping;
// 	static float       mLinCFM;
// 	static float       mLinERP;
// 	static float       mAngCFM;

	static mndl::kit::params::PInterfaceGl            mParamsRope;
	static int         mRopePart;
	static float       mRopeMass;
	static float       mKVCF;           // Velocities correction factor (Baumgarte)
	static float       mKDP;            // Damping coefficient [0,1]
	static float       mKDG;            // Drag coefficient [0,+inf]
	static float       mKLF;            // Lift coefficient [0,+inf]
	static float       mKPR;            // Pressure coefficient [-inf,+inf]
	static float       mKVC;            // Volume conversation coefficient [0,+inf]
	static float       mKDF;            // Dynamic friction coefficient [0,1]
	static float       mKMT;            // Pose matching coefficient [0,1]		
	static float       mKCHR;           // Rigid contacts hardness [0,1]
	static float       mKKHR;           // Kinetic contacts hardness [0,1]
	static float       mKSHR;           // Soft contacts hardness [0,1]
	static float       mKAHR;           // Anchors hardness [0,1]
	static float       mMaxvolume;      // Maximum volume ratio for pose
	static float       mTimescale;      // Time scale
	static int         mViterations;    // Velocities solver iterations
	static int         mPiterations;    // Positions solver iterations
	static int         mDiterations;    // Drift solver iterations
	static int         mCiterations;    // Cluster solver iterations

// 	static float       mTau;
// 	static float       mDamping;
// 	static float       mImpulseClamp;

	btVector3                mPosHead;
	btVector3                mPosBody;
	btVector3                mPosRightLeg;
	btVector3                mPosLeftLeg;

	ci::Vec3f                mPosHangCenter;
	ci::Vec3f                mPosHangFront;
	ci::Vec3f                mPosHangBack;
	ci::Vec3f                mPosHangLeft;
	ci::Vec3f                mPosHangRight;

	Shapes          mShapes;
	RigidBodies     mRigidBodies;
	SoftBodies      mSoftBodies;
	Constraints     mConstraints;
};

#endif // __BulletBird_H__
