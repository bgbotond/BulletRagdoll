#ifndef __AssimpBird_H__
#define __AssimpBird_H__

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "mndlkit/params/PParams.h"
#include "AssimpLoader.h"
#include "Node.h"

class AssimpBird
{

typedef std::vector< btCollisionShape  * > Shapes;
typedef std::map< std::string, btRigidBody * > RigidBodies;
typedef std::vector< btSoftBody        * > SoftBodies;
typedef std::vector< btTypedConstraint * > Constraints;

typedef std::map< std::string, std::string > Bones;

public:
	AssimpBird( btDynamicsWorld *ownerWorld, btSoftBodyWorldInfo *softBodyWorldInfo, const ci::Vec3f &worldOffset, const boost::filesystem::path &file );
	~AssimpBird();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );
	void draw();

	static void setupParams();

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );
	btSoftBody  *localCreateRope( const ci::Vec3f &from, const ci::Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo );

	void  defineBones();
	void  defineJoints();
	void  buildBones ( const mndl::NodeRef &node );
	void  buildBone  ( const mndl::NodeRef &node );
	void  buildJoints( const mndl::NodeRef &node );
	void  buildJoint ( const mndl::NodeRef &node );
	void  updateBones( const mndl::NodeRef &node );
	void  updateBone ( const mndl::NodeRef &node );
	float getLength  ( const mndl::NodeRef &node );
	bool  filterNode ( const mndl::NodeRef &node );

	void  buildHang();
	void  updateHang( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

protected:
	btDynamicsWorld         *mOwnerWorld;
	btSoftBodyWorldInfo     *mSoftBodyWorldInfo;
	boost::filesystem::path  mFile;

	mndl::assimp::AssimpLoader mAssimpLoader;

	static mndl::kit::params::PInterfaceGl            mParamsBird;

	// rigid body
	static float       mLinearDamping;  // [0-1]
	static float       mAngularDamping; // [0-1]
	static float       mDeactivationTime;
	static float       mLinearSleepingThresholds;
	static float       mAngularSleepingThresholds;

	// cone twist constraint
	static float       mDamping;
	static float       mStopCMF;
	static float       mStopERP;
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

	static float       MIN_BONE_LENGTH;
	static float       CAPSULE_RADIUS;

	static bool        mDrawSkin;
	static bool        mEnableWireframe;

// 	static float       mTau;
// 	static float       mDamping;
// 	static float       mImpulseClamp;

	Shapes          mShapes;
	RigidBodies     mRigidBodies;
	SoftBodies      mSoftBodies;
	Constraints     mConstraints;

	ci::Vec3f       mHangCenter;
	float           mHangSizeFront;
	float           mHangSizeBack;
	float           mHangSizeLeft;
	float           mHangSizeRight;
	float           mHangSizeMiddle;

	Bones           mBones;
};

#endif // __AssimpBird_H__
