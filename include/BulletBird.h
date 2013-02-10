#ifndef __BulletBird_H__
#define __BulletBird_H__

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "mndlkit/params/PParams.h"
#include "HangConstraint.h"

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
	};

typedef std::vector< btCollisionShape  * > Shapes;
typedef std::vector< btRigidBody       * > Bodies;
typedef std::vector< btTypedConstraint * > Constraints;
typedef std::vector< HangConstraint    * > HangConstraints;

public:
	BulletBird( btDynamicsWorld *ownerWorld, const ci::Vec3f &worldOffset );
	~BulletBird();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

	static void setupParams();

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

	btCollisionShape  *getShape( BodyPart bodyPart );
	btRigidBody       *getBody ( BodyPart bodyPart, int count = 0 );

protected:
	btDynamicsWorld   *mOwnerWorld;

	static mndl::kit::params::PInterfaceGl            mParams;
	static float       mBeckSize;   // cylinder shape
	static float       mHeadSize;   // sphere shape
	static float       mNeckSize;   // sphere shape
	static float       mBodySize;   // sphere shape
	static float       mLegSize;    // cylinder shape
	static float       mFootSize;   // sphere shape
	static float       mStickSize;  // control cross size

	static int         mNeckPart;   // count of neck sphere
	static int         mLegPart;    // count of leg  sphere
	static int         mStringSize; // size of string from head

	static float       mTau;
	static float       mDamping;
	static float       mImpulseClamp;

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
	Bodies          mBodies;
	Constraints     mConstraints;
	HangConstraints mHangConstraints;
};

#endif // __BulletBird_H__
