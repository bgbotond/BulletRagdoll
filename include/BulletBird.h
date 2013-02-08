#ifndef __BulletBird_H__
#define __BulletBird_H__

#include <vector>
#include "btBulletDynamicsCommon.h"

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

public:
	BulletBird( btDynamicsWorld *ownerWorld, const ci::Vec3f &worldOffset );
	~BulletBird();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

	btCollisionShape  *getShape( BodyPart bodyPart );
	btRigidBody       *getBody ( BodyPart bodyPart, int count = 0 );

	void setPos( btRigidBody *rigidBody, ci::Vec3f &pos );

protected:
	btDynamicsWorld   *mOwnerWorld;

	float       mBeckSize;  // cylinder shape
	float       mHeadSize;  // sphere shape
	float       mNeckSize;  // sphere shape
	float       mBodySize;  // sphere shape
	float       mLegSize;   // cylinder shape
	float       mFootSize;  // sphere shape

	int         mNeckPart;  // count of neck sphere
	int         mLegPart;   // count of leg  sphere
	int         mStringSize; // size of string from head

	float       mStickSize;

	btVector3                mPosHead;
	btVector3                mPosBody;
	btVector3                mPosRightLeg;
	btVector3                mPosLeftLeg;
	float                    mHangPivot[4];
	btPoint2PointConstraint *mHangConstraint[4];

	Shapes      mShapes;
	Bodies      mBodies;
	Constraints mConstraints;
};

#endif // __BulletBird_H__
