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
		BODYPART_HANG,
	};

typedef std::vector< btCollisionShape  * > Shapes;
typedef std::vector< btRigidBody       * > Bodies;
typedef std::vector< btTypedConstraint * > Constraints;

public:
	BulletBird( btDynamicsWorld *ownerWorld, const btVector3 &positionOffset );
	~BulletBird();

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

	btCollisionShape  *getShape( BodyPart bodyPart );
	btRigidBody       *getBody ( BodyPart bodyPart, int count = 0 );

protected:
	btDynamicsWorld   *mOwnerWorld;

	float       mBeckSize;  // cylinder shape
	float       mHeadSize;  // sphere shape
	float       mNeckSize;  // sphere shape
	float       mBodySize;  // sphere shape
	float       mLegSize;   // cylinder shape
	float       mFootSize;  // sphere shape
	float       mHangSize;  // box shape

	int         mNeckPart;  // count of neck sphere
	int         mLegPart;   // count of leg  sphere
	int         mHangPart;  // count of hang sphere
	int         mStringSize; // size of string from head

	Shapes      mShapes;
	Bodies      mBodies;
	Constraints mConstraints;
};

#endif // __BulletBird_H__
