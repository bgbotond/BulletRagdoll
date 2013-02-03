#ifndef __BulletRagdoll_H__
#define __BulletRagdoll_H__

#include "btBulletDynamicsCommon.h"

class BulletRagdoll
{
	enum BodyPart
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum Joint
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

public:
	BulletRagdoll( btDynamicsWorld *ownerWorld, const btVector3 &positionOffset );
	~BulletRagdoll();

protected:
	btRigidBody *localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

protected:
	btDynamicsWorld   *mOwnerWorld;
	btCollisionShape  *mShapes[ BODYPART_COUNT ];
	btRigidBody       *mBodies[ BODYPART_COUNT ];
	btTypedConstraint *mJoints[ JOINT_COUNT ];
};

#endif // __BulletRagdoll_H__
