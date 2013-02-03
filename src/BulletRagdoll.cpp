#include "BulletRagdoll.h"

#define CONSTRAINT_DEBUG_SIZE 0.2f
#ifndef M_PI
#define M_PI                  3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2                1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4                0.785398163397448309616
#endif

BulletRagdoll::BulletRagdoll( btDynamicsWorld *ownerWorld, const btVector3 &positionOffset )
: mOwnerWorld( ownerWorld )
{
	// Setup the geometry
	mShapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
	mShapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
	mShapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
	mShapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
	mShapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
	mShapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
	mShapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
	mShapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
	mShapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
	mShapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
	mShapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

	// Setup all the rigid bodies
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
	mBodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_PELVIS]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
	mBodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_SPINE]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
	mBodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_HEAD]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
	mBodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_LEFT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
	mBodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_LEFT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
	mBodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_RIGHT_UPPER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
	mBodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_RIGHT_LOWER_LEG]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	mBodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_LEFT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,M_PI_2);
	mBodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_LEFT_LOWER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	mBodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_RIGHT_UPPER_ARM]);

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI_2);
	mBodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, mShapes[BODYPART_RIGHT_LOWER_ARM]);

	// Setup some damping on the mBodies
	for (int i = 0; i < BODYPART_COUNT; ++i)
	{
		mBodies[i]->setDamping(0.05, 0.85);
		mBodies[i]->setDeactivationTime(0.8);
		mBodies[i]->setSleepingThresholds(1.6, 2.5);
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;
	btConeTwistConstraint* coneC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
	hingeC =  new btHingeConstraint(*mBodies[BODYPART_PELVIS], *mBodies[BODYPART_SPINE], localA, localB);
	hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
	mJoints[JOINT_PELVIS_SPINE] = hingeC;
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_PELVIS_SPINE], true);


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	coneC = new btConeTwistConstraint(*mBodies[BODYPART_SPINE], *mBodies[BODYPART_HEAD], localA, localB);
	coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
	mJoints[JOINT_SPINE_HEAD] = coneC;
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_SPINE_HEAD], true);


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
	coneC = new btConeTwistConstraint(*mBodies[BODYPART_PELVIS], *mBodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
	coneC->setLimit(M_PI_4, M_PI_4, 0);
	mJoints[JOINT_LEFT_HIP] = coneC;
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_LEFT_HIP], true);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
	hingeC =  new btHingeConstraint(*mBodies[BODYPART_LEFT_UPPER_LEG], *mBodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	mJoints[JOINT_LEFT_KNEE] = hingeC;
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_LEFT_KNEE], true);


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
	coneC = new btConeTwistConstraint(*mBodies[BODYPART_PELVIS], *mBodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
	coneC->setLimit(M_PI_4, M_PI_4, 0);
	mJoints[JOINT_RIGHT_HIP] = coneC;
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_RIGHT_HIP], true);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
	hingeC =  new btHingeConstraint(*mBodies[BODYPART_RIGHT_UPPER_LEG], *mBodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	mJoints[JOINT_RIGHT_KNEE] = hingeC;
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_RIGHT_KNEE], true);


	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
	coneC = new btConeTwistConstraint(*mBodies[BODYPART_SPINE], *mBodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
	coneC->setLimit(M_PI_2, M_PI_2, 0);
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mJoints[JOINT_LEFT_SHOULDER] = coneC;
	mOwnerWorld->addConstraint(mJoints[JOINT_LEFT_SHOULDER], true);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	hingeC =  new btHingeConstraint(*mBodies[BODYPART_LEFT_UPPER_ARM], *mBodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
	//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	mJoints[JOINT_LEFT_ELBOW] = hingeC;
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_LEFT_ELBOW], true);



	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
	coneC = new btConeTwistConstraint(*mBodies[BODYPART_SPINE], *mBodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
	coneC->setLimit(M_PI_2, M_PI_2, 0);
	mJoints[JOINT_RIGHT_SHOULDER] = coneC;
	coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_RIGHT_SHOULDER], true);

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
	hingeC =  new btHingeConstraint(*mBodies[BODYPART_RIGHT_UPPER_ARM], *mBodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
	//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	mJoints[JOINT_RIGHT_ELBOW] = hingeC;
	hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

	mOwnerWorld->addConstraint(mJoints[JOINT_RIGHT_ELBOW], true);
}

BulletRagdoll::~BulletRagdoll()
{
	// Remove all constraints
	for( int i = 0; i < JOINT_COUNT; ++i )
	{
		mOwnerWorld->removeConstraint( mJoints[ i ] );
		delete mJoints[ i ]; mJoints[ i ] = 0;
	}

	// Remove all bodies and shapes
	for( int i = 0; i < BODYPART_COUNT; ++i )
	{
		mOwnerWorld->removeRigidBody( mBodies[ i ] );

		delete mBodies[ i ]->getMotionState();

		delete mBodies[ i ]; mBodies[ i ] = 0;
		delete mShapes[ i ]; mShapes[ i ] = 0;
	}
}

btRigidBody *BulletRagdoll::localCreateRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
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