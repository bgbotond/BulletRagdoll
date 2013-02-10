#ifndef __HangConstraint_H__
#define __HangConstraint_H__

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "cinder/Vector.h"

class HangConstraint
{
public:
	HangConstraint( const ci::Vec3f &position, btRigidBody *rigidBody, const ci::Vec3f &pos );
	~HangConstraint();

	void                    update( const ci::Vec3f &position );

	btPoint2PointConstraint *mConstraint;
	float                    mDistance;
	ci::Vec3f                mPosition;
};

#endif // __HangConstraint_H__
