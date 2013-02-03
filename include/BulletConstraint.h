#ifndef __BulleConstraint_H__
#define __BulleConstraint_H__

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "cinder/Ray.h"
#include "cinder/Vector.h"

struct BulletConstraint
{
	void                    init();

	void                    reset();

	void                    update( const ci::Ray &ray );

	btPoint2PointConstraint *mConstraint;
	float                    mDistance;
	ci::Vec3f                mPosition;
};

#endif // __BulleConstraint_H__
