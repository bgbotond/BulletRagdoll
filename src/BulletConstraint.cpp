#include "CinderBullet.h"
#include "BulletConstraint.h"

void BulletConstraint::init()
{
	mConstraint = 0;
	mDistance   = 0.0f;
	mPosition   = ci::Vec3f::one() * mDistance;
}

void BulletConstraint::reset()
{
	if( mConstraint != 0 )
	{
		delete mConstraint;
		mConstraint = 0;
	}
	init();
}

void BulletConstraint::update( const ci::Ray &ray )
{
	mPosition = ray.getOrigin() + ray.getDirection().normalized() * mDistance;
	mConstraint->setPivotB( CinderBullet::convert( mPosition ) );
}

