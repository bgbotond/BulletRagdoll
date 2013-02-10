#include "CinderBullet.h"
#include "HangConstraint.h"

using namespace ci;

HangConstraint::HangConstraint( const ci::Vec3f &position, btRigidBody *rigidBody, const ci::Vec3f &pos )
: mConstraint( 0 )
, mDistance( 0 )
, mPosition( position )
{
	mConstraint = new btPoint2PointConstraint( *rigidBody, CinderBullet::convert( Vec3f::zero()));
	mDistance   = ( position - pos ).length();

	Vec3f pivot;
	if( mDistance > 0.00001 || mDistance < -0.00001 )
		pivot = position + (( pos - position ).normalized() * mDistance );
	else
		pivot = position;
	mConstraint->setPivotB( CinderBullet::convert( pivot ));
}

HangConstraint::~HangConstraint()
{
	delete mConstraint;
}

void HangConstraint::update( const ci::Vec3f &position )
{
	mPosition = position;

	btTransform transform;
	mConstraint->getRigidBodyA().getMotionState()->getWorldTransform( transform );

	Vec3f pos = CinderBullet::convert( transform.getOrigin());

	Vec3f pivot;
	if( mDistance > 0.00001 || mDistance < -0.00001 )
		pivot = position + (( pos - position ).normalized() * mDistance );
	else
		pivot = position;

	mConstraint->setPivotB( CinderBullet::convert( pivot ));
}
