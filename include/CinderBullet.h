#ifndef __CinderBullet_H__
#define __CinderBullet_H__

#include "cinder/Vector.h"
#include "cinder/Quaternion.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

class CinderBullet
{
public:
	static inline btVector3 convert( const ci::Vec3f &vector )
	{
		return btVector3( vector.x, vector.y, vector.z );
	}

	static inline ci::Vec3f convert( const btVector3 &vector )
	{
		return ci::Vec3f( vector.x(), vector.y(), vector.z() );
	}

	static inline btQuaternion convert( const ci::Quatf &quaternion )
	{
		return btQuaternion( quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.w );
	}

	static inline ci::Quatf convert( const btQuaternion &quaternion )
	{
		return ci::Quatf( ci::Vec3f( quaternion.x(), quaternion.y(), quaternion.z() ), quaternion.w() );
	}
};

#endif // __CinderBullet_H__
