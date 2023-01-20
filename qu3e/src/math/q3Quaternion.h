//--------------------------------------------------------------------------------------------------
/**
@file	q3Quaternion.h

@author	Randy Gaul
@date	10/10/2014

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3QUATERNION_H
#define Q3QUATERNION_H


#include "q3Vec3.h"

//--------------------------------------------------------------------------------------------------
// q3Quaternion
//--------------------------------------------------------------------------------------------------
struct q3Mat3;

class q3Quaternion
{
public:
	union
	{
		float v[ 4 ];

		struct
		{
			float x;
			float y;
			float z;

			float w;
		};
	};

	q3Quaternion( );
	q3Quaternion( float a, float b, float c, float d );
	q3Quaternion( const q3Vec3& axis, float radians );

	void Set( const q3Vec3& axis, float radians );
	void ToAxisAngle( q3Vec3* axis, float* angle ) const;
	void Integrate( const q3Vec3& dv, float dt );

	const q3Quaternion operator*( const q3Quaternion& rhs ) const;
	q3Quaternion& operator*=( const q3Quaternion& rhs );

	const q3Mat3 ToMat3( void ) const;
};

//--------------------------------------------------------------------------------------------------
inline const q3Quaternion q3Normalize( const q3Quaternion& q )
{
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;

	float d = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

	if( d == 0 )
		w = float( 1.0 );

	d = float( 1.0 ) / std::sqrt( d );

	if ( d > float( 1.0e-8 ) )
	{
		x *= d;
		y *= d;
		z *= d;
		w *= d;
	}

	return q3Quaternion( x, y, z, w );
}

#endif // Q3QUATERNION_H
