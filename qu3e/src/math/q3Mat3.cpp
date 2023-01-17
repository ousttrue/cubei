//--------------------------------------------------------------------------------------------------
/**
@file	q3Mat3.cpp

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

#include <cassert>
#include "q3Mat3.h"

//--------------------------------------------------------------------------------------------------
// q3Mat3
//--------------------------------------------------------------------------------------------------
q3Mat3::q3Mat3( )
{
}

//--------------------------------------------------------------------------------------------------
q3Mat3::q3Mat3( float a, float b, float c, float d, float e, float f, float g, float h, float i )
	: ex( a, b, c )
	, ey( d, e, f )
	, ez( g, h, i )
{
}

//--------------------------------------------------------------------------------------------------
q3Mat3::q3Mat3( const q3Vec3& _x, const q3Vec3& _y, const q3Vec3& _z )
	: ex( _x )
	, ey( _y )
	, ez( _z )
{
}

//--------------------------------------------------------------------------------------------------
void q3Mat3::Set( float a, float b, float c, float d, float e, float f, float g, float h, float i )
{
	ex.Set( a, b, c );
	ey.Set( d, e, f );
	ez.Set( g, h, i );
}

//--------------------------------------------------------------------------------------------------
void q3Mat3::Set( const q3Vec3& axis, float angle )
{
	float s = std::sin( angle );
	float c = std::cos( angle );
	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	float xy = x * y;
	float yz = y * z;
	float zx = z * x;
	float t = float( 1.0 ) - c;

	Set(
		x * x * t + c, xy * t + z * s, zx * t - y * s,
		xy * t - z * s, y * y * t + c, yz * t + x * s,
		zx * t + y * s, yz * t - x * s, z * z * t + c
		);
}

//--------------------------------------------------------------------------------------------------
void q3Mat3::SetRows( const q3Vec3& x, const q3Vec3& y, const q3Vec3& z )
{
	ex = x;
	ey = y;
	ez = z;
}

//--------------------------------------------------------------------------------------------------
q3Mat3& q3Mat3::operator=( const q3Mat3& rhs )
{
	ex = rhs.ex;
	ey = rhs.ey;
	ez = rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
q3Mat3& q3Mat3::operator*=( const q3Mat3& rhs )
{
	*this = *this * rhs;

	return *this;
}

//--------------------------------------------------------------------------------------------------
q3Mat3& q3Mat3::operator*=( float f )
{
	ex *= f;
	ey *= f;
	ez *= f;

	return *this;
}


//--------------------------------------------------------------------------------------------------
q3Mat3& q3Mat3::operator+=( const q3Mat3& rhs )
{
	ex += rhs.ex;
	ey += rhs.ey;
	ez += rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
q3Mat3& q3Mat3::operator-=( const q3Mat3& rhs )
{
	ex -= rhs.ex;
	ey -= rhs.ey;
	ez -= rhs.ez;

	return *this;
}

//--------------------------------------------------------------------------------------------------
q3Vec3& q3Mat3::operator[]( uint32_t index )
{
	switch ( index )
	{
	case 0:
		return ex;
	case 1:
		return ey;
	case 2:
		return ez;
	default:
		assert( false );
		return ex;
	}
}

//--------------------------------------------------------------------------------------------------
const q3Vec3& q3Mat3::operator[]( uint32_t index ) const
{
	switch ( index )
	{
	case 0:
		return ex;
	case 1:
		return ey;
	case 2:
		return ez;
	default:
		assert( false );
		return ex;
	}
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column0( ) const
{
	return q3Vec3( ex.x, ey.x, ez.x );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column1( ) const
{
	return q3Vec3( ex.y, ey.y, ez.y );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column2( ) const
{
	return q3Vec3( ex.z, ey.z, ez.z );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Mat3::operator*( const q3Vec3& rhs ) const
{
	return q3Vec3(
		ex.x * rhs.x + ey.x * rhs.y + ez.x * rhs.z,
		ex.y * rhs.x + ey.y * rhs.y + ez.y * rhs.z,
		ex.z * rhs.x + ey.z * rhs.y + ez.z * rhs.z
		);
}

//--------------------------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator*( const q3Mat3& rhs ) const
{
	return q3Mat3(
		( *this * rhs.ex ),
		( *this * rhs.ey ),
		( *this * rhs.ez )
		);
}

//--------------------------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator*( float f ) const
{
	return q3Mat3( ex * f, ey * f, ez * f );
}

//--------------------------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator+( const q3Mat3& rhs ) const
{
	return q3Mat3( ex + rhs.ex, ey + rhs.ey, ez + rhs.ez );
}

//--------------------------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator-( const q3Mat3& rhs ) const
{
	return q3Mat3( ex - rhs.ex, ey - rhs.ey, ez - rhs.ez );
}
