//--------------------------------------------------------------------------------------------------
// q3Math.inl
//
//	Copyright (c) 2014 Randy Gaul http://www.randygaul.net
//
//	This software is provided 'as-is', without any express or implied
//	warranty. In no event will the authors be held liable for any damages
//	arising from the use of this software.
//
//	Permission is granted to anyone to use this software for any purpose,
//	including commercial applications, and to alter it and redistribute it
//	freely, subject to the following restrictions:
//	  1. The origin of this software must not be misrepresented; you must not
//	     claim that you wrote the original software. If you use this software
//	     in a product, an acknowledgment in the product documentation would be
//	     appreciated but is not required.
//	  2. Altered source versions must be plainly marked as such, and must not
//	     be misrepresented as being the original software.
//	  3. This notice may not be removed or altered from any source distribution.
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
inline float q3Invert( float a )
{
	return a != 0.0f ? 1.0f / a : 0.0f;
}

//--------------------------------------------------------------------------------------------------
inline float q3Sign( float a )
{
	if ( a >= float( 0.0 ) )
	{
		return float( 1.0 );
	}

	else
	{
		return float( -1.0 );
	}
}


//--------------------------------------------------------------------------------------------------
inline float q3Clamp01( float val )
{
	if ( val >= float( 1.0 ) )
		return 1.0;

	if ( val <= float( 0.0 ) )
		return 0.0;

	return val;
}

//--------------------------------------------------------------------------------------------------
inline float q3Clamp( float min, float max, float a )
{
	if ( a < min )
		return min;

	if ( a > max )
		return max;

	return a;
}

//--------------------------------------------------------------------------------------------------
inline float q3Lerp( float a, float b, float t )
{
	return a * (float( 1.0 ) - t) + b * t;
}

//--------------------------------------------------------------------------------------------------
inline const q3Vec3 q3Lerp( const q3Vec3& a, const q3Vec3& b, float t )
{
	return a * (float( 1.0 ) - t) + b * t;
}

//--------------------------------------------------------------------------------------------------
inline float q3RandomFloat( float l, float h )
{
	float a = float( rand( ) );
	a /= float( RAND_MAX );
	a = (h - l) * a + l;

	return a;
}

//--------------------------------------------------------------------------------------------------
inline int q3RandomInt( int low, int high )
{
	return (rand( ) % (high - low + 1) + low);
}
