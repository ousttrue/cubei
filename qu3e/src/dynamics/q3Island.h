//--------------------------------------------------------------------------------------------------
/**
@file	q3Island.h

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

#ifndef Q3ISLAND_H
#define Q3ISLAND_H

#include "../math/q3Math.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"
#include <vector>
//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
class q3BroadPhase;
class q3Body;
struct q3ContactConstraint;
struct q3ContactConstraintState;

struct q3VelocityState
{
	q3Vec3 w;
	q3Vec3 v;
};

struct q3Island
{
	void Solve( );
	void Add( q3Body *body );
	void Add( q3ContactConstraint *contact );
	void Initialize( );

	std::vector<q3Body*> m_bodies;
	std::vector<q3VelocityState> m_velocities;
	std::vector<q3ContactConstraint*> m_contacts;
	std::vector<q3ContactConstraintState> m_contactStates;

	float m_dt;
	q3Vec3 m_gravity;
	int m_iterations;

	bool m_allowSleep;
	bool m_enableFriction;
};

#endif // Q3ISLAND_H
