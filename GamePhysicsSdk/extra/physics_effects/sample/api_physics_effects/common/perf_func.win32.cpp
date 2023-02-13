/*
Physics Effects Copyright(C) 2012 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "common.h"
#include "perf_func.h"

void perfInit()
{
}

void perfRelease()
{
}

void perfPushMarker(char *str)
{
}

void perfPopMarker()
{
}

void perfSync()
{
}

unsigned long long perfGetCount()
{
	LARGE_INTEGER cnt;
	QueryPerformanceCounter(&cnt);
	return cnt.QuadPart;
}

float perfGetTimeMillisecond(unsigned long long time1,unsigned long long time2)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return (time2 - time1) / (float)freq.QuadPart * 1000.0f;
}
