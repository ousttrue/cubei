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

#define NUM_BOX_VTX 36
#define NUM_BOX_IDX 36
const float box_vtx[] = {
	-0.500000f,-0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	-0.500000f,0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	0.500000f,0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	-0.500000f,-0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	0.500000f,0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	0.500000f,-0.500000f,-0.500000f,0.000000f,0.000000f,-1.000000f,
	-0.500000f,-0.500000f,-0.500000f,0.000000f,-1.000000f,0.000000f,
	0.500000f,-0.500000f,-0.500000f,0.000000f,-1.000000f,0.000000f,
	0.500000f,-0.500000f,0.500000f,0.000000f,-1.000000f,0.000000f,
	-0.500000f,-0.500000f,-0.500000f,0.000000f,-1.000000f,0.000000f,
	0.500000f,-0.500000f,0.500000f,0.000000f,-1.000000f,0.000000f,
	-0.500000f,-0.500000f,0.500000f,0.000000f,-1.000000f,0.000000f,
	-0.500000f,-0.500000f,-0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,-0.500000f,0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,0.500000f,0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,-0.500000f,-0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,0.500000f,0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,0.500000f,-0.500000f,-1.000000f,0.000000f,0.000000f,
	-0.500000f,-0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	0.500000f,-0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	0.500000f,0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	-0.500000f,-0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	0.500000f,0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	-0.500000f,0.500000f,0.500000f,0.000000f,0.000000f,1.000000f,
	-0.500000f,0.500000f,-0.500000f,0.000000f,1.000000f,0.000000f,
	-0.500000f,0.500000f,0.500000f,0.000000f,1.000000f,0.000000f,
	0.500000f,0.500000f,0.500000f,0.000000f,1.000000f,0.000000f,
	-0.500000f,0.500000f,-0.500000f,0.000000f,1.000000f,0.000000f,
	0.500000f,0.500000f,0.500000f,0.000000f,1.000000f,0.000000f,
	0.500000f,0.500000f,-0.500000f,0.000000f,1.000000f,0.000000f,
	0.500000f,-0.500000f,-0.500000f,1.000000f,0.000000f,0.000000f,
	0.500000f,0.500000f,-0.500000f,1.000000f,0.000000f,0.000000f,
	0.500000f,0.500000f,0.500000f,1.000000f,0.000000f,0.000000f,
	0.500000f,-0.500000f,-0.500000f,1.000000f,0.000000f,0.000000f,
	0.500000f,0.500000f,0.500000f,1.000000f,0.000000f,0.000000f,
	0.500000f,-0.500000f,0.500000f,1.000000f,0.000000f,0.000000f,
};
const unsigned short box_idx[] = {
	0,1,2,
	3,4,5,
	6,7,8,
	9,10,11,
	12,13,14,
	15,16,17,
	18,19,20,
	21,22,23,
	24,25,26,
	27,28,29,
	30,31,32,
	33,34,35,
};
