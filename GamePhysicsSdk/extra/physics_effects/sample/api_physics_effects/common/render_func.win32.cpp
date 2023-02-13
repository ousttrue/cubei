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

#include "render_func.h"

#include <gl/gl.h>

#include "box.h"
#include "sphere.h"
#include "cylinder.h"

// context
static HDC s_hDC;
static HGLRC s_hRC;
static HWND s_hWnd;
static HINSTANCE s_hInstance;
static bool s_enableGlWindow;

// local variables
static char s_title[256];
static int s_screenWidth,s_screenHeight;
static PfxMatrix4 s_pMat,s_vMat;
static PfxVector3 s_viewPos,s_lightPos,s_viewTgt;
static float s_lightRadius,s_lightRadX,s_lightRadY;
static float s_viewRadius,s_viewRadX,s_viewRadY,s_viewHeight;

static unsigned short *s_boxWireIdx;
static unsigned short *s_sphereWireIdx;
static unsigned short *s_cylinderWireIdx;

struct MeshBuff {
	float *vtx;
	float *nml;
	int numVtx;
	unsigned short *idx;
	unsigned short *wireIdx;
	int numIdx;

	MeshBuff(): vtx(0), nml(0), numVtx(0), idx(0), wireIdx(0),numIdx(0){}
};

static sce::PhysicsEffects::PfxArray<MeshBuff>* s_meshBuff;

void releaseWindow()
{
	if(s_enableGlWindow) {
		if(s_hRC) {
			wglMakeCurrent(0,0);
			wglDeleteContext(s_hRC);
		}
	}
	if(s_hDC) ReleaseDC(s_hWnd,s_hDC);
	if(s_hWnd) DestroyWindow(s_hWnd);

	UnregisterClass(s_title,s_hInstance);
}

LRESULT CALLBACK WndProc(HWND s_hWnd,UINT	uMsg,WPARAM	wParam,LPARAM lParam)
{
	switch(uMsg) {
		case WM_SYSCOMMAND:
		{
			switch (wParam) {
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
				return 0;
			}
			break;
		}

		case WM_CLOSE:
		PostQuitMessage(0);
		return 0;

		case WM_SIZE:
		renderResize(LOWORD(lParam),HIWORD(lParam));
		return 0;
	}

	return DefWindowProc(s_hWnd,uMsg,wParam,lParam);
}

bool createWindow(const char* title, int width, int height)
{
	strncpy_s(s_title,title,sizeof(s_title));
	s_title[255] = 0;

	WNDCLASS wc;
	RECT rect;
	rect.left=0;
	rect.right=width;
	rect.top=0;
	rect.bottom=height;

	s_hInstance = GetModuleHandle(NULL);
	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wc.lpfnWndProc = (WNDPROC) WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = s_hInstance;
	wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wc.hbrBackground = NULL;
	wc.lpszMenuName = NULL;
	wc.lpszClassName = s_title;

	if(!RegisterClass(&wc)) {
		return false;
	}

	AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, WS_EX_APPWINDOW | WS_EX_WINDOWEDGE);

	if(!(s_hWnd=CreateWindowEx(WS_EX_APPWINDOW|WS_EX_WINDOWEDGE,s_title,s_title,
							WS_OVERLAPPEDWINDOW|WS_CLIPSIBLINGS|WS_CLIPCHILDREN,
							0,0,rect.right-rect.left,rect.bottom-rect.top,
							NULL,NULL,s_hInstance,NULL))) {
		releaseWindow();
		return false;
	}

    static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32,
		0, 0,
		0, 0,
		0, 0,
		0, 0,
		0,
		0, 0, 0, 0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
    };
	
	if(!(s_hDC=GetDC(s_hWnd)))
	{
		releaseWindow();
		OutputDebugString("");
		return FALSE;
	}
	
	int pixelformat;
	
    if ( (pixelformat = ChoosePixelFormat(s_hDC, &pfd)) == 0 ){
		OutputDebugString("ChoosePixelFormat Failed....");
        return FALSE;
    }

    if (SetPixelFormat(s_hDC, pixelformat, &pfd) == FALSE){
		OutputDebugString("SetPixelFormat Failed....");
        return FALSE;
    }

	if (!(s_hRC=wglCreateContext(s_hDC))){
		OutputDebugString("Creating HGLRC Failed....");
		return FALSE;
	}
	
	wglMakeCurrent(s_hDC,s_hRC);
	
	// Set Vsync
	BOOL (WINAPI *wglSwapIntervalEXT)(int) = NULL;
	wglSwapIntervalEXT = (BOOL (WINAPI*)(int))wglGetProcAddress("wglSwapIntervalEXT");
	if(wglSwapIntervalEXT) wglSwapIntervalEXT(1);
	
	ShowWindow(s_hWnd,SW_SHOW);
	SetForegroundWindow(s_hWnd);
	SetFocus(s_hWnd);

	renderResize(width, height);
	
	glClearColor(0.0f,0.0f,0.0f,0.0f);
	glClearDepth(1.0f);
	
	return TRUE;
}

void renderInit(const char *title)
{
	s_enableGlWindow = (title != NULL);

	s_screenWidth = DISPLAY_WIDTH;
	s_screenHeight = DISPLAY_HEIGHT;

	if(s_enableGlWindow) {
		if(!createWindow(title,s_screenWidth,s_screenHeight)) {
			MessageBox(NULL,"Can't create gl window.","ERROR",MB_OK|MB_ICONEXCLAMATION);
			SCE_PFX_ALWAYS_ASSERT(0);
		}
	}

	// initalize matrix
	s_pMat = PfxMatrix4::perspective(3.1415f/4.0f, (float)s_screenWidth/(float)s_screenHeight,0.1f, 1000.0f);

	// initalize parameters
	s_lightRadius = 40.0f;
	s_lightRadX = -0.6f;
	s_lightRadY = 0.6f;
	s_viewRadius = 40.0f;
	s_viewRadX = -0.01f;
	s_viewRadY = 0.0f;
	s_viewHeight = 1.0f;

	s_viewTgt = PfxVector3(0.0f,s_viewHeight,0.0f);

	s_boxWireIdx = new unsigned short [NUM_BOX_IDX*2];
	s_sphereWireIdx = new unsigned short [NUM_SPHERE_IDX*2];
	s_cylinderWireIdx = new unsigned short [NUM_CYLINDER_IDX*2];

	for(int i=0;i<NUM_BOX_IDX/3;i++) {
		s_boxWireIdx[i*6  ] = box_idx[i*3  ];
		s_boxWireIdx[i*6+1] = box_idx[i*3+1];
		s_boxWireIdx[i*6+2] = box_idx[i*3+1];
		s_boxWireIdx[i*6+3] = box_idx[i*3+2];
		s_boxWireIdx[i*6+4] = box_idx[i*3+2];
		s_boxWireIdx[i*6+5] = box_idx[i*3  ];
	}

	for(int i=0;i<NUM_SPHERE_IDX/3;i++) {
		s_sphereWireIdx[i*6  ] = sphere_idx[i*3  ];
		s_sphereWireIdx[i*6+1] = sphere_idx[i*3+1];
		s_sphereWireIdx[i*6+2] = sphere_idx[i*3+1];
		s_sphereWireIdx[i*6+3] = sphere_idx[i*3+2];
		s_sphereWireIdx[i*6+4] = sphere_idx[i*3+2];
		s_sphereWireIdx[i*6+5] = sphere_idx[i*3  ];
	}

	for(int i=0;i<NUM_CYLINDER_IDX/3;i++) {
		s_cylinderWireIdx[i*6  ] = cylinder_idx[i*3  ];
		s_cylinderWireIdx[i*6+1] = cylinder_idx[i*3+1];
		s_cylinderWireIdx[i*6+2] = cylinder_idx[i*3+1];
		s_cylinderWireIdx[i*6+3] = cylinder_idx[i*3+2];
		s_cylinderWireIdx[i*6+4] = cylinder_idx[i*3+2];
		s_cylinderWireIdx[i*6+5] = cylinder_idx[i*3  ];
	}
	
	s_meshBuff = new PfxArray<MeshBuff>();
	s_meshBuff->clear();
}

void renderRelease()
{
	delete [] s_boxWireIdx;
	delete [] s_sphereWireIdx;
	delete [] s_cylinderWireIdx;
	
	for (PfxUInt32 c=0; c<s_meshBuff->size(); ++c)
	{
		delete[] (*s_meshBuff)[c].vtx;
		delete[] (*s_meshBuff)[c].nml;
		delete[] (*s_meshBuff)[c].idx;
		delete[] (*s_meshBuff)[c].wireIdx;
	}
	s_meshBuff->clear();
	delete s_meshBuff;

	releaseWindow();
}

void renderBegin()
{
	wglMakeCurrent(s_hDC, s_hRC);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glFrontFace(GL_CCW);
    glDepthFunc(GL_LESS);
	glCullFace(GL_BACK);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&s_pMat);

	// create view matrix
	s_viewPos = 
		PfxMatrix3::rotationY(s_viewRadY) * 
		PfxMatrix3::rotationX(s_viewRadX) * 
		PfxVector3(0,0,s_viewRadius);

	s_lightPos = 
		PfxMatrix3::rotationY(s_lightRadY) * 
		PfxMatrix3::rotationX(s_lightRadX) * 
		PfxVector3(0,0,s_lightRadius);

	PfxMatrix4 viewMtx = PfxMatrix4::lookAt(PfxPoint3(s_viewTgt+s_viewPos),PfxPoint3(s_viewTgt),PfxVector3(0.0f,1.0f,0.0f));

	s_vMat = s_pMat * viewMtx;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&viewMtx);
}

void renderLookAtTarget(const PfxVector3 &viewPos,const PfxVector3 &viewTarget)
{
	s_viewPos = viewPos;
	PfxMatrix4 viewMtx = PfxMatrix4::lookAt(PfxPoint3(viewPos),PfxPoint3(viewTarget),PfxVector3(0.0f,1.0f,0.0f));
	s_vMat = s_pMat * viewMtx;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((GLfloat*)&viewMtx);
}

void renderEnd()
{
	SwapBuffers(s_hDC);
}

void renderDebugBegin()
{
	glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);
}

void renderDebugEnd()
{
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

void renderGetViewAngle(float &angleX,float &angleY,float &radius)
{
	angleX = s_viewRadX;
	angleY = s_viewRadY;
	radius = s_viewRadius;
}

void renderSetViewAngle(float angleX,float angleY,float radius)
{
	s_viewRadX   = angleX;
	s_viewRadY   = angleY;
	s_viewRadius = radius;
}

void renderSphere(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius)
{
	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(PfxVector3(radius));

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,24,sphere_vtx);

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_SPHERE_IDX,GL_UNSIGNED_SHORT,sphere_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0f);
	glDrawElements(GL_LINES,NUM_SPHERE_IDX*2,GL_UNSIGNED_SHORT,s_sphereWireIdx);
	
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void renderBox(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxVector3 &halfExtent)
{
	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(2.0f*halfExtent);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);

	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,24,box_vtx);

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_BOX_IDX,GL_UNSIGNED_SHORT,box_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0f);
	glDrawElements(GL_LINES,NUM_BOX_IDX*2,GL_UNSIGNED_SHORT,s_boxWireIdx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void renderCylinder(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength)
{
	PfxVector3 scale(halfLength,radius,radius);

	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(scale);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);
	
	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3,GL_FLOAT,24,cylinder_vtx);

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,NUM_CYLINDER_IDX,GL_UNSIGNED_SHORT,cylinder_idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0f);
	glDrawElements(GL_LINES,NUM_CYLINDER_IDX*2,GL_UNSIGNED_SHORT,s_cylinderWireIdx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void renderCapsule(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	const PfxFloatInVec &radius,
	const PfxFloatInVec &halfLength)
{
	PfxTransform3 tr1 = PfxTransform3::translation(PfxVector3(-halfLength,0.0f,0.0f));
	PfxTransform3 tr2 = PfxTransform3::translation(PfxVector3(halfLength,0.0f,0.0f));

	renderSphere(transform*tr1,color,radius);
	renderSphere(transform*tr2,color,radius);

	renderCylinder(transform,color,radius,halfLength);
}

int renderInitMesh(
	const float *vtx,unsigned int vtxStrideBytes,
	const float *nml,unsigned int nmlStrideBytes,
	const unsigned short *tri,unsigned int triStrideBytes,
	int numVtx,int numTri)
{
//	assert(numMesh<MAX_MESH);
	
	MeshBuff buff;
	buff.vtx = new float [3*numVtx];
	buff.nml = new float [3*numVtx];
	buff.idx = new unsigned short [numTri*3];
	buff.wireIdx = new unsigned short [numTri*6];
	buff.numIdx = numTri*3;
	buff.numVtx = numVtx;

	for(int i=0;i<numVtx;i++) {
		const float *v = (float*)((uintptr_t)vtx + vtxStrideBytes * i);
		const float *n = (float*)((uintptr_t)nml + nmlStrideBytes * i);
		buff.vtx[i*3  ] = v[0];
		buff.vtx[i*3+1] = v[1];
		buff.vtx[i*3+2] = v[2];
		buff.nml[i*3  ] = n[0];
		buff.nml[i*3+1] = n[1];
		buff.nml[i*3+2] = n[2];
	}

	for(int i=0;i<numTri;i++) {
		const unsigned short *idx = (unsigned short*)((uintptr_t)tri + triStrideBytes * i);
		buff.idx[i*3  ] = idx[0];
		buff.idx[i*3+1] = idx[1];
		buff.idx[i*3+2] = idx[2];
		buff.wireIdx[i*6  ] = buff.idx[i*3  ];
		buff.wireIdx[i*6+1] = buff.idx[i*3+1];
		buff.wireIdx[i*6+2] = buff.idx[i*3+1];
		buff.wireIdx[i*6+3] = buff.idx[i*3+2];
		buff.wireIdx[i*6+4] = buff.idx[i*3+2];
		buff.wireIdx[i*6+5] = buff.idx[i*3  ];
	}

	s_meshBuff->push(buff);
	return s_meshBuff->size()-1;
}
void renderReleaseMeshAll()
{
	for (PfxUInt32 c=0; c<s_meshBuff->size(); ++c)
	{
		delete[] (*s_meshBuff)[c].vtx;
		delete[] (*s_meshBuff)[c].nml;
		delete[] (*s_meshBuff)[c].idx;
		delete[] (*s_meshBuff)[c].wireIdx;
	}
	s_meshBuff->clear();
}

void renderMesh(
	const PfxTransform3 &transform,
	const PfxVector3 &color,
	int meshId)
{
	assert(meshId>=0&&(PfxUInt32)meshId<s_meshBuff->size());
	
	MeshBuff &buff = (*s_meshBuff)[meshId];

	PfxMatrix4 wMtx = PfxMatrix4(transform);

	glPushMatrix();
	glMultMatrixf((GLfloat*)&wMtx);

	glEnableClientState(GL_VERTEX_ARRAY);
	
	glVertexPointer(3,GL_FLOAT,0,buff.vtx);

	glColor4f(color[0],color[1],color[2],1.0f);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1.0f,1.0f);
	glDrawElements(GL_TRIANGLES,buff.numIdx,GL_UNSIGNED_SHORT,buff.idx);
	glDisable(GL_POLYGON_OFFSET_FILL);

	glColor4f(0.0f,0.0f,0.0f,1.0f);
	glDrawElements(GL_LINES,buff.numIdx*2,GL_UNSIGNED_SHORT,buff.wireIdx);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
}

void renderResize(int width,int height)
{
	glViewport(0,0,width,height);
	s_pMat = PfxMatrix4::perspective(3.1415f/4.0f, (float)width/(float)height,0.1f, 1000.0f);
	s_screenWidth = width;
	s_screenHeight = height;
}

void renderDebugPoint(
	const PfxVector3 &position,
	const PfxVector3 &color)
{
	glColor4f(color[0],color[1],color[2],1.0f);

	glPointSize(5.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)&position);
	glDrawArrays(GL_POINTS,0,1);
	glDisableClientState(GL_VERTEX_ARRAY);
	glPointSize(1.0f);
}

void renderDebugLine(
	const PfxVector3 &position1,
	const PfxVector3 &position2,
	const PfxVector3 &color)
{
	glColor4f(color[0],color[1],color[2],1.0f);
	
	const PfxVector3 points[2] = {
		position1,
		position2,
	};
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawArrays(GL_LINES,0,2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebugArc(
	const PfxVector3 &center,
	const PfxVector3 &normal,
	const float radius,
	const float start_rad,
	const float end_rad,
	const PfxVector3 &color)
{
	glColor4fv((float*)&color);
	
	GLfloat vtx[42];
	const int pNum = 12;
	
	PfxVector3 upVec(0,1,0);

	if(dot(upVec,normal) > 0.99f) {
		upVec = PfxVector3(1,0,0);
	}

	PfxVector3 t = radius * normalize(cross(upVec,normalize(normal)));
	PfxVector3 cpS = rotate(PfxQuat::rotation(start_rad,normalize(normal)),t);
	PfxQuat rot = PfxQuat::rotation((end_rad-start_rad)/(float)pNum,normalize(normal));

	int vc = 0;
	vtx[vc++] = center[0];
	vtx[vc++] = center[1];
	vtx[vc++] = center[2];

	PfxVector3 cp = cpS;
	for(int i=0;i<=pNum;i++) {
		PfxVector3 pos = center + cp;
		vtx[vc++] = pos[0];
		vtx[vc++] = pos[1];
		vtx[vc++] = pos[2];
		cp = rotate(rot,cp);
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,&vtx[0]);
	glColor4f(color[0],color[1],color[2],(GLfloat)1.0);
	glDrawArrays(GL_LINE_LOOP,0,pNum+2);
	glDisableClientState(GL_VERTEX_ARRAY);	
}

void renderDebugArc(
	const PfxVector3 &pos,
	const PfxVector3 &axis,
	const PfxVector3 &dir,
	const float radius,
	const float start_rad,
	const float end_rad,
	const PfxVector3 &color)
{
	GLfloat vtx[42];
	const int pNum = 12;

	PfxVector3 cp = rotate(PfxQuat::rotation(start_rad,normalize(axis)),dir);
	PfxQuat rot = PfxQuat::rotation((end_rad-start_rad)/(float)pNum,normalize(axis));

	int vc = 0;
	vtx[vc++] = pos[0];
	vtx[vc++] = pos[1];
	vtx[vc++] = pos[2];
	for(int i=0;i<=pNum;i++) {
		PfxVector3 p = pos + radius * cp;
		vtx[vc++] = p[0];
		vtx[vc++] = p[1];
		vtx[vc++] = p[2];
		cp = rotate(rot,cp);
	}
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,0,&vtx[0]);
	glColor4f(color[0],color[1],color[2],1.0);
	glDrawArrays(GL_LINE_LOOP,0,pNum+2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebugBox(
	const PfxTransform3 &transform,
	const PfxVector3 &extent,
	const PfxVector3 &color)
{
	PfxMatrix4 wMtx = PfxMatrix4(transform) * PfxMatrix4::scale(2.0f*extent);
	const PfxPoint3 points[8] = {
		 transform * mulPerElem(PfxPoint3(-1,-1,-1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3(-1,-1, 1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3( 1,-1, 1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3( 1,-1,-1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3(-1, 1,-1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3(-1, 1, 1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3( 1, 1, 1),PfxPoint3(extent)),
		 transform * mulPerElem(PfxPoint3( 1, 1,-1),PfxPoint3(extent)),
	};
	
	const unsigned short indices[] = {
		0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7,
	};
	
	glColor4f(color[0],color[1],color[2],1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawElements(GL_LINES,24,GL_UNSIGNED_SHORT,indices);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebugAabb(
	const PfxVector3 &center,
	const PfxVector3 &extent,
	const PfxVector3 &color)
{
	const PfxVector3 points[8] = {
		center + mulPerElem(PfxVector3(-1,-1,-1),extent),
		center + mulPerElem(PfxVector3(-1,-1, 1),extent),
		center + mulPerElem(PfxVector3( 1,-1, 1),extent),
		center + mulPerElem(PfxVector3( 1,-1,-1),extent),
		center + mulPerElem(PfxVector3(-1, 1,-1),extent),
		center + mulPerElem(PfxVector3(-1, 1, 1),extent),
		center + mulPerElem(PfxVector3( 1, 1, 1),extent),
		center + mulPerElem(PfxVector3( 1, 1,-1),extent),
	};
	
	const unsigned short indices[] = {
		0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7,
	};
	
	glColor4f(color[0],color[1],color[2],1.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3,GL_FLOAT,16,(float*)points);
	glDrawElements(GL_LINES,24,GL_UNSIGNED_SHORT,indices);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void renderDebug2dBegin()
{
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	PfxMatrix4 proj = PfxMatrix4::orthographic(-s_screenWidth*0.5f,s_screenWidth*0.5f,-s_screenHeight*0.5f,s_screenHeight*0.5f, -10.0f, 10.0f);
	glMultMatrixf((GLfloat*)&proj);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	PfxMatrix4 modelview = PfxMatrix4::translation(PfxVector3(0,0,-1));
	glMultMatrixf((GLfloat*)&modelview);

	glDisable(GL_DEPTH_TEST);
}

void renderDebug2dEnd()
{
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
}

PfxVector3 renderGetWorldPosition(const PfxVector3 &screenPos)
{
	PfxMatrix4 mvp,mvpInv;
	mvp = s_vMat;
	mvpInv = inverse(mvp);

	PfxVector4 wp(screenPos,1.0f);

	wp[0] /= (0.5f * (float)s_screenWidth);
	wp[1] /= (0.5f * (float)s_screenHeight);

	float w =	mvpInv[0][3] * wp[0] +  
				mvpInv[1][3] * wp[1] +  
				mvpInv[2][3] * wp[2] +  
				mvpInv[3][3];

	wp = mvpInv * wp;
	wp /= w;

	return wp.getXYZ();
}

PfxVector3 renderGetScreenPosition(const PfxVector3 &worldPos)
{
	PfxVector4 sp(worldPos,1.0f);

	PfxMatrix4 mvp;
	mvp = s_vMat;

	sp = mvp * sp;
	sp /= (float)sp[3];
	sp[0] *= (0.5f * (float)s_screenWidth);
	sp[1] *= (0.5f * (float)s_screenHeight);

	return sp.getXYZ();
}

void renderGetScreenSize(int &width,int &height)
{
	width = s_screenWidth;
	height = s_screenHeight;
}

void renderGetViewTarget(PfxVector3 &targetPos)
{
	targetPos = s_viewTgt;
}

void renderSetViewTarget(const PfxVector3 &targetPos)
{
	s_viewTgt = targetPos;
}

void renderGetViewRadius(float &radius)
{
	radius = s_viewRadius;
}

void renderSetViewRadius(float radius)
{
	s_viewRadius = radius;
}

void renderSetContext(HDC hDC,HGLRC hRC)
{
	s_hDC = hDC;
	s_hRC = hRC;
}

void renderEnableDepthTest()
{
	glEnable(GL_DEPTH_TEST);
}

void renderDisableDepthTest()
{
	glDisable(GL_DEPTH_TEST);
}

void renderWait()
{
	glFinish();
}
