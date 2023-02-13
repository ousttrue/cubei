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

#include "create_render_mesh.h"
#include "render_func.h"

void _constructMesh(const PfxVector3* verts, const PfxUInt32 num_verts, const PfxUInt16* idxs, const PfxUInt32 num_idxs, const PfxUInt32 offset_idx, PfxFloat* overts, PfxFloat* onmls, PfxUInt16* oidx)
{
	for (PfxUInt32 c=0; c<num_verts; ++c)
	{
		pfxStoreVector3(verts[c],&overts[c*3]);
	}
	for (PfxUInt32 c=0; c<num_verts; ++c)
	{
		PfxVector3 normal(0.0f);
		for (PfxUInt32 d=0; d<num_idxs/3; ++d)
		{
			if (idxs[d*3+0]==c || idxs[d*3+1]==c || idxs[d*3+2]==c)
			{
				const PfxVector3& v0 = verts[idxs[d*3+0]];
				const PfxVector3& v1 = verts[idxs[d*3+1]];
				const PfxVector3& v2 = verts[idxs[d*3+2]];
				PfxVector3 v01 = v1 - v0;
				PfxVector3 v02 = v2 - v0;
				normal += cross(v01, v02);
			}
		}
		normal = normalize(normal);

		pfxStoreVector3(normal,&onmls[c*3]);
	}
	for (PfxUInt32 c=0; c<num_idxs/3; ++c)
	{
		oidx[c*3+0] = idxs[c*3+0] + offset_idx;
		oidx[c*3+1] = idxs[c*3+1] + offset_idx;
		oidx[c*3+2] = idxs[c*3+2] + offset_idx;
	}
}

void _constructMesh(const float* verts, const PfxUInt32 num_verts, const PfxUInt16* idxs, const PfxUInt32 num_idxs, const PfxUInt32 offset_idx, PfxFloat* overts, PfxFloat* onmls, PfxUInt16* oidx)
{
	for (PfxUInt32 c=0; c<num_verts * 3; ++c)
	{
		overts[c] = verts[c];
	}
	for (PfxUInt32 c=0; c<num_verts; ++c)
	{
		PfxVector3 normal(0.0f);
		for (PfxUInt32 d=0; d<num_idxs/3; ++d)
		{
			if (idxs[d*3+0]==c || idxs[d*3+1]==c || idxs[d*3+2]==c)
			{
				const PfxVector3& v0 = pfxReadVector3(&verts[idxs[d*3+0]*3]);
				const PfxVector3& v1 = pfxReadVector3(&verts[idxs[d*3+1]*3]);
				const PfxVector3& v2 = pfxReadVector3(&verts[idxs[d*3+2]*3]);
				PfxVector3 v01 = v1 - v0;
				PfxVector3 v02 = v2 - v0;
				normal += cross(v01, v02);
			}
		}
		normal = normalize(normal);

		pfxStoreVector3(normal,&onmls[c*3]);
	}
	for (PfxUInt32 c=0; c<num_idxs/3; ++c)
	{
		oidx[c*3+0] = idxs[c*3+0] + offset_idx;
		oidx[c*3+1] = idxs[c*3+1] + offset_idx;
		oidx[c*3+2] = idxs[c*3+2] + offset_idx;
	}
}

template<class TIsland>
int createRenderMeshExpanded(PfxLargeTriMesh* largeMesh)
{
	//setup the meshes to be rendered
	PfxUInt32 	num_verts	= 0;
	PfxUInt32 	num_tris	= 0;
	PfxFloat* 	verts		= 0;
	PfxFloat* 	normals		= 0;
	PfxUInt16*	tris		= 0;
	PfxUInt16*	org_tris	= 0;

	//	calcurate the number of the vertices for meshes to be rendered
	for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
		TIsland *island = (TIsland*)largeMesh->m_islands + d;
		num_verts += island->m_numVerts;
		num_tris  += island->m_numFacets;
	}

	verts		= (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * num_verts*3);
	normals		= (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * num_verts*3);
	tris		= (PfxUInt16*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxUInt16) * num_tris*3);
	org_tris	= (PfxUInt16*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxUInt16) * num_tris*3);
	
	SCE_PFX_ALWAYS_ASSERT(verts);
	SCE_PFX_ALWAYS_ASSERT(normals);
	SCE_PFX_ALWAYS_ASSERT(tris);
	SCE_PFX_ALWAYS_ASSERT(org_tris);
	
	if(largeMesh->m_numIslands > 0) {
		PfxUInt32 index_tris = 0;
		for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
			TIsland *island = (TIsland*)largeMesh->m_islands + d;
			for (PfxUInt32 e=0; e<island->m_numFacets; ++e) {
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[0];
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[1];
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[2];
			}
		}
		SCE_PFX_ASSERT(index_tris == num_tris*3);

		PfxUInt32 offset_tris	= 0;
		PfxUInt32 offset_verts	= 0;
		for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
			TIsland *island = (TIsland*)largeMesh->m_islands + d;
			_constructMesh((PfxFloat*)island->m_verts, island->m_numVerts, &org_tris[offset_tris], island->m_numFacets*3, offset_verts/3, verts+offset_verts, normals+offset_verts, tris+offset_tris);
			offset_tris	 += island->m_numFacets*3;
			offset_verts += island->m_numVerts*3;
		}
		SCE_PFX_ASSERT(offset_tris == num_tris*3);
		SCE_PFX_ASSERT(offset_verts == num_verts*3);
	}
	
	int renderMeshId = renderInitMesh(
		verts,		sizeof(PfxFloat)*3,
		normals,	sizeof(PfxFloat)*3,
		tris,		sizeof(PfxUInt16)*3,
		num_verts, num_tris);

	SCE_PFX_UTIL_FREE(verts);
	SCE_PFX_UTIL_FREE(normals);
	SCE_PFX_UTIL_FREE(tris);
	SCE_PFX_UTIL_FREE(org_tris);
	
	return renderMeshId;
}

template<class TIsland>
int createRenderMeshQuantized(PfxLargeTriMesh* largeMesh)
{
	//setup the meshes to be rendered
	PfxUInt32 	num_verts	= 0;
	PfxUInt32 	num_tris	= 0;
	PfxFloat* 	verts		= 0;
	PfxFloat* 	normals		= 0;
	PfxUInt16*	tris		= 0;
	PfxUInt16*	org_tris	= 0;
	PfxVector3* exp_verts	= 0;

	//	calcurate the number of the vertices for meshes to be rendered
	for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
		TIsland *island = (TIsland*)largeMesh->m_islands + d;
		num_verts += island->m_numVerts;
		num_tris  += island->m_numFacets;
	}

	verts		= (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * num_verts*3);
	normals		= (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * num_verts*3);
	tris		= (PfxUInt16*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxUInt16) * num_tris*3);
	org_tris	= (PfxUInt16*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxUInt16) * num_tris*3);
	exp_verts	= (PfxVector3*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxVector3) * num_verts);

	SCE_PFX_ALWAYS_ASSERT(verts);
	SCE_PFX_ALWAYS_ASSERT(normals);
	SCE_PFX_ALWAYS_ASSERT(tris);
	SCE_PFX_ALWAYS_ASSERT(org_tris);
	SCE_PFX_ALWAYS_ASSERT(exp_verts);

	if(largeMesh->m_numIslands > 0) {
		PfxUInt32 index_tris = 0;
		PfxUInt32 index_verts = 0;
		for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
			TIsland *island = (TIsland*)largeMesh->m_islands + d;
			for (PfxUInt32 e=0; e<island->m_numFacets; ++e) {
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[0];
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[1];
				org_tris[index_tris++] = island->m_facets[e].m_vertIds[2];
			}
			for(PfxUInt32 v=0; v<island->m_numVerts; v++) {
				exp_verts[index_verts++] = largeMesh->decodePosition(island->m_verts[v]);
			}
		}
		SCE_PFX_ASSERT(index_tris == num_tris*3);

		PfxUInt32 offset_tris	= 0;
		PfxUInt32 offset_verts	= 0;
		for (PfxUInt32 d=0; d<largeMesh->m_numIslands; ++d) {
			TIsland *island = (TIsland*)largeMesh->m_islands + d;
			_constructMesh(exp_verts+offset_verts/3, island->m_numVerts, &org_tris[offset_tris], island->m_numFacets*3, offset_verts/3, verts+offset_verts, normals+offset_verts, tris+offset_tris);
			offset_tris	 += island->m_numFacets*3;
			offset_verts += island->m_numVerts*3;
		}
		SCE_PFX_ASSERT(offset_tris == num_tris*3);
		SCE_PFX_ASSERT(offset_verts == num_verts*3);
	}
	
	int renderMeshId = renderInitMesh(
		verts,		sizeof(PfxFloat)*3,
		normals,	sizeof(PfxFloat)*3,
		tris,		sizeof(PfxUInt16)*3,
		num_verts, num_tris);
	
	SCE_PFX_UTIL_FREE(verts);
	SCE_PFX_UTIL_FREE(normals);
	SCE_PFX_UTIL_FREE(tris);
	SCE_PFX_UTIL_FREE(org_tris);
	SCE_PFX_UTIL_FREE(exp_verts);

	return renderMeshId;
}

int createRenderMesh(PfxConvexMesh *convexMesh)
{
	SCE_PFX_ALWAYS_ASSERT(convexMesh);
	
	PfxFloat*	verts = (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * convexMesh->m_numVerts * 3);
	PfxFloat*	nmls = (PfxFloat*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxFloat) * convexMesh->m_numVerts * 3);
	PfxUInt16*	idx	= (PfxUInt16*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxUInt16) * convexMesh->m_numIndices);
	
	SCE_PFX_ALWAYS_ASSERT(verts);
	SCE_PFX_ALWAYS_ASSERT(nmls);
	SCE_PFX_ALWAYS_ASSERT(idx);
	
	_constructMesh(convexMesh->m_verts, convexMesh->m_numVerts, convexMesh->m_indices, convexMesh->m_numIndices, 0, verts, nmls, idx);
	
	int renderMeshId = renderInitMesh(
		verts,	sizeof(PfxFloat)*3,
		nmls,	sizeof(PfxFloat)*3,
		idx,	sizeof(PfxUInt16)*3,
		convexMesh->m_numVerts, convexMesh->m_numIndices/3);
	
	SCE_PFX_UTIL_FREE(idx);
	SCE_PFX_UTIL_FREE(nmls);
	SCE_PFX_UTIL_FREE(verts);
	
	return renderMeshId;
}

int createRenderMesh(PfxLargeTriMesh *largeMesh)
{
	SCE_PFX_ALWAYS_ASSERT(largeMesh);
	
	int renderMeshId = -1;
	
	switch(largeMesh->m_type) {
		case SCE_PFX_LARGE_MESH_TYPE_EXPANDED_ARRAY:
		renderMeshId = createRenderMeshExpanded<PfxExpandedTriMesh>(largeMesh);
		break;
		
		case SCE_PFX_LARGE_MESH_TYPE_QUANTIZED_ARRAY:
		renderMeshId = createRenderMeshQuantized<PfxQuantizedTriMesh>(largeMesh);
		break;

		case SCE_PFX_LARGE_MESH_TYPE_EXPANDED_BVH:
		renderMeshId = createRenderMeshExpanded<PfxExpandedTriMeshBvh>(largeMesh);
		break;

		case SCE_PFX_LARGE_MESH_TYPE_QUANTIZED_BVH:
		renderMeshId = createRenderMeshQuantized<PfxQuantizedTriMeshBvh>(largeMesh);
		break;
		
		default:
		SCE_PFX_ALWAYS_ASSERT_MSG(false,"unknown type of large mesh");
	}

	return renderMeshId;
}
