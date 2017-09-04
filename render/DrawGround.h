#pragma once

#include "sim/Ground.h"
#include "render/DrawMesh.h"

class cDrawGround
{
public:
	static void BuildMesh(const cGround* ground, cDrawMesh* out_mesh);
	static void DrawRuler2D(const cGround* ground, const tVector& bound_min, const tVector& bound_max);

protected:
	static void BuildMeshPlane(const cGround* ground, cDrawMesh* out_mesh);
	static void BuildMeshVar2D(const cGround* ground, cDrawMesh* out_mesh);
	static void BuildMeshVar3D(const cGround* ground, cDrawMesh* out_mesh);
	static void BuildMeshDynamicObstacles3D(const cGround* ground, cDrawMesh* out_mesh);
};