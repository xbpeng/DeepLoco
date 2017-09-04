#pragma once

#include "DrawUtil.h"
#include "util/MathUtil.h"
#include "sim/SimBox.h"
#include "sim/SimPlane.h"
#include "sim/SimCapsule.h"
#include "sim/SimSphere.h"

class cDrawObj
{
public:
	static void Draw(const cSimObj* obj, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawBox(const cSimBox* box, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawBox(const cSimBox* box, const tVector& tex_coord_min, const tVector& tex_coord_max, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawPlane(const cSimPlane* plane, double size, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawCapsule(const cSimCapsule* cap, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawSphere(const cSimSphere* box, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
};