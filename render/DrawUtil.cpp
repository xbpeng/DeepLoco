#include "DrawUtil.h"
#include <GL/glew.h>

tVector cDrawUtil::gColor = tVector::Ones();
cShader cDrawUtil::gCopyProg = cShader();

const int gNumSlice = 16;
const int gNumStacks = 8;
const int gDiskSlices = 32;

std::unique_ptr<cDrawMesh> cDrawUtil::gPointMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gLineMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gQuadMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gBoxSolidMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gBoxWireMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gSphereMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gDiskMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gTriangleMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtil::gCylinderMesh = nullptr;

void cDrawUtil::InitDrawUtil()
{
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glFrontFace(GL_CCW);

	BuildShaders();
	BuildMeshes();
}

void cDrawUtil::DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	tVector a = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector b = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector c = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	tVector d = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	DrawQuad(a, b, c, d, draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	DrawBox(pos, size, tVector::Zero(), tVector::Ones(), draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max, eDrawMode draw_mode)
{
	if (draw_mode == eDrawWire)
	{
		DrawBoxWire(pos, size);
	}
	else
	{
		DrawBoxSolid(pos, size, tex_coord_min, tex_coord_max);
	}
}

void cDrawUtil::DrawBoxSolid(const tVector& pos, const tVector& size, const tVector& tex_coord_min, const tVector& tex_coord_max)
{
	GLenum gl_mode = GL_TRIANGLES;

	const int num_faces = 6;
	const int pos_len = num_faces * 6 * cMeshUtil::gPosDim;
	const int coord_len = num_faces * 6 * cMeshUtil::gCoordDim;

	tVector sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);

	tVector sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);

	const float pos_data[pos_len] = {
		ne0[0], ne0[1], ne0[2], // top
		nw0[0], nw0[1], nw0[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		ne1[0], ne1[1], ne1[2],
		ne0[0], ne0[1], ne0[2],

		se1[0], se1[1], se1[2],  // bottom
		sw1[0], sw1[1], sw1[2],
		sw0[0], sw0[1], sw0[2],
		sw0[0], sw0[1], sw0[2],
		se0[0], se0[1], se0[2],
		se1[0], se1[1], se1[2],

		se1[0], se1[1], se1[2], // front
		se0[0], se0[1], se0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		ne1[0], ne1[1], ne1[2],
		se1[0], se1[1], se1[2],

		sw0[0], sw0[1], sw0[2], // back
		sw1[0], sw1[1], sw1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		nw0[0], nw0[1], nw0[2],
		sw0[0], sw0[1], sw0[2],

		sw0[0], sw0[1], sw0[2], // left
		nw0[0], nw0[1], nw0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		se0[0], se0[1], se0[2],
		sw0[0], sw0[1], sw0[2],

		se1[0], se1[1], se1[2], // right
		ne1[0], ne1[1], ne1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		sw1[0], sw1[1], sw1[2],
		se1[0], se1[1], se1[2]
	};

	const float coord_data[coord_len] = {
		tex_coord_min[0], tex_coord_min[2], // top
		tex_coord_max[0], tex_coord_min[2],
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_min[2],

		tex_coord_max[0], tex_coord_min[2], // bottom
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_min[2],
		tex_coord_max[0], tex_coord_min[2],

		tex_coord_min[0], tex_coord_min[1], // front
		tex_coord_max[0], tex_coord_min[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_min[1],

		tex_coord_min[0], tex_coord_min[1], // back
		tex_coord_max[0], tex_coord_min[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_min[1],

		tex_coord_max[2], tex_coord_min[1], // left
		tex_coord_max[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_min[1],
		tex_coord_max[2], tex_coord_min[1],

		tex_coord_max[2], tex_coord_min[1], // right
		tex_coord_max[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_min[1],
		tex_coord_max[2], tex_coord_min[1]
	};


	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gBoxSolidMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
	attr_info.mAttribSize = sizeof(coord_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gCoordDim;
	gBoxSolidMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);
	
	gBoxSolidMesh->Draw(gl_mode);
}

void cDrawUtil::DrawBoxWire(const tVector& pos, const tVector& size)
{
	GLenum gl_mode = GL_LINES;

	const int num_edges = 12;
	const int pos_len = num_edges * 2 * cMeshUtil::gPosDim;

	tVector sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);

	tVector sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);

	const float pos_data[pos_len] = {
		ne0[0], ne0[1], ne0[2], // top
		nw0[0], nw0[1], nw0[2],
		nw0[0], nw0[1], nw0[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		ne1[0], ne1[1], ne1[2],
		ne1[0], ne1[1], ne1[2],
		ne0[0], ne0[1], ne0[2],

		se1[0], se1[1], se1[2],  // bottom
		sw1[0], sw1[1], sw1[2],
		sw1[0], sw1[1], sw1[2],
		sw0[0], sw0[1], sw0[2],
		sw0[0], sw0[1], sw0[2],
		se0[0], se0[1], se0[2],
		se0[0], se0[1], se0[2],
		se1[0], se1[1], se1[2],

		se1[0], se1[1], se1[2], // front
		ne1[0], ne1[1], ne1[2],

		sw0[0], sw0[1], sw0[2], // back
		nw0[0], nw0[1], nw0[2],
		
		ne0[0], ne0[1], ne0[2], // left
		se0[0], se0[1], se0[2],
		
		nw1[0], nw1[1], nw1[2], // right
		sw1[0], sw1[1], sw1[2],
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gBoxWireMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gBoxWireMesh->Draw(gl_mode);
}

void cDrawUtil::DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINE_LOOP : GL_TRIANGLES;
	PushMatrix();
	Translate(pos);
	Scale(tVector(side_len, side_len, side_len, 1));
	gTriangleMesh->Draw(gl_mode);
	PopMatrix();
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode)
{
	DrawQuad(a, b, c, d, tVector(0, 0, 0, 0), tVector(1, 0, 0, 0),
			tVector(1, 1, 0, 0), tVector(0, 1, 0, 0), draw_mode);
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d,
						const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
						eDrawMode draw_mode)
{
	const int num_verts = 4;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const int norm_len = num_verts * cMeshUtil::gNormDim;
	const int coord_len = num_verts * cMeshUtil::gCoordDim;

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;

	tVector normal = (b - a).cross3(d - a);
	double normal_len = normal.norm();
	if (normal_len == 0 )
	{
		normal = tVector(0, 0, 1, 0);
	}
	else
	{
		normal = (normal / normal_len);
	}

	const float pos_data[pos_len] = 
	{
		a[0], a[1], a[2],
		b[0], b[1], b[2],
		c[0], c[1], c[2],
		d[0], d[1], d[2]
	};

	const float norm_data[norm_len] = 
	{
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2],
		normal[0], normal[1], normal[2]
	};

	const float coord_data[coord_len] = 
	{
		coord_a[0], coord_a[1],
		coord_b[0], coord_b[1],
		coord_c[0], coord_c[1],
		coord_d[0], coord_d[1]
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeNormal;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gNormDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * norm_len, (GLubyte*)norm_data, 0, 1, &attr_info);

	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
	attr_info.mAttribSize = sizeof(norm_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gCoordDim;
	gQuadMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);

	gQuadMesh->Draw(gl_mode);
}

void cDrawUtil::DrawDisk(const tVector& pos, double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(pos);
	DrawDisk(r, draw_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawDisk(double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Scale(tVector(r, r, r, 1));

	if (draw_mode == eDrawWire)
	{
		gDiskMesh->Draw(GL_LINE_STRIP, 1);
	}
	else
	{
		gDiskMesh->Draw(GL_TRIANGLE_FAN);
	}

	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawPoint(const tVector& pt)
{
	const int num_verts = 1;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const float pos_data[pos_len] =
	{
		pt[0], pt[1], pt[2]
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gPointMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gPointMesh->Draw(GL_POINTS);
}

void cDrawUtil::DrawLine(const tVector& a, const tVector& b)
{
	const int num_verts = 2;
	const int pos_len = num_verts * cMeshUtil::gPosDim;
	const float pos_data[pos_len] =
	{
		a[0], a[1], a[2],
		b[0], b[1], b[2],
	};

	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gLineMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);

	gLineMesh->Draw(GL_LINES);
}

void cDrawUtil::DrawLineStrip(const tVectorArr& pts)
{
	int num_pts = static_cast<int>(pts.size());
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < num_pts - 1; ++i)
	{
		const tVector& a = pts[i];
		const tVector& b = pts[i + 1];
		glVertex3d(a[0], a[1], a[2]);
		glVertex3d(b[0], b[1], b[2]);
	}
	glEnd();
}

void cDrawUtil::DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode)
{
	tVector delta = b - a;
	tVector orthogonal = tVector(-delta[1], delta[0], 0, 0);
	orthogonal.normalize();

	tVector v0 = a - width * 0.5 * orthogonal;
	tVector v1 = b - width * 0.5 * orthogonal;
	tVector v2 = b + width * 0.5 * orthogonal;
	tVector v3 = a + width * 0.5 * orthogonal;

	DrawQuad(v0, v1, v2, v3, draw_mode);
}

void cDrawUtil::DrawCross(const tVector& pos, double size)
{
	DrawLine(tVector(pos[0] - 0.5 * size, pos[1], pos[2], 0),
			tVector(pos[0] + 0.5 * size, pos[1], pos[2], 0));
	DrawLine(tVector(pos[0], pos[1] - 0.5 * size, pos[2], 0),
			tVector(pos[0], pos[1] + 0.5 * size, pos[2], 0));
}

void cDrawUtil::DrawSphere(double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix(); 
	cDrawUtil::Scale(tVector(r, r, r, 1));
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINES : GL_TRIANGLES;
	gSphereMesh->Draw(gl_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawCylinder(double h, double r, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawWire) ? GL_LINES : GL_TRIANGLES;
	const int slices = gNumSlice;
	const int num_verts = 12 * slices;
	const int pos_size = cMeshUtil::gPosDim;
	const int norm_size = cMeshUtil::gNormDim;
	const int coord_size = cMeshUtil::gCoordDim;
	const int pos_len = num_verts * pos_size;

	float pos_data[pos_len];
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = (i * 2 * M_PI) / slices;
		double theta1 = ((i + 1) * 2 * M_PI) / slices;

		double x0 = r * std::cos(theta0);
		double z0 = r * std::sin(-theta0);

		double x1 = r * std::cos(theta1);
		double z1 = r * std::sin(-theta1);

		tVector n0 = tVector(x0, 0, z0, 0).normalized();
		tVector n1 = tVector(x1, 0, z1, 0).normalized();

		int pos_offset = i * 12 * pos_size;

		pos_data[pos_offset] = x0;
		pos_data[pos_offset + 1] = -0.5 * h;
		pos_data[pos_offset + 2] = z0;
		pos_data[pos_offset + 3] = x1;
		pos_data[pos_offset + 4] = -0.5 * h;
		pos_data[pos_offset + 5] = z1;
		pos_data[pos_offset + 6] = x1;
		pos_data[pos_offset + 7] = 0.5 * h;
		pos_data[pos_offset + 8] = z1;
		pos_data[pos_offset + 9] = x1;
		pos_data[pos_offset + 10] = 0.5 * h;
		pos_data[pos_offset + 11] = z1;
		pos_data[pos_offset + 12] = x0;
		pos_data[pos_offset + 13] = 0.5 * h;
		pos_data[pos_offset + 14] = z0;
		pos_data[pos_offset + 15] = x0;
		pos_data[pos_offset + 16] = -0.5 * h;
		pos_data[pos_offset + 17] = z0;

		pos_data[pos_offset + 18] = x0;
		pos_data[pos_offset + 19] = 0.5 * h;
		pos_data[pos_offset + 20] = z0;
		pos_data[pos_offset + 21] = x1;
		pos_data[pos_offset + 22] = 0.5 * h;
		pos_data[pos_offset + 23] = z1;
		pos_data[pos_offset + 24] = 0;
		pos_data[pos_offset + 25] = 0.5 * h;
		pos_data[pos_offset + 26] = 0;
		pos_data[pos_offset + 27] = 0;
		pos_data[pos_offset + 28] = -0.5 * h;
		pos_data[pos_offset + 29] = 0;
		pos_data[pos_offset + 30] = x1;
		pos_data[pos_offset + 31] = -0.5 * h;
		pos_data[pos_offset + 32] = z1;
		pos_data[pos_offset + 33] = x0;
		pos_data[pos_offset + 34] = -0.5 * h;
		pos_data[pos_offset + 35] = z0;
	}
	
	tAttribInfo attr_info;
	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
	attr_info.mAttribSize = sizeof(pos_data[0]);
	attr_info.mDataOffset = 0;
	attr_info.mDataStride = 0;
	attr_info.mNumComp = cMeshUtil::gPosDim;
	gCylinderMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);
	
	gCylinderMesh->Draw(gl_mode);
}

void cDrawUtil::DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode)
{
	const Eigen::Vector3d ref = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d n = Eigen::Vector3d(coeffs[0], coeffs[1], coeffs[2]);
	double c = coeffs[3];

	Eigen::Vector3d axis = ref.cross(n);
	double axis_len = axis.norm();
	double theta = 0;
	if (axis_len != 0)
	{
		axis /= axis_len;
		theta = std::acos(ref.dot(n));
	}

	Eigen::Vector3d offset = c * n;

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(tVector(offset[0], offset[1], offset[2], 0));
	if (theta != 0)
	{
		cDrawUtil::Rotate(theta, tVector(axis[0], axis[1], axis[2], 0));
	}
	DrawRect(tVector::Zero(), tVector(size, size, 0, 0), draw_mode);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawCapsule(double h, double r, eDrawMode draw_mode)
{
	cDrawUtil::PushMatrix();
	DrawCylinder(h, r, draw_mode);

	cDrawUtil::Translate(tVector(0, h * 0.5, 0, 0));
	DrawSphere(r, draw_mode);
	cDrawUtil::Translate(tVector(0, -h, 0, 0));
	DrawSphere(r, draw_mode);

	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawArrow2D(const tVector& start, const tVector& end, double head_size)
{
	GLboolean prev_enable;
	glGetBooleanv(GL_CULL_FACE, &prev_enable);
	glDisable(GL_CULL_FACE);

	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}
	
	dir[3] = 0;
	tVector axis = tVector(0, 0, 1, 0);
	tVector tangent = axis.cross3(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	tVector body_end = end - dir * head_size;

	tVector a = start - width * tangent;
	tVector b = body_end - width * tangent;
	tVector c = body_end + width * tangent;
	tVector d = start + width * tangent;
	DrawQuad(a, b, c, d);

	tVector e0 = body_end - tangent * head_size * 0.5f;
	tVector e1 = body_end + tangent * head_size * 0.5f;
	DrawQuad(end, e1, e0, end);

	if (prev_enable)
	{
		glEnable(GL_CULL_FACE);
	}
}

void cDrawUtil::DrawGrid2D(const tVector& origin, const tVector& size, double spacing, double big_spacing)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w;
	double min_y = origin(1) - h;
	double max_x = origin(0) + w;
	double max_y = origin(1) + h;

	const double offset_z = origin[2];

	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));

	for (int i = 0; i < 2; ++i)
	{
		double curr_spacing = (i == 0) ? spacing : big_spacing;
		cDrawUtil::SetLineWidth((i == 0) ? 1 : 2);
		double strip_w = (i == 0) ? 0.005 : 0.015;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			tVector a = tVector(x, min_y, offset_z, offset_z);
			tVector b = tVector(x, max_y, offset_z, offset_z);
			cDrawUtil::DrawLine(a, b);
		}
		for (double y = min_y - std::fmod(min_y, curr_spacing); y < max_y; y += curr_spacing)
		{
			tVector a = tVector(min_x, y, offset_z, offset_z);
			tVector b = tVector(max_x, y, offset_z, offset_z);
			cDrawUtil::DrawLine(a, b);
		}
	}
}

void cDrawUtil::DrawRuler2D(const tVector& origin, const tVector& size, 
			const tVector& col, double marker_spacing, double big_marker_spacing,
			double marker_h, double big_marker_h)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w * 0.5;
	double min_y = origin(1) - h * 0.5;
	double max_x = origin(0) + w * 0.5;
	double max_y = origin(1) + h * 0.5;

	const double offset_z = origin[2];

	glTexCoord2d(0, 0);
	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawRect(origin, size, cDrawUtil::eDrawSolid);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawLine(tVector(min_x, max_y, 0, 0), tVector(max_x, max_y, 0, 0));

	// draw markers
	cDrawUtil::SetColor(tVector(0.f, 0.f, 0.f, 1.f));

	for (int i = 0; i < 2; ++i)
	{
		bool big = (i == 1);
		cDrawUtil::SetLineWidth((big) ? 3 : 2);
		double curr_spacing = (big) ? big_marker_spacing : marker_spacing;
		double curr_h = (big) ? big_marker_h : marker_h;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			tVector a = tVector(x, max_y + curr_h * 0.5f, 0, 0);
			tVector b = tVector(x, max_y - curr_h * 0.5f, 0, 0);
			cDrawUtil::DrawLine(a, b);
		}
	}
}

void cDrawUtil::DrawSemiCircle(const tVector& pos, double r, int slices, 
							double min_theta, double max_theta, eDrawMode draw_mode)
{
	double d_theta = (max_theta - min_theta) / (slices - 1);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);
	
	glTexCoord2d(0, 0);
	glNormal3d(0, 0, 1);
	glVertex3d(pos[0], pos[1], pos[2]);
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = i * d_theta + min_theta;
		double theta1 = i * d_theta + min_theta;

		double x0 = r * std::cos(theta0) + pos[0];
		double y0 = r * std::sin(theta0) + pos[1];
		double x1 = r * std::cos(theta1) + pos[0];
		double y1 = r * std::sin(theta1) + pos[1];;

		glVertex3d(x0, y0, pos[2]);
		glVertex3d(x1, y1, pos[2]);
	}
	glEnd();
}

void cDrawUtil::DrawCalibMarker(const tVector& pos, double r, int slices, 
								const tVector& col0, const tVector& col1, 
								eDrawMode draw_mode)
{
	SetColor(col0);
	DrawSemiCircle(pos, r, slices, 0, M_PI * 0.5, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI, M_PI * 1.5, draw_mode);

	SetColor(col1);
	DrawSemiCircle(pos, r, slices, M_PI * 0.5, M_PI, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI * 1.5, M_PI * 2, draw_mode);
}

void cDrawUtil::DrawString(const std::string& str, const tVector& scale)
{
	std::istringstream str_stream(str);
	std::string curr_str = "";

	cDrawUtil::PushMatrix();
	const double default_scale = 0.0045f;
	cDrawUtil::Scale(tVector(default_scale * scale[0], default_scale * scale[1], scale[2], 0));

	while (std::getline(str_stream, curr_str))
	{
		cDrawUtil::PushMatrix();
		for (size_t i = 0; i < curr_str.size(); ++i)
		{
			glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, curr_str[i]);
		}
		cDrawUtil::PopMatrix();

		cDrawUtil::Translate(tVector(0, -1 / default_scale, 0, 0));
	}
	cDrawUtil::PopMatrix();
}

void cDrawUtil::DrawTexQuad(const cTextureDesc& tex, const tVector& pos, const tVector& size)
{
	tex.BindTex(GL_TEXTURE0);
	DrawQuad(tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
				tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
				tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
				tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
				tVector(0, 0, 0, 0), tVector(1, 0, 0, 0), tVector(1, 1, 0, 0), tVector(0, 1, 0, 0),
				eDrawSolid);
	tex.UnbindTex(GL_TEXTURE0);
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex)
{
	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PushMatrix();
	glLoadIdentity();

	GLint depth_test_param;
	glGetIntegerv(GL_DEPTH_TEST, &depth_test_param);
	glDisable(GL_DEPTH_TEST);

	GLint blend_param;
	glGetIntegerv(GL_BLEND, &blend_param);
	glDisable(GL_BLEND);

	SetColor(tVector::Ones());
	gCopyProg.Bind();
	dst_tex.BindBuffer();

	DrawTexQuad(src_tex, tVector::Zero(), tVector(2, 2, 0, 0));

	dst_tex.UnbindBuffer();
	gCopyProg.Unbind();

	if (depth_test_param == 1)
	{
		glEnable(GL_DEPTH_TEST);
	}

	if (blend_param == 1)
	{
		glEnable(GL_BLEND);
	}

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PopMatrix();

	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PopMatrix();
}

void cDrawUtil::ClearColor(const tVector& col)
{
	glClearColor(static_cast<float>(col[0]), static_cast<float>(col[1]),
				static_cast<float>(col[2]), static_cast<float>(col[3]));
	glClear(GL_COLOR_BUFFER_BIT);
}

void cDrawUtil::ClearDepth(double depth)
{
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void cDrawUtil::GLMultMatrix(const tMatrix& mat)
{
	glMultMatrixd(mat.data());
}

void cDrawUtil::PushMatrix()
{
	glPushMatrix();
}

void cDrawUtil::PopMatrix()
{
	glPopMatrix();
}

void cDrawUtil::Finish()
{
	glFinish();
}

void cDrawUtil::BuildMeshes()
{
	cMeshUtil::BuildPointMesh(gPointMesh);
	cMeshUtil::BuildLineMesh(gLineMesh);
	cMeshUtil::BuildQuadMesh(gQuadMesh);
	cMeshUtil::BuildBoxSolidMesh(gBoxSolidMesh);
	cMeshUtil::BuildBoxWireMesh(gBoxWireMesh);
	cMeshUtil::BuildSphereMesh(gNumStacks, gNumSlice, gSphereMesh);
	cMeshUtil::BuildDiskMesh(gDiskSlices, gDiskMesh);
	cMeshUtil::BuildTriangleMesh(gTriangleMesh);
	cMeshUtil::BuildCylinder(gNumSlice, gCylinderMesh);
}

const cShader& cDrawUtil::GetCopyProg()
{
	return gCopyProg;
}

void cDrawUtil::Translate(const tVector& trans)
{
	glTranslated(trans[0], trans[1], trans[2]);
}

void cDrawUtil::Scale(const tVector& scale)
{
	glScaled(scale[0], scale[1], scale[2]);
}

void cDrawUtil::Rotate(const tVector& euler)
{
	double theta;
	tVector axis;
	cMathUtil::EulerToAxisAngle(euler, axis, theta);
	Rotate(theta, axis);
}

void cDrawUtil::Rotate(double theta, const tVector& axis)
{
	glRotated(theta * gRadiansToDegrees, axis[0], axis[1], axis[2]);
}

void cDrawUtil::Rotate(const tQuaternion& q)
{
	double theta;
	tVector axis;
	cMathUtil::QuaternionToAxisAngle(q, axis, theta);
	Rotate(theta, axis);
}

void cDrawUtil::SetColor(const tVector& col)
{
	glColor4d(col[0], col[1], col[2], col[3]);
	gColor = col;
}

void cDrawUtil::SetLineWidth(double w)
{
	glLineWidth(static_cast<float>(w));
}

void cDrawUtil::SetPointSize(double pt_size)
{
	glPointSize(static_cast<float>(pt_size));
}

void cDrawUtil::BuildShaders()
{
	{
		gCopyProg.BuildShader("render/shaders/FullScreenQuad_VS.glsl", "render/shaders/DownSample_PS.glsl");
	}
}