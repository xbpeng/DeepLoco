#include "DrawGround.h"
#include "DrawUtil.h"
#include "sim/GroundPlane.h"
#include "sim/GroundVar2D.h"
#include "sim/GroundVar3D.h"

const double gLineSpacing = 0.1;
const double gMarkerSpacing = 0.20;
const double gBigMarkerSpacing = gMarkerSpacing * 5;
const double gMarkerH = 0.04;
const double gBigMarkerH = 0.075;

void cDrawGround::BuildMesh(const cGround* ground, cDrawMesh* out_mesh)
{
	cGround::eClass ground_class = ground->GetGroundClass();
	switch (ground_class)
	{
	case cGround::eClassPlane:
		BuildMeshPlane(ground, out_mesh);
		break;
	case cGround::eClassVar2D:
		BuildMeshVar2D(ground, out_mesh);
		break;
	case cGround::eClassVar3D:
	case cGround::eClassHills3D:
	case cGround::eClassTrail3D:
	case cGround::eClassObstacles3D:
		BuildMeshVar3D(ground, out_mesh);
		break;
	case cGround::eClassDynamicObstacles3D:
	case cGround::eClassConveyor3D:
		BuildMeshDynamicObstacles3D(ground, out_mesh);
		break;
	default:
		assert(false); // unsupported ground type
		break;
	}
}

void cDrawGround::DrawRuler2D(const cGround* ground, const tVector& bound_min, const tVector& bound_max)
{
	// draw markers
	cDrawUtil::SetColor(tVector(0.f, 0.f, 0.f, 1.f));
	double min_x = bound_min[0];
	double max_x = bound_max[0];
	tVector ground_origin = ground->GetPos();

	tVector aabb_min;
	tVector aabb_max;
	ground->CalcAABB(aabb_min, aabb_max);

	cDrawUtil::SetLineWidth(1);
	glBegin(GL_LINE_STRIP);
	for (double x = min_x - std::fmod(min_x, gLineSpacing) - gLineSpacing; x < max_x + gLineSpacing; x += gLineSpacing)
	{
		bool valid_sample = true;
		double ground_h = ground->SampleHeight(tVector(x, 0, ground_origin[2], 0), valid_sample);
		if (valid_sample)
		{
			glVertex3d(x, ground_h, aabb_max[2]);
		}
	}
	glEnd();

	for (int i = 0; i < 2; ++i)
	{
		bool big = (i == 1);
		cDrawUtil::SetLineWidth((big) ? 3.f : 2.f);
		double curr_spacing = (big) ? gBigMarkerSpacing : gMarkerSpacing;
		double curr_h = (big) ? gBigMarkerH : gMarkerH;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			bool valid_sample = true;
			double ground_h = ground->SampleHeight(tVector(x, 0, ground_origin[2], 0), valid_sample);

			if (valid_sample)
			{
				tVector a = tVector(x, ground_h + curr_h * 0.5f, aabb_max[2], 0);
				tVector b = tVector(x, ground_h - curr_h * 0.5f, aabb_max[2], 0);
				cDrawUtil::DrawLine(a, b);
			}
		}
	}
}

void cDrawGround::BuildMeshPlane(const cGround* ground, cDrawMesh* out_mesh)
{
	assert(ground->GetGroundClass() == cGround::eClassPlane
		|| ground->GetGroundClass() == cGround::eClassDynamicObstacles3D
		|| ground->GetGroundClass() == cGround::eClassConveyor3D);
	const cGroundPlane* ground_plane = reinterpret_cast<const cGroundPlane*>(ground);
	const tVector& center = ground_plane->GetPrevCenter();

	const tVector tex_size = tVector(0.5, 0.5, 0, 0);
	const int num_faces = 1;
	const int verts_per_face = 6;
	const int vert_size = 3;
	const int norm_size = 3;
	const int coord_size = 2;
	const double w = 200;

	tVector size = tVector(w, 0, w, 0);
	tVector ground_origin = ground->GetPos();

	tVector a = tVector(center[0] - 0.5 * size[0], ground_origin[1], center[2] - 0.5 * size[2], 0);
	tVector b = tVector(center[0] - 0.5 * size[0], ground_origin[1], center[2] + 0.5 * size[2], 0);
	tVector c = tVector(center[0] + 0.5 * size[0], ground_origin[1], center[2] + 0.5 * size[2], 0);
	tVector d = tVector(center[0] + 0.5 * size[0], ground_origin[1], center[2] - 0.5 * size[2], 0);

	tVector min_coord = a - ground_origin;
	tVector max_coord = c - ground_origin;
	min_coord[0] /= tex_size[0];
	min_coord[1] = min_coord[2] / tex_size[1];
	max_coord[0] /= tex_size[0];
	max_coord[1] = max_coord[2] / tex_size[1];

	tVector coord_a = tVector(min_coord[0], min_coord[1], 0, 0);
	tVector coord_b = tVector(min_coord[0], max_coord[1], 0, 0);
	tVector coord_c = tVector(max_coord[0], max_coord[1], 0, 0);
	tVector coord_d = tVector(max_coord[0], min_coord[1], 0, 0);
	
	const int vert_len = num_faces * verts_per_face * vert_size;
	const int norm_len = num_faces * verts_per_face * norm_size;
	const int coord_len = num_faces * verts_per_face * coord_size;
	const int idx_len = num_faces * verts_per_face;

	float vert_data[vert_len] =
	{
		a[0], a[1], a[2],
		b[0], b[1], b[2],
		c[0], c[1], c[2],
		c[0], c[1], c[2],
		d[0], d[1], d[2],
		a[0], a[1], a[2]
	};

	float norm_data[norm_len] = 
	{
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f
	};

	float coord_data[coord_len] =
	{
		coord_a[0], coord_a[1],
		coord_b[0], coord_b[1],
		coord_c[0], coord_c[1],
		coord_c[0], coord_c[1],
		coord_d[0], coord_d[1],
		coord_a[0], coord_a[1]
	};

	int idx_data[idx_len];
	for (int i = 0; i < idx_len; ++i)
	{
		idx_data[i] = i;
	}

	cMeshUtil::BuildDrawMesh(vert_data, vert_len, norm_data, norm_len, coord_data, coord_len, idx_data, idx_len, out_mesh);
}

void cDrawGround::BuildMeshVar2D(const cGround* ground, cDrawMesh* out_mesh)
{
	assert(ground->GetGroundClass() == cGround::eClassVar2D);
	const tVector tex_size = tVector(0.5, 0.5, 0, 0);
	const double y_pad = 10;
	const int verts_per_face = 6;
	const int vert_size = 3;
	const int norm_size = 3;
	const int coord_size = 2;

	const cGroundVar2D* ground_var = reinterpret_cast<const cGroundVar2D*>(ground);
	int grid_w = ground_var->GetGridWidth();
	double ground_w = ground_var->GetWidth();
	tVector ground_origin = ground_var->GetPos();

	tVector aabb_min;
	tVector aabb_max;
	ground_var->CalcAABB(aabb_min, aabb_max);
	double min_y = aabb_min[1] - y_pad;

	int num_faces = 3 * (grid_w - 1);
	std::vector<float> vert_data(num_faces * verts_per_face * vert_size);
	std::vector<float> norm_data(num_faces * verts_per_face * norm_size);
	std::vector<float> coord_data(num_faces * verts_per_face * coord_size);
	std::vector<int> idx_data(num_faces * verts_per_face);
	
	for (int i = 0; i < grid_w - 1; ++i)
	{
		tVector a = ground_var->GetVertex(i, 0);
		tVector b = ground_var->GetVertex(i + 1, 0);

		double curr_min_y = std::min(a[1], b[1]);
		curr_min_y = std::min(curr_min_y, min_y);

		tVector c = b;
		tVector d = a;
		c[1] = curr_min_y;
		d[1] = curr_min_y;

		double z0 = ground_origin[2] - 0.5 * ground_w;
		double z1 = ground_origin[2] + 0.5 * ground_w;
		double a_coord = a[0] / tex_size[0];
		double b_coord = b[0] / tex_size[0];
		b_coord -= a_coord;
		a_coord = a_coord - static_cast<int>(a_coord);
		b_coord += a_coord;

		double slope = std::abs((b[1] - a[1]) / (b[0] - a[0]));
		if (slope > 1)
		{
			a_coord = 0;
			b_coord = 0;
		}

		tVector coord_a0 = tVector(a_coord, z0 / tex_size[1], 0, 0);
		tVector coord_b0 = tVector(b_coord, z0 / tex_size[1], 0, 0);
		tVector coord_a1 = tVector(a_coord, z1 / tex_size[1], 0, 0);
		tVector coord_b1 = tVector(b_coord, z1 / tex_size[1], 0, 0);

		tVector coord_c = tVector(0, 0, 0, 0);
		tVector coord_d = tVector(0, 0, 0, 0);

		tVector norm0 = tVector(0, 0, -1, 0);
		tVector norm1 = tVector(0, 0, 1, 0);
		
		a[2] = z0;
		b[2] = z0;
		c[2] = z0;
		d[2] = z0;
		
		const tVector* verts0[] = { &a, &b, &c, &c, &d, &a };
		const tVector* norms0[] = { &norm0, &norm0, &norm0, &norm0, &norm0, &norm0 };
		const tVector* coords0[] = { &coord_d, &coord_c, &coord_c, &coord_c, &coord_d, &coord_c };
		for (int v = 0; v < verts_per_face; ++v)
		{
			int vert_offset = ((3 * i) * verts_per_face + v) * vert_size;
			int norm_offset = ((3 * i) * verts_per_face + v) * norm_size;
			int coord_offset = ((3 * i) * verts_per_face + v) * coord_size;

			const tVector* curr_vert = verts0[v];
			const tVector* curr_norm = norms0[v];
			const tVector* curr_coord = coords0[v];

			for (int k = 0; k < vert_size; ++k)
			{
				vert_data[vert_offset + k] = static_cast<float>((*curr_vert)[k]);
			}
			for (int k = 0; k < norm_size; ++k)
			{
				norm_data[norm_offset + k] = static_cast<float>((*curr_norm)[k]);
			}
			for (int k = 0; k < coord_size; ++k)
			{
				coord_data[coord_offset + k] = static_cast<float>((*curr_coord)[k]);
			}
		}

		a[2] = z1;
		b[2] = z1;
		c[2] = z1;
		d[2] = z1;
		
		const tVector* verts1[] = { &a, &d, &c, &c, &b, &a };
		const tVector* norms1[] = { &norm1, &norm1, &norm1, &norm1, &norm1, &norm1 };
		const tVector* coords1[] = { &coord_d, &coord_d, &coord_c, &coord_c, &coord_c, &coord_d };
		for (int v = 0; v < verts_per_face; ++v)
		{
			int vert_offset = ((3 * i + 1) * verts_per_face + v) * vert_size;
			int norm_offset = ((3 * i + 1) * verts_per_face + v) * norm_size;
			int coord_offset = ((3 * i + 1) * verts_per_face + v) * coord_size;

			const tVector* curr_vert = verts1[v];
			const tVector* curr_norm = norms1[v];
			const tVector* curr_coord = coords1[v];

			for (int k = 0; k < vert_size; ++k)
			{
				vert_data[vert_offset + k] = static_cast<float>((*curr_vert)[k]);
			}
			for (int k = 0; k < norm_size; ++k)
			{
				norm_data[norm_offset + k] = static_cast<float>((*curr_norm)[k]);
			}
			for (int k = 0; k < coord_size; ++k)
			{
				coord_data[coord_offset + k] = static_cast<float>((*curr_coord)[k]);
			}
		}


		tVector a1 = tVector(a[0], a[1], z0, 0);
		tVector b1= tVector(a[0], a[1], z1, 0);
		tVector c1 = tVector(b[0], b[1], z1, 0);
		tVector d1 = tVector(b[0], b[1], z0, 0);
		tVector norm2 = (b1 - a1).cross3(d1 - a1).normalized();

		const tVector* verts2[] = { &a1, &b1, &c1, &c1, &d1, &a1 };
		const tVector* norms2[] = { &norm2, &norm2, &norm2, &norm2, &norm2, &norm2 };
		const tVector* coords2[] = { &coord_a0, &coord_a1, &coord_b1, &coord_b1, &coord_b0, &coord_a0 };
		for (int v = 0; v < verts_per_face; ++v)
		{
			int vert_offset = ((3 * i + 2) * verts_per_face + v) * vert_size;
			int norm_offset = ((3 * i + 2) * verts_per_face + v) * norm_size;
			int coord_offset = ((3 * i + 2) * verts_per_face + v) * coord_size;

			const tVector* curr_vert = verts2[v];
			const tVector* curr_norm = norms2[v];
			const tVector* curr_coord = coords2[v];

			for (int k = 0; k < vert_size; ++k)
			{
				vert_data[vert_offset + k] = static_cast<float>((*curr_vert)[k]);
			}
			for (int k = 0; k < norm_size; ++k)
			{
				norm_data[norm_offset + k] = static_cast<float>((*curr_norm)[k]);
			}
			for (int k = 0; k < coord_size; ++k)
			{
				coord_data[coord_offset + k] = static_cast<float>((*curr_coord)[k]);
			}
		}
	}

	for (int i = 0; i < idx_data.size(); ++i)
	{
		idx_data[i] = i;
	}

	cMeshUtil::BuildDrawMesh(vert_data.data(), static_cast<int>(vert_data.size()),
							norm_data.data(), static_cast<int>(norm_data.size()),
							coord_data.data(), static_cast<int>(coord_data.size()),
							idx_data.data(), static_cast<int>(idx_data.size()),
							out_mesh);
}

void cDrawGround::BuildMeshVar3D(const cGround* ground, cDrawMesh* out_mesh)
{
	assert(ground->GetGroundClass() == cGround::eClassVar3D
		|| ground->GetGroundClass() == cGround::eClassHills3D
		|| ground->GetGroundClass() == cGround::eClassTrail3D
		|| ground->GetGroundClass() == cGround::eClassObstacles3D);
	const tVector tex_size = tVector(0.5, 0.5, 0, 0);
	const int verts_per_face = 6;
	const int vert_size = 3;
	const int norm_size = 3;
	const int coord_size = 2;
	const double norm_threshold = 0.8;

	const cGroundVar3D* ground_var = reinterpret_cast<const cGroundVar3D*>(ground);
	int num_slabs = ground_var->GetNumSlabs();
	int num_faces = 0;
	for (int s = 0; s < num_slabs; ++s)
	{
		const auto& slab = ground_var->GetSlab(s);
		int res_x = slab.mResX;
		int res_z = slab.mResZ;
		int curr_num_faces = (res_x - 1) * (res_z - 1);
		assert(curr_num_faces > 0);
		num_faces += curr_num_faces;
	}

	std::vector<float> vert_data(num_faces * verts_per_face * vert_size);
	std::vector<float> norm_data(num_faces * verts_per_face * norm_size);
	std::vector<float> coord_data(num_faces * verts_per_face * coord_size);
	std::vector<int> idx_data(num_faces * verts_per_face);

	int face_count = 0;
	for (int s = 0; s < num_slabs; ++s)
	{
		const auto& slab = ground_var->GetSlab(s);
		int res_x = slab.mResX;
		int res_z = slab.mResZ;

		for (int j = 0; j < res_z - 1; ++j)
		{
			for (int i = 0; i < res_x - 1; ++i)
			{
				tVector vert0 = slab.GetVertex(i, j);
				tVector vert1 = slab.GetVertex(i, j + 1);
				tVector vert2 = slab.GetVertex(i + 1, j + 1);
				tVector vert3 = slab.GetVertex(i + 1, j);

				int flags0 = slab.GetFlags(i, j);
				int flags1 = slab.GetFlags(i, j + 1);
				int flags2 = slab.GetFlags(i + 1, j + 1);
				int flags3 = slab.GetFlags(i + 1, j);

				tVector coord0 = tVector(vert0[0] / tex_size[0], vert0[2] / tex_size[1], 0, 0);
				tVector coord1 = tVector(vert1[0] / tex_size[0], vert1[2] / tex_size[1], 0, 0);
				tVector coord2 = tVector(vert2[0] / tex_size[0], vert2[2] / tex_size[1], 0, 0);

				tVector coord3 = tVector(vert2[0] / tex_size[0], vert2[2] / tex_size[1], 0, 0);
				tVector coord4 = tVector(vert3[0] / tex_size[0], vert3[2] / tex_size[1], 0, 0);
				tVector coord5 = tVector(vert0[0] / tex_size[0], vert0[2] / tex_size[1], 0, 0);

				tVector normal0 = (vert1 - vert0).cross3(vert2 - vert0).normalized();
				tVector normal1 = (vert3 - vert2).cross3(vert0 - vert2).normalized();

				double norm_y0 = std::abs(normal0[1]);
				double norm_y1 = std::abs(normal1[1]);

				if (((flags0 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0)
					|| ((flags1 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0)
					|| ((flags2 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0))
				{
					coord0.setZero();
					coord1.setZero();
					coord2.setZero();
				}
				
				if (((flags2 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0)
					|| ((flags3 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0)
					|| ((flags0 & (1 << cTerrainGen3D::eVertFlagEnableTex)) == 0))
				{
					coord3.setZero();
					coord4.setZero();
					coord5.setZero();
				}
				
				const tVector* verts[] = { &vert0, &vert1, &vert2, &vert2, &vert3, &vert0 };
				const tVector* norms[] = { &normal0, &normal0, &normal0, &normal1, &normal1, &normal1 };
				const tVector* coords[] = { &coord0, &coord1, &coord2, &coord3, &coord4, &coord5 };

				for (int v = 0; v < verts_per_face; ++v)
				{
					int vert_offset = ((face_count + j * (res_x - 1) + i) * verts_per_face + v) * vert_size;
					int norm_offset = ((face_count + j * (res_x - 1) + i) * verts_per_face + v) * norm_size;
					int coord_offset = ((face_count + j * (res_x - 1) + i) * verts_per_face + v) * coord_size;

					const tVector* curr_vert = verts[v];
					const tVector* curr_norm = norms[v];
					const tVector* curr_coord = coords[v];

					for (int k = 0; k < vert_size; ++k)
					{
						vert_data[vert_offset + k] = static_cast<float>((*curr_vert)[k]);
					}
					for (int k = 0; k < norm_size; ++k)
					{
						norm_data[norm_offset + k] = static_cast<float>((*curr_norm)[k]);
					}
					for (int k = 0; k < coord_size; ++k)
					{
						coord_data[coord_offset + k] = static_cast<float>((*curr_coord)[k]);
					}
				}
			}
		}

		face_count += (res_x - 1) * (res_z - 1);
	}
	assert(face_count == num_faces);

	for (int i = 0; i < idx_data.size(); ++i)
	{
		idx_data[i] = i;
	}

	cMeshUtil::BuildDrawMesh(vert_data.data(), static_cast<int>(vert_data.size()),
							norm_data.data(), static_cast<int>(norm_data.size()),
							coord_data.data(), static_cast<int>(coord_data.size()),
							idx_data.data(), static_cast<int>(idx_data.size()), 
							out_mesh);
}

void cDrawGround::BuildMeshDynamicObstacles3D(const cGround* ground, cDrawMesh* out_mesh)
{
	BuildMeshPlane(ground, out_mesh);
}