#include "sim/GroundObstacles3D.h"

const double gDefaultHeight = 0;

cGroundObstacles3D::cGroundObstacles3D()
{
}

cGroundObstacles3D::~cGroundObstacles3D()
{
}

tVector cGroundObstacles3D::FindRandFlatPos()
{
	const int x_res = 3;
	const int z_res = 3;
	const tVector search_bound = tVector(2, 0, 2, 0);

	const double valid_h = gDefaultHeight;

	tVector bound_min;
	tVector bound_max;
	GetBounds(bound_min, bound_max);

	tVector pos = tVector::Zero();
	double max_h = valid_h;
	do
	{
		double u = mRand.RandDouble(0.2, 0.8);
		double v = mRand.RandDouble(0.2, 0.8);
		pos[0] = (1 - u) * bound_min[0] + u * bound_max[0];
		pos[2] = (1 - v) * bound_min[2] + v * bound_max[2];
		max_h = valid_h;

		tVector aabb_min = pos - 0.5 * search_bound;
		tVector aabb_max = pos + 0.5 * search_bound;
		for (int s = 0; s < GetNumSegs(); ++s)
		{
			const auto& seg = GetSeg(s);
			const tVector& seg_aabb_min = seg.mStart;
			const tVector& seg_aabb_max = seg.mEnd;
			
			if (cMathUtil::IntersectAABBXZ(seg_aabb_min, seg_aabb_max, aabb_min, aabb_max))
			{
				double h = seg.mStart[1];
				max_h = h;
				break;
			}
		}
	} while (max_h != valid_h);

	pos[1] = max_h;

	return pos;
}

cGroundObstacles3D::eClass cGroundObstacles3D::GetGroundClass() const
{
	return eClassObstacles3D;
}

void cGroundObstacles3D::InitTrail(const tVector& bound_min, const tVector& bound_max)
{
	mUVOffset = tVector(mRand.RandDouble(), mRand.RandDouble(), 0, 0);

	tVector mid = 0.5 * (bound_max + bound_min);
	tVector size = bound_max - bound_min;
	size = size.cwiseMax(mParams.mGroundWidth);
	tVector bound_min1 = mid - size;
	tVector bound_max1 = mid + size;

	mTrail.clear();
	UpdateTrail(bound_min1, bound_max1);
}

void cGroundObstacles3D::UpdateTrail(const tVector& bound_min, const tVector& bound_max)
{
	CullSegs(bound_min, bound_max);

	const double origin_buffer = 2;
	const double density = mBlendParams[cTerrainGen3D::eParamsObstacleDensity];
	const double min_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth0];
	const double max_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth1];
	const double min_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight0];
	const double max_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight1];
	
	const tVector origin_buffer_min = tVector(-origin_buffer, 0, -origin_buffer, 0);
	const tVector origin_buffer_max = tVector(origin_buffer, 0, origin_buffer, 0);

	tVector curr_min;
	tVector curr_max;
	GetBounds(curr_min, curr_max);

	tVector intersect_min;
	tVector intersec_max;
	cMathUtil::CalcAABBIntersection(curr_min, curr_max, bound_min, bound_max, intersect_min, intersec_max);

	double area = (bound_max[0] - bound_min[0]) * (bound_max[2] - bound_min[2]);
	double intersect_area = (intersec_max[0] - intersect_min[0]) * (intersec_max[2] - intersect_min[2]);
	area -= intersect_area;
	int num_obstacles = static_cast<int>(area * density);

	for (int i = 0; i < num_obstacles; ++i)
	{
		double h = mRand.RandDouble(min_h, max_h);
		tVector min_pos = tVector(0, h, 0, 0);
		tVector max_pos = tVector(0, h, 0, 0);

		do
		{
			min_pos[0] = mRand.RandDouble(bound_min[0], bound_max[0]);
			min_pos[2] = mRand.RandDouble(bound_min[2], bound_max[2]);
			max_pos[0] = min_pos[0] + mRand.RandDouble(min_w, max_w);
			max_pos[2] = min_pos[2] + mRand.RandDouble(min_w, max_w);
		} while (cMathUtil::IntersectAABBXZ(origin_buffer_min, origin_buffer_max, min_pos, max_pos)
				|| cMathUtil::IntersectAABBXZ(curr_min, curr_max, min_pos, max_pos));

		tTrailSeg seg;
		seg.mStart = min_pos;
		seg.mEnd = max_pos;

		mTrail.push_back(seg);
	}
}

void cGroundObstacles3D::BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max,
											std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double default_h = mBlendParams[cTerrainGen3D::eParamsPathHeight1];

	int num_segs = GetNumSegs();
	for (int s = 0; s < num_segs; ++s)
	{
		const tTrailSeg& seg = GetSeg(s);
		RasterizeSeg(seg, bound_min, bound_max, out_data);
	}

	for (size_t idx = 0; idx < out_data.size(); ++idx)
	{
		int curr_flags = 0;
		if (out_data[idx] == cTerrainGen3D::gInvalidHeight)
		{
			out_data[idx] = gDefaultHeight;
			curr_flags |= 1 << cTerrainGen3D::eVertFlagEnableTex;
		}

		out_flags[idx] = curr_flags;
	}
}

void cGroundObstacles3D::RasterizeSeg(const tTrailSeg& seg, const tVector& bound_min, const tVector& bound_max, std::vector<float>& out_data)
{
	const tVector& strip_aabb_min = seg.mStart;
	const tVector& strip_aabb_max = seg.mEnd;

	if (cMathUtil::IntersectAABBXZ(strip_aabb_min, strip_aabb_max, bound_min, bound_max))
	{
		const tVector& origin = bound_min;
		tVector size = bound_max - bound_min;

		int res_x = std::sqrt(out_data.size());
		int res_z = std::sqrt(out_data.size());

		double h = seg.mStart[1];

		int i_min = std::ceil((strip_aabb_min[0] - origin[0]) / size[0] * (res_x - 1));
		int j_min = std::ceil((strip_aabb_min[2] - origin[2]) / size[2] * (res_z - 1));
		int i_max = std::floor((strip_aabb_max[0] - origin[0]) / size[0] * (res_x - 1));
		int j_max = std::floor((strip_aabb_max[2] - origin[2]) / size[2] * (res_z - 1));
		i_min = std::min(i_min, i_max);
		j_min = std::min(j_min, j_max);

		i_min = cMathUtil::Clamp(i_min, 0, res_x - 1);
		j_min = cMathUtil::Clamp(j_min, 0, res_z - 1);
		i_max = cMathUtil::Clamp(i_max, 0, res_x - 1);
		j_max = cMathUtil::Clamp(j_max, 0, res_z - 1);

		for (int j = j_min; j <= j_max; ++j)
		{
			for (int i = i_min; i <= i_max; ++i)
			{
				out_data[j * res_x + i] = h;
			}
		}
	}
}

void cGroundObstacles3D::CullSegs(const tVector& bound_min, const tVector& bound_max)
{
	int num_segs = GetNumSegs();
	int idx = 0;
	for (int s = 0; s < num_segs; ++s)
	{
		const tTrailSeg& seg = GetSeg(s);
		if (cMathUtil::IntersectAABBXZ(bound_min, bound_max, seg.mStart, seg.mEnd))
		{
			mTrail[idx] = seg;
			++idx;
		}
	}
	mTrail.resize(idx);
}