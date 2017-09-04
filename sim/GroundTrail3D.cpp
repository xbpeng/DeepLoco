#include "sim/GroundTrail3D.h"
#include "util/PerlinNoise.h"
#include "util/FileUtil.h"
#include "util/PerlinNoise.h"

#define MAKE_EASIER

const double gTrailSegLen = 2;

cGroundTrail3D::tTrailSeg::tTrailSeg()
{
	mStart.setZero();
	mEnd.setZero();
	mWidth = 1;
}

void cGroundTrail3D::tTrailSeg::BuildAABB(tVector& out_min, tVector& out_max) const
{
	out_min = mStart.cwiseMin(mEnd) - 0.5 * tVector(mWidth, 0, mWidth, 0);
	out_max = mStart.cwiseMax(mEnd) + 0.5 * tVector(mWidth, 0, mWidth, 0);
}

cGroundTrail3D::cGroundTrail3D()
{
}

cGroundTrail3D::~cGroundTrail3D()
{
}

void cGroundTrail3D::Init(std::shared_ptr<cWorld> world, const tParams& params,
							const tVector& bound_min, const tVector& bound_max)
{
	cGroundHills3D::Init(world, params, bound_min, bound_max);
}

void cGroundTrail3D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	tVector curr_min;
	tVector curr_max;
	GetBounds(curr_min, curr_max);

	if (bound_min[0] > curr_min[0] && bound_min[2] > curr_min[2]
		&& bound_max[0] < curr_max[0] && bound_max[2] < curr_max[2])
	{
		// nothing to do
	}
	else if ((bound_min[0] > curr_max[0] && bound_min[2] > curr_max[2])
		|| (bound_max[0] < curr_min[0] && bound_max[2] < curr_min[2]))
	{
		InitSlabs(bound_min, bound_max);
		FlagUpdate();
	}
	else
	{
		double w = mParams.mGroundWidth;
		bool need_update[gNumSlabs] = { false };
		tVector mid = GetPos();

		if (bound_min[0] < curr_min[0])
		{
			curr_min[0] -= w;
			curr_max[0] = mid[0];
		}
		else if (bound_max[0] > curr_max[0])
		{
			curr_max[0] += w;
			curr_min[0] = mid[0];
		}

		if (bound_min[2] < curr_min[2])
		{
			curr_min[2] -= w;
			curr_max[2] = mid[2];
		}
		else if (bound_max[2] > curr_max[2])
		{
			curr_max[2] += w;
			curr_min[2] = mid[2];
		}

		UpdateTrail(curr_min, curr_max);

		mid = 0.5 * (curr_max + curr_min);
		for (int s = 0; s < gNumSlabs; ++s)
		{
			tVector slab_bound_min = tVector::Zero();
			tVector slab_bound_max = tVector::Zero();

			slab_bound_min[0] = (s % 2 == 0) ? -w : 0;
			slab_bound_max[0] = (s % 2 == 0) ? 0 : w;
			slab_bound_min[2] = (s < 2) ? -w : 0;
			slab_bound_max[2] = (s < 2) ? 0 : w;
			slab_bound_min += mid;
			slab_bound_max += mid;

			BuildSlab(s, slab_bound_min, slab_bound_max);
		}

		FlagUpdate();
	}
}

void cGroundTrail3D::Clear()
{
	cGroundHills3D::Clear();
	mTrail.clear();
}

cGroundTrail3D::eClass cGroundTrail3D::GetGroundClass() const
{
	return eClassTrail3D;
}

int cGroundTrail3D::GetNumSegs() const
{
	return static_cast<int>(mTrail.size());
}

const cGroundTrail3D::tTrailSeg& cGroundTrail3D::GetSeg(int i) const
{
	return mTrail[i];
}

int cGroundTrail3D::FindNearestSegment(const tVector& pos) const
{
	int nearest_seg = gInvalidIdx;
	double min_dist = std::numeric_limits<double>::infinity();

	int num_segs = GetNumSegs();
	for (int s = 0; s < num_segs; ++s)
	{
		const auto& curr_seg = GetSeg(s);
		tVector mid = 0.5 * (curr_seg.mEnd + curr_seg.mStart);
		tVector delta = pos - mid;
		delta[1] = 0;
		double curr_dist = delta.squaredNorm();

		if (curr_dist < min_dist)
		{
			nearest_seg = s;
			min_dist = curr_dist;
		}
	}

	return nearest_seg;
}

double cGroundTrail3D::CalcTrailDist(const tVector& pos_beg, const tVector& pos_end) const
{
	double dist = 0;

	int s_beg = FindNearestSegment(pos_beg);
	int s_end = FindNearestSegment(pos_end);

	for (int s = s_beg + 1; s < s_end; ++s)
	{
		const auto& seg = GetSeg(s);
		tVector delta = seg.mEnd - seg.mStart;
		delta[1] = 0;
		dist += delta.norm();
	}

	const auto& seg_beg = GetSeg(s_beg);
	tVector beg_dir = (seg_beg.mEnd - seg_beg.mStart).normalized();
	double d_beg = beg_dir.dot(pos_beg - seg_beg.mStart);
	dist += std::abs(d_beg);

	const auto& seg_end = GetSeg(s_end);
	tVector end_dir = (seg_end.mEnd - seg_end.mStart).normalized();
	double d_end = end_dir.dot(pos_end - seg_end.mStart);
	dist += std::abs(d_end);

	return dist;
}

void cGroundTrail3D::FindRandTrailPlacement(tVector& out_pos, tQuaternion& out_rot)
{
	const double aabb_pad = 1;
	int num_segs = GetNumSegs();
	int rand_seg_id = gInvalidIdx;
	
	tVector bound_min;
	tVector bound_max;
	CalcAABB(bound_min, bound_max);

	bool done = false;
	do
	{
		rand_seg_id = mRand.RandInt(0, num_segs);
		const auto& curr_seg = GetSeg(rand_seg_id);

		tVector seg_aabb_min;
		tVector seg_aabb_max;
		curr_seg.BuildAABB(seg_aabb_min, seg_aabb_max);
		seg_aabb_min += tVector(-aabb_pad, 0, -aabb_pad, 0);
		seg_aabb_max += tVector(aabb_pad, 0, aabb_pad, 0);

		done = cMathUtil::ContainsAABBXZ(seg_aabb_min, seg_aabb_max, bound_min, bound_max);
	} while (!done);
	
	const auto& seg = GetSeg(rand_seg_id);
	tVector seg_delta = seg.mEnd - seg.mStart;

	const tVector axis = tVector(0, 1, 0, 0);
	double theta = std::atan2(-seg_delta[2], seg_delta[0]);

	double t = mRand.RandDouble();
	out_pos = (1 - t) * seg.mEnd + t * seg.mStart;
	out_rot = cMathUtil::AxisAngleToQuaternion(axis, theta);
}

void cGroundTrail3D::InitSlabs(const tVector& bound_min, const tVector& bound_max)
{
	InitTrail(bound_min, bound_max);
	cGroundHills3D::InitSlabs(bound_min, bound_max);
}

void cGroundTrail3D::InitTrail(const tVector& bound_min, const tVector& bound_max)
{
	const double pad_len = 2.5;
	double h = mBlendParams[cTerrainGen3D::eParamsPathHeight0];
	double w0 = mBlendParams[cTerrainGen3D::eParamsPathWidth0];
	double w1 = mBlendParams[cTerrainGen3D::eParamsPathWidth1];

	mUVOffset = tVector(mRand.RandDouble(), mRand.RandDouble(), 0, 0);

	tVector mid = 0.5 * (bound_max + bound_min);
	tVector size = bound_max - bound_min;
	size = size.cwiseMax(mParams.mGroundWidth);
	tVector bound_min1 = mid - size;
	tVector bound_max1 = mid + size;

	mTrail.clear();

	tVector origin = tVector::Zero();
	if (cMathUtil::ContainsAABBXZ(tVector::Zero(), bound_min1, bound_max1))
	{
		origin = tVector(-0.5 * pad_len, h, 0, 0);
	}
	else
	{
		origin = tVector(bound_min1[0], h, 0, 0);
	}
	
	int num_pad_segs = static_cast<int>(std::ceil(pad_len / gTrailSegLen));
	tVector prev_pos = origin;
	double w = mRand.RandDouble(w0, w1);

	for (int s = 0; s < num_pad_segs; ++s)
	{
		tTrailSeg seg;
		seg.mStart = prev_pos;
		seg.mEnd = seg.mStart;
		seg.mEnd[0] += gTrailSegLen;
		seg.mWidth = w;

		mTrail.push_back(seg);
		prev_pos = seg.mEnd;
	}

	UpdateTrail(bound_min1, bound_max1);
}

void cGroundTrail3D::UpdateTrail(const tVector& bound_min, const tVector& bound_max)
{
	const tTrailSeg& last_seg = mTrail.back();
	const double step_h_min = mBlendParams[cTerrainGen3D::eParamsStepHeightMin];
	const double step_h_max = mBlendParams[cTerrainGen3D::eParamsStepHeightMax];
	const double step_prob = mBlendParams[cTerrainGen3D::eParamsTrailStepProb];
	const double w0 = mBlendParams[cTerrainGen3D::eParamsPathWidth0];
	const double w1 = mBlendParams[cTerrainGen3D::eParamsPathWidth1];
	const double w_delta_stdev = mBlendParams[cTerrainGen3D::eParamsPathWidthDeltaStdev];
	const double max_turn_delta = mBlendParams[cTerrainGen3D::eParamsPathMaxTurnDelta];

	double mean_w = 0.5 * (w0 + w1);
	double w_diff = std::abs(w0 - w1);
	double min_w = mean_w - 0.5 * w_diff;
	double max_w = mean_w + 0.5 * w_diff;
	tVector prev_pos = last_seg.mEnd;
	double prev_w = last_seg.mWidth;

	if (cMathUtil::ContainsAABBXZ(prev_pos, bound_min, bound_max))
	{
		double h = prev_pos[1];
		double w = last_seg.mWidth;

		tVector delta = last_seg.mEnd - last_seg.mStart;
		double curr_theta = std::atan2(-delta[2], delta[0]);

		bool done = false;
		while (!done)
		{
			tVector start_pos = prev_pos;
			const double theta_tol = 1;
			double max_d_theta = max_turn_delta;
			double curr_min_w = min_w;
			double curr_max_w = max_w;

#if defined(MAKE_EASIER)
			double h_val = CalcNoiseHeight(start_pos);
			if (h_val > 0)
			{
				curr_min_w = mean_w - 0.25 * w_diff;
			}
#endif
			double curr_w = prev_w;
			double dw = mRand.RandDoubleNorm(0, w_delta_stdev);
			curr_w += dw;
			curr_w = cMathUtil::Clamp(curr_w, curr_min_w, curr_max_w);

#if defined(MAKE_EASIER)
			if (curr_w < min_w + 0.25 * (max_w - min_w))
			{
				max_d_theta *= 0.5;
			}
#endif
			double d_theta = mRand.RandDouble(0, max_d_theta);

			double sign_rand = mRand.RandDouble(-1, 1);
			double theta_val = (std::abs(curr_theta) - theta_tol);
			theta_val = std::max(theta_val, 0.0);
			theta_val *= cMathUtil::Sign(curr_theta);

			double sign_threshold = theta_val / (0.6 * M_PI - theta_tol);
			bool neg = sign_rand < sign_threshold;
			d_theta = (neg) ? -d_theta : d_theta;
			
			curr_theta += d_theta;
			tVector dir = tVector(1, 0, 0, 0);
			tMatrix rot_mat = cMathUtil::RotateMat(tVector(0, 1, 0, 0), curr_theta);
			dir = rot_mat * dir;

			if (mRand.FlipCoin(step_prob))
			{
				double step_h = mRand.RandDouble(step_h_min, step_h_max);
				step_h = mRand.FlipCoin() ? -step_h : step_h;
				start_pos[1] += step_h;
			}

			tTrailSeg seg;
			seg.mStart = start_pos;
			seg.mEnd = seg.mStart;
			seg.mEnd += dir * gTrailSegLen;
			seg.mWidth = curr_w;

			mTrail.push_back(seg);
			prev_pos = seg.mEnd;
			prev_w = curr_w;

			done = !cMathUtil::ContainsAABBXZ(prev_pos, bound_min, bound_max);
		}
	}
}

void cGroundTrail3D::BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
										std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double default_h = mBlendParams[cTerrainGen3D::eParamsPathHeight1];
	
	int num_segs = GetNumSegs();
	for (int s = 0; s < num_segs; ++s)
	{
		const tTrailSeg& seg = GetSeg(s);
		RasterizeSeg(seg, bound_min, bound_max, out_data);
	}

	int res_x = std::sqrt(out_data.size());
	int res_z = std::sqrt(out_data.size());
	for (size_t idx = 0; idx < out_data.size(); ++idx)
	{
		int curr_flags = 0;
		if (out_data[idx] == cTerrainGen3D::gInvalidHeight)
		{
			int i = idx % res_x;
			int j = idx / res_z;
			double x = i / (res_x - 1.0);
			double z = j / (res_z - 1.0);
			x = (1 - x) * bound_min[0] + x * bound_max[0];
			z = (1 - z) * bound_min[2] + z * bound_max[2];

			double val = CalcNoiseHeight(tVector(x, 0, z, 0));
			val += default_h;
			out_data[idx] = val;
		}
		else
		{
			curr_flags |= 1 << cTerrainGen3D::eVertFlagEnableTex;
		}
		out_flags[idx] = curr_flags;
	}
}

double cGroundTrail3D::CalcNoiseHeight(const tVector& pos) const
{
	const double scale_mult = 0.005;
	const double amplitude_mult = 5;
	const double scale[] = { 1.0, 1.96, 4.03, 7.92, 16.7, 31.12};//, 1.67, 3.2, 6.4, 12.8, 25.6};
	const double amplitude[] = {1.0, 1.0/2, 1.0/4, 1.0/8, 1.0/16, 1.0/32, 1.0/64, 1.0/128, 1.0/256 };
	const int num_octaves = sizeof(scale) / sizeof(scale[0]);

	double val = 0;
	tVector uv = tVector(pos[0], pos[2], 0, 0);

	for (int i = 0; i < num_octaves; ++i)
	{
		double curr_scale = scale_mult * scale[i];
		double curr_amplitude = amplitude_mult * amplitude[i];
		double curr_val = SampleNoise(uv * curr_scale);
		curr_val = 2 * curr_val - 1;
		curr_val *= curr_amplitude;

		val += curr_val;
	}

	return val;
}

void cGroundTrail3D::RasterizeSeg(const tTrailSeg& seg, const tVector& bound_min, const tVector& bound_max, std::vector<float>& out_data)
{
	double w = seg.mWidth;
	tVector plane_pos_beg = seg.mStart;
	tVector plane_pos_end = seg.mEnd;
	plane_pos_beg[1] = 0;
	plane_pos_end[1] = 0;

	tVector strip_aabb_min;
	tVector strip_aabb_max;
	seg.BuildAABB(strip_aabb_min, strip_aabb_max);

	if (cMathUtil::IntersectAABBXZ(strip_aabb_min, strip_aabb_max, bound_min, bound_max))
	{
		const tVector& origin = bound_min;
		tVector size = bound_max - bound_min;
		double dist_sq_threshold = 0.25 * w * w;

		int res_x = std::sqrt(out_data.size());
		int res_z = std::sqrt(out_data.size());

		double h = seg.mStart[1];
		
		tVector plane_delta = (plane_pos_end - plane_pos_beg);
		double len = plane_delta.norm();
		tVector plane_dir = plane_delta /= len;

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
				tVector pos = tVector(origin[0] + (i * size[0]) / (res_x - 1), 0, 
									origin[2] + (j * size[2]) / (res_z - 1), 0);
				double d = (pos - plane_pos_beg).dot(plane_dir);
				d = cMathUtil::Clamp(d, 0.0, len);

				tVector nearest_pos = plane_pos_beg + d * plane_dir;
				double dist = (pos - nearest_pos).squaredNorm();

				if (dist <= dist_sq_threshold)
				{
					out_data[j * res_x + i] = h;
				}
			}
		}
	}
}