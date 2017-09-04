#include "sim/GroundHills3D.h"
#include "util/PerlinNoise.h"
#include "util/FileUtil.h"
#include "util/PerlinNoise.h"

const int gNoiseRes = 256;
const double gNoiseScale = 16;

cGroundHills3D::cGroundHills3D()
{
}

cGroundHills3D::~cGroundHills3D()
{
}

void cGroundHills3D::Init(std::shared_ptr<cWorld> world, const tParams& params,
							const tVector& bound_min, const tVector& bound_max)
{
	InitNoise(gNoiseRes);
	cGroundVar3D::Init(world, params, bound_min, bound_max);
}

void cGroundHills3D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	cGroundVar3D::Update(time_elapsed, bound_min, bound_max);
}

void cGroundHills3D::Clear()
{
	cGroundVar3D::Clear();
}

cGroundHills3D::eClass cGroundHills3D::GetGroundClass() const
{
	return eClassHills3D;
}

void cGroundHills3D::InitSlabs(const tVector& bound_min, const tVector& bound_max)
{
	mUVOffset = tVector(mRand.RandDouble(), mRand.RandDouble(), 0, 0);
	cGroundVar3D::InitSlabs(bound_min, bound_max);
}

void cGroundHills3D::BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
										std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = std::sqrt(out_data.size());
	int res_z = std::sqrt(out_data.size());
	for (size_t idx = 0; idx < out_data.size(); ++idx)
	{
		int curr_flags = 0;
		int i = idx % res_x;
		int j = idx / res_z;
		double x = i / (res_x - 1.0);
		double z = j / (res_z - 1.0);
		x = (1 - x) * bound_min[0] + x * bound_max[0];
		z = (1 - z) * bound_min[2] + z * bound_max[2];

		double val = CalcNoiseHeight(tVector(x, 0, z, 0));
		out_data[idx] = val;

		curr_flags |= 1 << cTerrainGen3D::eVertFlagEnableTex;

		out_flags[idx] = curr_flags;
	}
}

void cGroundHills3D::InitNoise(int res)
{
	cPerlinNoise noise_gen;
	noise_gen.SetScale(gNoiseScale);
	mNoise.resize(res, res);

	for (int j = 0; j < res; ++j)
	{
		for (int i = 0; i < res; ++i)
		{
			tVector p = tVector(i / (res - 1.0), j / (res - 1.0), 0.5, 0);
			double val = noise_gen.Eval(p);
			mNoise(i, j) = val;
		}
	}
}

double cGroundHills3D::SampleNoise(const tVector& uv) const
{
	double u = std::fmod(uv[0] + mUVOffset[0], 1.0);
	double v = std::fmod(uv[1] + mUVOffset[1], 1.0);
	u = (u < 0) ? (1 + u) : u;
	v = (v < 0) ? (1 + v) : v;

	u *= gNoiseRes;
	v *= gNoiseRes;
	int ui0 = static_cast<int>(u);
	int vi0 = static_cast<int>(v);
	u -= ui0;
	v -= vi0;

	ui0 %= gNoiseRes;
	vi0 %= gNoiseRes;
	int ui1 = (ui0 + 1) % gNoiseRes;
	int vi1 = (vi0 + 1) % gNoiseRes;

	double sw = mNoise(ui0, vi0);
	double se = mNoise(ui1, vi0);
	double nw = mNoise(ui0, vi1);
	double ne = mNoise(ui1, vi1);

	double bot = cMathUtil::Lerp(u, sw, se);
	double top = cMathUtil::Lerp(u, nw, ne);
	double val = cMathUtil::Lerp(v, bot, top);
	return val;
}

double cGroundHills3D::CalcNoiseHeight(const tVector& pos) const
{
	const double scale_mult = 0.001;
	const double amplitude_mult = 2;
	const double scale[] = { 1.0, 1.96, 4.03, 7.92, 16.7, 31.12};
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