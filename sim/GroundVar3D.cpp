#include "sim/GroundVar3D.h"
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <time.h>
#include <iostream>
#include "util/FileUtil.h"
#include "sim/TerrainGen3D.h"

bool cGroundVar3D::ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params)
{
	cTerrainGen3D::LoadParams(json, out_params);
	return true;
}

cGroundVar3D::cGroundVar3D()
{
	ResetParams();
	mTerrainFunc = cTerrainGen3D::BuildFlat;
}

cGroundVar3D::~cGroundVar3D()
{
}

void cGroundVar3D::Init(std::shared_ptr<cWorld> world, const tParams& params,
						const tVector& bound_min, const tVector& bound_max)
{
	ResetParams();
	mParams = params;
	mWorld = world;
	SetupRandGen();

	mParams.mGroundWidth = 40; // hack
	SetParamBlend(params.mBlend);
	InitSlabs(bound_min, bound_max);
	FlagUpdate();
}

void cGroundVar3D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
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
		bool need_update[gNumSlabs] = { false };
		tVector mid = tVector::Zero();
		mid = GetPos();
		
		int sw = GetSlabID(0);
		int se = GetSlabID(1);
		int nw = GetSlabID(2);
		int ne = GetSlabID(3);
		if (bound_min[0] < curr_min[0] || bound_max[0] > curr_max[0])
		{
			mSlabOrder[0] = se;
			mSlabOrder[1] = sw;
			mSlabOrder[2] = ne;
			mSlabOrder[3] = nw;

			if (bound_min[0] < curr_min[0])
			{
				mid[0] = mSlabs[sw].mMin[0];
				need_update[0] = true;
				need_update[2] = true;
			}
			else
			{
				mid[0] = mSlabs[se].mMax[0];
				need_update[1] = true;
				need_update[3] = true;
			}
		}

		sw = GetSlabID(0);
		se = GetSlabID(1);
		nw = GetSlabID(2);
		ne = GetSlabID(3);

		if (bound_min[2] < curr_min[2] || bound_max[2] > curr_max[2])
		{
			mSlabOrder[0] = nw;
			mSlabOrder[1] = ne;
			mSlabOrder[2] = sw;
			mSlabOrder[3] = se;

			if (bound_min[2] < curr_min[2])
			{
				mid[2] = mSlabs[sw].mMin[2];
				need_update[0] = true;
				need_update[1] = true;
			}
			else
			{
				mid[2] = mSlabs[nw].mMax[2];
				need_update[2] = true;
				need_update[3] = true;
			}
		}

		for (int s = 0; s < gNumSlabs; ++s)
		{
			if (need_update[s])
			{
				ClearSlab(s);
			}
		}

		double w = mParams.mGroundWidth;
		for (int s = 0; s < gNumSlabs; ++s)
		{
			if (need_update[s])
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
		}

		FlagUpdate();
	}
}

void cGroundVar3D::Clear()
{
	ResetParams();
	ClearSlabs();
}

double cGroundVar3D::SampleHeight(const tVector& pos) const
{
	return cGround::SampleHeight(pos);
}

double cGroundVar3D::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	double h = cTerrainGen3D::gInvalidHeight;
	out_valid_sample = false;
	for (int s = 0; s < GetNumSlabs(); ++s)
	{
		const tSlab& slab = GetSlab(s);
		if (slab.ContainsPos(pos))
		{
			h = slab.SampleHeight(pos, out_valid_sample);
			break;
		}
	}

	return h;
}

void cGroundVar3D::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
{
	int num_pos = static_cast<int>(pos.rows());
	out_h.resize(num_pos);
	for (int i = 0; i < num_pos; ++i)
	{
		tVector curr_pos = pos.row(i);
		double h = SampleHeight(curr_pos);
		out_h[i] = h;
	}
}

tVector cGroundVar3D::GetVertex(int i, int j) const
{
	int slab_idx = gInvalidIdx;
	int slab_i = gInvalidIdx;
	int slab_j = gInvalidIdx;
	LocateSlab(i, j, slab_idx, slab_i, slab_j);

	const tSlab& slab = GetSlab(slab_idx);
	tVector vert = slab.GetVertex(slab_i, slab_j);
	return vert;
}

void cGroundVar3D::LocateSlab(int i, int j, int& out_slab_idx, int& out_slab_i, int& out_slab_j) const
{
	int res_x;
	int res_z;
	GetRes(res_x, res_z);
	assert(i >= 0 && i < res_x);
	assert(j >= 0 && j < res_z);

	const tSlab& slab_sw = GetSlab(0);
	out_slab_i = i;
	out_slab_j = j;
	out_slab_idx = 0;
	if (i >= slab_sw.mResX - 1)
	{
		out_slab_idx += 1;
		out_slab_i -= slab_sw.mResX - 1;
	}

	if (j >= slab_sw.mResZ - 1)
	{
		out_slab_idx += 2;
		out_slab_j -= slab_sw.mResZ - 1;
	}
}

tVector cGroundVar3D::GetPos() const
{
	const tSlab& slab = GetSlab(0);
	tVector pos = slab.mMax;
	return pos;
}

void cGroundVar3D::GetRes(int& out_x_res, int& out_z_res) const
{
	out_x_res = GetSlab(0).mResX + GetSlab(1).mResX - 1;
	out_z_res = GetSlab(0).mResZ + GetSlab(2).mResZ - 1;
}


void cGroundVar3D::SetTerrainFunc(cTerrainGen3D::tTerrainFunc func)
{
	mTerrainFunc = func;
}

bool cGroundVar3D::HasSimBody() const
{
	const tSlab& slab = GetSlab(0);
	return slab.HasSimBody();
}

cGroundVar3D::eClass cGroundVar3D::GetGroundClass() const
{
	return eClassVar3D;
}

int cGroundVar3D::GetNumSlabs() const
{
	return static_cast<int>(gNumSlabs);
}

void cGroundVar3D::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min = std::numeric_limits<double>::infinity() * tVector::Ones();
	out_max = -std::numeric_limits<double>::infinity() * tVector::Ones();

	for (int s = 0; s < GetNumSlabs(); ++s)
	{
		const auto& slab = GetSlab(s);
		tVector curr_min;
		tVector curr_max;
		slab.CalcAABB(curr_min, curr_max);
		cMathUtil::CalcAABBUnion(out_min, out_max, curr_min, curr_max, out_min, out_max);
	}
	out_min[3] = 0;
	out_max[3] = 0;
}

bool cGroundVar3D::Output(const std::string& out_file) const
{
	bool succ = false;
	FILE* f = cFileUtil::OpenFile(out_file, "w");
	if (f != nullptr)
	{
		fprintf(f, "{\n");

		fprintf(f, "\"XSpacing\": %.5f,\n", cGroundVar3D::tSlab::gGridSpacingX);
		fprintf(f, "\"ZSpacing\": %.5f,\n", cGroundVar3D::tSlab::gGridSpacingZ);

		int res_x = 0;
		int res_z = 0;
		GetRes(res_x, res_z);

		fprintf(f, "\"HeightMap\": [\n");
		for (int j = res_z - 1; j >= 0; --j)
		{
			if (j != res_z - 1)
			{
				fprintf(f, ",\n");
			}
			fprintf(f, "[");

			for (int i = 0; i < res_x; ++i)
			{
				tVector vert = GetVertex(i, j);
				if (i != 0)
				{
					fprintf(f, ",\t");
				}
				fprintf(f, "%.5f", vert[1]);
			}

			fprintf(f, "]");
		}
		fprintf(f, "\n]");

		fprintf(f, "\n}");
		cFileUtil::CloseFile(f);
		succ = true;
	}
	else
	{
		printf("Failed to output ground to %s\n", out_file.c_str());
		succ = false;
	}
	return succ;
}

void cGroundVar3D::ResetParams()
{
	mUpdateCount = 0;
	for (int s = 0; s < gNumSlabs; ++s)
	{
		mSlabOrder[s] = s;
	}
}

int cGroundVar3D::GetBlendParamSize() const
{
	return static_cast<int>(cTerrainGen3D::eParamsMax);
}

void cGroundVar3D::ClearSlabs()
{
	for (int s = 0; s < gNumSlabs; ++s)
	{
		ClearSlab(s);
	}
}

void cGroundVar3D::ClearSlab(int s)
{
	tSlab& slab = GetSlab(s);
	slab.Clear();
}

void cGroundVar3D::InitSlabs(const tVector& bound_min, const tVector& bound_max)
{
	ClearSlabs();

	tVector mid = 0.5 * (bound_max + bound_min);
	double w = mParams.mGroundWidth;
    
    // HACK, to make sure randomness is consistent between slabs
    double randomness = mRand.RandDouble();
	for (int s = 0; s < gNumSlabs; ++s)
	{
        mRand.Seed(randomness);
		tVector fix_point = tVector::Zero();
		tVector bound_min = tVector::Zero();
		tVector bound_max = tVector::Zero();
		bound_min[0] = (s % 2 == 0) ? -w : 0;
		bound_max[0] = (s % 2 == 0) ? 0 : w;
		bound_min[2] = (s < 2) ? -w : 0;
		bound_max[2] = (s < 2) ? 0 : w;

		bound_min += mid;
		bound_max += mid;

		BuildSlab(s, bound_min, bound_max);
	}
}

int cGroundVar3D::GetSlabID(int s) const
{
	return mSlabOrder[s];
}

const cGroundVar3D::tSlab& cGroundVar3D::GetSlab(int s) const
{
	return mSlabs[GetSlabID(s)];
}

cGroundVar3D::tSlab& cGroundVar3D::GetSlab(int s)
{
	return mSlabs[GetSlabID(s)];
}

void cGroundVar3D::BuildSlab(int s, const tVector& bound_min, const tVector& bound_max)
{
	tSlab& slab = GetSlab(s);
	slab.Clear();
	
	tVector size = bound_max - bound_min;
	slab.mResX = cTerrainGen3D::CalcResX(size[0]);
	slab.mResZ = cTerrainGen3D::CalcResZ(size[2]);
	if (slab.mData.size() == 0)
	{
		slab.mData.resize(slab.mResX * slab.mResZ);
		slab.mFlags.resize(slab.mResX * slab.mResZ);
	}

	slab.mMin = bound_min;
	slab.mMax = bound_max;
	
	ClearSlabData(s);
	//FillSlabBorders(s);

	BuildSlabHeighData(bound_min, bound_max, slab.mData, slab.mFlags);

	slab.Init(mWorld, mParams.mFriction);
}

void cGroundVar3D::BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
										std::vector<float>& out_data, std::vector<int>& out_flags)
{
	(*mTerrainFunc)(bound_min, bound_max - bound_min, mBlendParams, mRand, out_data, out_flags);
}

void cGroundVar3D::GetBounds(tVector& bound_min, tVector& bound_max) const
{
	const tSlab& slab0 = GetSlab(0);
	const tSlab& slab1 = GetSlab(1);
	const tSlab& slab2 = GetSlab(2);
	const tSlab& slab3 = GetSlab(3);

	bound_min.setZero();
	bound_max.setZero();

	bound_min[0] = std::max(slab0.mMin[0], slab2.mMin[0]);
	bound_min[2] = std::max(slab0.mMin[2], slab1.mMin[2]);
	bound_max[0] = std::min(slab1.mMax[0], slab3.mMax[0]);
	bound_max[2] = std::min(slab2.mMax[2], slab3.mMax[2]);
}

void cGroundVar3D::ClearSlabData(int s)
{
	tSlab& slab = GetSlab(s);
	std::fill(slab.mData.begin(), slab.mData.end(), static_cast<float>(cTerrainGen3D::gInvalidHeight));
}

void cGroundVar3D::FillSlabBorders(int s)
{
	const int slab_res = std::sqrt(gNumSlabs);
	int n_hor = (s % slab_res == 0) ? (s + 1) : (s - 1);
	int n_vert = (s / slab_res == 0) ? (s + slab_res) : (s - slab_res);
	
	tSlab& curr_slab = GetSlab(s);
	const tSlab& slab_hor = GetSlab(n_hor);
	if (slab_hor.IsValid())
	{
		int src_i = (s < n_hor) ? 0 : (slab_hor.mResX - 1);
		int dst_i = (s < n_hor) ? (curr_slab.mResX - 1) : 0;
		for (int j = 0; j < curr_slab.mResZ; ++j)
		{
			double h = slab_hor.GetVertex(src_i, j)[1];
			curr_slab.mData[curr_slab.CalcDataIdx(dst_i, j)] = h;
		}
	}

	const tSlab& slab_vert = GetSlab(n_vert);
	if (slab_vert.IsValid())
	{
		int src_j = (s < n_vert) ? 0 : (slab_vert.mResZ - 1);
		int dst_j = (s < n_vert) ? (slab_vert.mResZ - 1) : 0;
		for (int i = 0; i < curr_slab.mResX; ++i)
		{
			double h = slab_vert.GetVertex(i, src_j)[1];
			curr_slab.mData[curr_slab.CalcDataIdx(i, dst_j)] = h;
		}
	}
}


//////////////////////////////////////////
// tSlab
//////////////////////////////////////////

const double cGroundVar3D::tSlab::gGridSpacingX = cTerrainGen3D::gVertSpacing;
const double cGroundVar3D::tSlab::gGridSpacingZ = cTerrainGen3D::gVertSpacing;

cGroundVar3D::tSlab::tSlab()
{
	mType = eTypeStatic;
	mResX = 0;
	mResZ = 0;
	mMin = std::numeric_limits<double>::infinity() * tVector::Ones();
	mMax = -std::numeric_limits<double>::infinity() * tVector::Ones();
}

cGroundVar3D::tSlab::~tSlab()
{
	this->Clear();
}

void cGroundVar3D::tSlab::Clear()
{
	RemoveFromWorld();

	delete mShape.release();
	mShape.reset();
	delete mSimBody.release();
	mSimBody.reset();

	mMin = std::numeric_limits<double>::infinity() * tVector::Ones();
	mMax = -std::numeric_limits<double>::infinity() * tVector::Ones();
}

void cGroundVar3D::tSlab::Init(const std::shared_ptr<cWorld>& world, double friction)
{
	double world_scale = world->GetScale();
	double height_scale = 1;
	double x_scale = gGridSpacingX * world_scale;
	double z_scale = gGridSpacingZ * world_scale;

	const int up_axis = 1; // y axis
	const PHY_ScalarType data_type = PHY_FLOAT;
	const double h_pad = 0.1; // is this necessary for completely flat terrain?

	for (size_t i = 0; i < mData.size(); ++i)
	{
		double h = mData[i];
		mMin[1] = std::min(mMin[1], h);
		mMax[1] = std::max(mMax[1], h);
		mData[i] *= static_cast<float>(world_scale);
	}

	double min_height = world_scale * (mMin[1] - h_pad);
	double max_height = world_scale * (mMax[1] + h_pad);

	bool flip_quad_edges = false;
	btHeightfieldTerrainShape* height_field = new btHeightfieldTerrainShape(mResX, mResZ, mData.data(), static_cast<btScalar>(height_scale),
		static_cast<btScalar>(min_height), static_cast<btScalar>(max_height), up_axis, data_type, flip_quad_edges);

	height_field->setLocalScaling(btVector3(static_cast<btScalar>(x_scale), 1, static_cast<btScalar>(z_scale)));
	mShape = std::unique_ptr<btCollisionShape>(height_field);

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mShape.get(), btVector3(0, 0, 0));
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(friction));

	tVector origin = 0.5 * (mMax + mMin);

	cSimObj::Init(world);
	SetPos(origin);
	UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
}

size_t cGroundVar3D::tSlab::GetNumVerts() const
{
	return mData.size();
}

tVector cGroundVar3D::tSlab::GetScaling() const
{
	double world_scale = mWorld->GetScale();
	btVector3 bt_scale = mShape->getLocalScaling();
	return tVector(bt_scale[0], bt_scale[1], bt_scale[2], 0) / world_scale;
}

tVector cGroundVar3D::tSlab::GetVertex(int i, int j) const
{
	assert(i >= 0 && i < mResX);
	assert(j >= 0 && j < mResZ);

	tVector origin = GetPos();
	tVector scaling = GetScaling();

	tVector pos = origin;
	pos[0] += scaling[0] * (i - ((mResX - 1) * 0.5));
	pos[2] += scaling[2] * (j - ((mResZ - 1) * 0.5));

	int idx = CalcDataIdx(i, j);
	double height = mData[idx];
	pos[1] = height * scaling[1];
	return pos;
}

int cGroundVar3D::tSlab::GetFlags(int i, int j) const
{
	assert(i >= 0 && i < mResX);
	assert(j >= 0 && j < mResZ);
	int idx = CalcDataIdx(i, j);
	return mFlags[idx];
}

int cGroundVar3D::tSlab::CalcDataIdx(int i, int j) const
{
	int idx = j * mResX + i;
	return idx;
}

bool cGroundVar3D::tSlab::ContainsPos(const tVector& pos) const
{
	return (pos[0] >= mMin[0]) && (pos[0] <= mMax[0])
		&& (pos[2] >= mMin[2]) && (pos[2] <= mMax[2]);
}

tVector cGroundVar3D::tSlab::CalcGridCoord(const tVector& pos) const
{
	const double tol = 0.0001;

	tVector origin = GetPos();
	tVector scale = GetScaling();
	tVector coord = pos - origin;
	coord[1] = coord[2];

	coord[0] /= scale[0];
	coord[1] /= scale[2];

	coord[0] += ((mResX - 1) * 0.5);
	coord[1] += ((mResZ - 1) * 0.5);

	coord[2] = 0;
	coord[3] = 0;

	return coord;
}

tVector cGroundVar3D::tSlab::ClampCoord(const tVector& coord) const
{
	tVector clamped_coord = coord;
	clamped_coord[0] = cMathUtil::Clamp(coord[0], 0.0, mResX - 1.0);
	clamped_coord[1] = cMathUtil::Clamp(coord[1], 0.0, mResZ - 1.0);
	return clamped_coord;
}

double cGroundVar3D::tSlab::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	const double tol = 0.0001;

	double h = cTerrainGen3D::gInvalidHeight;
	out_valid_sample = false;
	if (ContainsPos(pos))
	{
		tVector coord = CalcGridCoord(pos);
		coord[0] = cMathUtil::Clamp(coord[0], 0.0, mResX - 1.0 - tol);
		coord[1] = cMathUtil::Clamp(coord[1], 0.0, mResZ - 1.0 - tol);

		int i = static_cast<int>(coord[0]);
		int j = static_cast<int>(coord[1]);
		double lerp_x = coord[0] - i;
		double lerp_z = coord[1] - j;

		bool lower_tri = (lerp_z <= lerp_x);
		int i0 = (lower_tri) ? i		: i + 1;
		int j0 = (lower_tri) ? j		: j + 1;
		int i1 = (lower_tri) ? i + 1	: i;
		int j1 = (lower_tri) ? j		: j + 1;
		int i2 = (lower_tri) ? i + 1	: i;
		int j2 = (lower_tri) ? j + 1	: j;

		tVector v0 = GetVertex(i0, j0);
		tVector v1 = GetVertex(i1, j1);
		tVector v2 = GetVertex(i2, j2);
		
		tVector b_coord = tVector(lerp_x, lerp_z, 0, 0);
		tVector c0;
		tVector c1;
		tVector c2;
		if (lower_tri)
		{
			c0 = tVector(0, 0, 0, 0);
			c1 = tVector(1, 0, 0, 0);
			c2 = tVector(1, 1, 0, 0);
		}
		else
		{
			c0 = tVector(1, 1, 0, 0);
			c1 = tVector(0, 1, 0, 0);
			c2 = tVector(0, 0, 0, 0);
		}

		tVector b = cMathUtil::CalcBarycentric(b_coord, c0, c1, c2);
		h = b[0] * v0[1] + b[1] * v1[1] + b[2] * v2[1];
		
		out_valid_sample = true;
	}

	return h;
}

bool cGroundVar3D::tSlab::IsValid() const
{
	bool valid = (mMin[0] < mMax[0]) && (mMin[2] < mMax[2]);
	return valid;
}
