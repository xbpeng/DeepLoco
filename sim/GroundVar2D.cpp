#include "GroundVar2D.h"
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <time.h>
#include <iostream>

const double gInvalidHeight = -std::numeric_limits<double>::infinity();

bool cGroundVar2D::ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params)
{
	cTerrainGen2D::LoadParams(json, out_params);
	return true;
}

cGroundVar2D::cGroundVar2D()
{
	ResetParams();
	mTerrainFunc = cTerrainGen2D::BuildFlat;

	Eigen::VectorXd terrain_params;
	cTerrainGen2D::GetDefaultParams(terrain_params);
	SetBlendParams(terrain_params);

	for (int s = 0; s < GetNumSegments(); ++s)
	{
		mSegments[s] = std::unique_ptr<tSegment>(new tSegment());
	}
}

cGroundVar2D::~cGroundVar2D()
{
}

void cGroundVar2D::Init(std::shared_ptr<cWorld> world, const tParams& params,
						const tVector& bound_min, const tVector& bound_max)
{
	ResetParams();
	mParams = params;
	mWorld = world;
	SetupRandGen();

	SetParamBlend(params.mBlend);
	InitSegments(bound_min, bound_max);
	FlagUpdate();
}

void cGroundVar2D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	double min_x = GetMinX();
	double mid_x = GetMidX();
	double max_x = GetMaxX();

	if (bound_max[0] < max_x && bound_min[0] > min_x)
	{
		// nothing to do
	}
	else if (bound_max[0] <= min_x || bound_min[0] >= max_x)
	{
		InitSegments(bound_min, bound_max);
		FlagUpdate();
	}
	else
	{
		int seg_id = 0;
		double seg_min_x = 0;
		double seg_max_x = 0;
		eAlignMode align_mode = eAlignMin;
		tVector fix_point = tVector::Zero();
		
		if (bound_max[0] >= max_x)
		{
			seg_id = GetSegID(0);
			seg_min_x = max_x;
			seg_max_x = seg_min_x + mParams.mGroundWidth;

			const auto& max_seg = GetMaxSegment();
			fix_point[0] = max_x;
			fix_point[1] = max_seg->GetEndHeight();
			align_mode = eAlignMin;
		}
		else
		{
			seg_id = GetSegID(1);
			seg_max_x = min_x;
			seg_min_x = seg_max_x - mParams.mGroundWidth;

			const auto& min_seg = GetMinSegment();
			fix_point[0] = min_x;
			fix_point[1] = min_seg->GetStartHeight();
			align_mode = eAlignMax;
		}

		BuildSegment(seg_id, seg_min_x, seg_max_x, align_mode, fix_point);
		mFlipSeg = GetSegID(0) == 0;
		FlagUpdate();
	}
}

void cGroundVar2D::Clear()
{
	ResetParams();
	ClearSegments();
}

double cGroundVar2D::SampleHeight(const tVector& pos) const
{
	return cGround::SampleHeight(pos);
}

double cGroundVar2D::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	double h = gInvalidHeight;
	out_valid_sample = false;
	for (int s = 0; s < GetNumSegments(); ++s)
	{
		const auto& seg = GetSegment(s);
		if (seg->ContainsPos(pos))
		{
			h = seg->SampleHeight(pos, out_valid_sample);
			break;
		}
	}
	return h;
}

void cGroundVar2D::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
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

void cGroundVar2D::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min = tVector::Ones() * std::numeric_limits<double>::infinity();
	out_max = tVector::Ones() * -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumSegments(); ++i)
	{
		tVector curr_min;
		tVector curr_max;
		mSegments[i]->CalcAABB(curr_min, curr_max);

		out_min = out_min.cwiseMin(curr_min);
		out_max = out_max.cwiseMax(curr_max);
	}
	out_min[3] = 0;
	out_max[3] = 0;
}

cGroundVar2D::eClass cGroundVar2D::GetGroundClass() const
{
	return eClassVar2D;
}

int cGroundVar2D::GetGridWidth() const
{
	int w = 0;
	for (int i = 0; i < GetNumSegments(); ++i)
	{
		w += mSegments[i]->GetGridWidth();
	}
	w -= 1; // minus 1 for overlap vertex between the two segments
	return w;
}

int cGroundVar2D::GetGridLength() const
{
	return mSegments[0]->GetGridLength();
}

tVector cGroundVar2D::GetVertex(int i, int j) const
{
	assert(i >= 0 && i < GetGridWidth());
	assert(j >= 0 && j < GetGridLength());

	const auto& min_seg = GetMinSegment();
	int seg_idx = 0;
	int min_grid_width = min_seg->GetGridWidth();
	if (i >= min_grid_width)
	{
		seg_idx = 1;
		i -= min_grid_width;
		++i;
	}
	
	const auto& seg = GetSegment(seg_idx);
	tVector vert = seg->GetVertex(i, j);
	return vert;
}

tVector cGroundVar2D::CalcGridCoord(const tVector& pos) const
{
	// careful, the grid coord might be outside the range of grid cells
	const auto& min_seg = GetMinSegment();
	int seg_idx = 0;
	double min_seg_max = min_seg->GetMaxX();
	int i_offset = 0;

	if (pos[0] > min_seg_max)
	{
		seg_idx = 1;
		i_offset = min_seg->GetGridWidth() - 1;
	}

	const auto& seg = GetSegment(seg_idx);
	tVector coord = seg->CalcGridCoord(pos);
	coord[0] += i_offset;

	return coord;
}

tVector cGroundVar2D::GetPos() const
{
	tVector aabb_min = tVector::Ones() * std::numeric_limits<double>::infinity();
	tVector aabb_max = tVector::Ones() * -std::numeric_limits<double>::infinity();
	CalcAABB(aabb_min, aabb_max);
	tVector pos = 0.5 * (aabb_max + aabb_min);
	return pos;
}

double cGroundVar2D::GetWidth() const
{
	// hack make this consistent
	double w = mSegments[0]->GetLength();
	return w;
}

int cGroundVar2D::GetNumSegments() const
{
	return gNumSegments;
}

void cGroundVar2D::SetTerrainFunc(cTerrainGen2D::tTerrainFunc func)
{
	mTerrainFunc = func;
}

void cGroundVar2D::ResetParams()
{
	mUpdateCount = 0;
	mFlipSeg = false;
}

int cGroundVar2D::GetBlendParamSize() const
{
	return static_cast<int>(cTerrainGen2D::eParamsMax);
}


bool cGroundVar2D::OutsideGrid(const tVector& coord) const
{
	return (coord[0] < 0 || coord[0] >= GetGridWidth()
			|| coord[1] < 0 || coord[1] >= GetGridLength());
}

void cGroundVar2D::InitSegments(const tVector& bound_min, const tVector& bound_max)
{
	ClearSegments();

	int num_verts = static_cast<int>(std::ceil(mParams.mGroundWidth / tSegment::gGridSpacingX + 1));
	double mid = 0.5 * (bound_max[0] + bound_min[0]);
	double w = mParams.mGroundWidth;
	for (int i = 0; i < GetNumSegments(); ++i)
	{
		int seg_id = GetSegID(i);
		tVector fix_point = tVector::Zero();
		eAlignMode align_mode = GetSegAlignMode(i);
		
		double bound_min = (align_mode == eAlignMax) ? -w : 0;
		double bound_max = (align_mode == eAlignMax) ? 0 : w;
		bound_min += mid;
		bound_max += mid;

		BuildSegment(seg_id, bound_min, bound_max, align_mode, fix_point);
	}
}

void cGroundVar2D::ClearSegments()
{
	for (int i = 0; i < GetNumSegments(); ++i)
	{
		mSegments[i]->Clear();
	}
	mFlipSeg = false;
}

int cGroundVar2D::GetSegID(int s) const
{
	if (mFlipSeg)
	{
		return (s == 0) ? 1 : 0;
	}
	return s;
}

const std::unique_ptr<cGroundVar2D::tSegment>& cGroundVar2D::GetSegment(int s) const
{
	int seg_id = GetSegID(s);
	return mSegments[seg_id];
}

std::unique_ptr<cGroundVar2D::tSegment>& cGroundVar2D::GetSegment(int s)
{
	int seg_id = GetSegID(s);
	return mSegments[seg_id];
}

const std::unique_ptr<cGroundVar2D::tSegment>& cGroundVar2D::GetMinSegment() const
{
	int seg_id = GetSegID(0);
	return mSegments[seg_id];
}

const std::unique_ptr<cGroundVar2D::tSegment>& cGroundVar2D::GetMaxSegment() const
{
	int seg_id = GetSegID(GetNumSegments() - 1);
	return mSegments[seg_id];
}

cGroundVar2D::eAlignMode cGroundVar2D::GetSegAlignMode(int seg_id) const
{
	if (mFlipSeg)
	{
		return (seg_id == 0) ? eAlignMin : eAlignMax;
	}
	return (seg_id == 0) ? eAlignMax : eAlignMin;
}

void cGroundVar2D::BuildSegment(int seg_id, double bound_min, double bound_max,
								eAlignMode align_mode, const tVector& fix_point)
{
	std::unique_ptr<tSegment>& seg = mSegments[seg_id];
	seg->Clear();

	bool contains_origin = (bound_min <= 0) && (bound_max >= 0);
	if (contains_origin)
	{
		AddPadding(seg_id, bound_min, bound_max);
	}
	(*mTerrainFunc)(bound_max - bound_min, mBlendParams, mRand, seg->mData);

	int num_verts = static_cast<int>(seg->mData.size());
	float end_h = 0;
	if (num_verts > 0)
	{
		end_h = (align_mode == eAlignMin) ? seg->mData[0] : seg->mData[num_verts - 1];
	}
	
	float h_offset = static_cast<float>(fix_point[1] - end_h);
	for (int i = 0; i < num_verts; ++i)
	{
		seg->mData[i] += h_offset;
	}

	double new_bound_min = (align_mode == eAlignMin) ? 
							(bound_min) :
							(bound_max - (num_verts - 1) * tSegment::gGridSpacingX);
	seg->Init(mWorld, new_bound_min, mParams.mFriction);
}

void cGroundVar2D::AddPadding(int seg_id, double bound_min, double bound_max)
{
	std::unique_ptr<tSegment>& seg = mSegments[seg_id];
	double flat_w = std::min(bound_max - bound_min, 1 - bound_min);
	cTerrainGen2D::BuildFlat(flat_w, mBlendParams, mRand, seg->mData);

	/*
	cTerrainGen2D::AddBox(5, 0.2, 0.5, seg->mData);
	// hack
	int i0 = 0;
	int i1 = 0;

	i0 = seg->mData.size();
	cTerrainGen2D::AddStep(4, -0.25, seg->mData);
	cTerrainGen2D::AddStep(5, 0.25, seg->mData);
	i1 = seg->mData.size();
	cTerrainGen2D::OverlaySlopes(0.1, 0, 0.5, 0.5, i0, i1, mRand, seg->mData);

	i0 = seg->mData.size();
	cTerrainGen2D::AddBox(6, 1, -2, seg->mData);
	cTerrainGen2D::AddBox(5, 0.2, 0.3, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.2, 0.5, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.2, -0.5, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.2, -0.5, seg->mData);
	i1 = seg->mData.size();
	cTerrainGen2D::OverlaySlopes(0.1, -0.25, 0, -0.25, i0, i1, mRand, seg->mData);

	i0 = seg->mData.size();
	cTerrainGen2D::AddBox(5, 0.1, -2, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.1, -2, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.1, -2, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.1, -2, seg->mData);
	cTerrainGen2D::AddBox(0.1, 0.1, -2, seg->mData);
	i1 = seg->mData.size();
	cTerrainGen2D::OverlaySlopes(0.1, -0.3, 0, -0.3, i0, i1, mRand, seg->mData);

	i0 = seg->mData.size();
	cTerrainGen2D::AddStep(5, -0.3, seg->mData);
	i1 = seg->mData.size();
	cTerrainGen2D::OverlaySlopes(0.1, -0.35, 0, -0.35, i0, i1, mRand, seg->mData);

	cTerrainGen2D::AddStep(4.7, 0.3, seg->mData);
	cTerrainGen2D::AddStep(2, 0.3, seg->mData);
	cTerrainGen2D::AddStep(2, 0.3, seg->mData);
	cTerrainGen2D::AddStep(2, 0.3, seg->mData);
	cTerrainGen2D::AddStep(2, 0.3, seg->mData);
	cTerrainGen2D::AddStep(2, 0.3, seg->mData);
	cTerrainGen2D::AddStep(5, -0.3, seg->mData);
	cTerrainGen2D::AddStep(2, -0.3, seg->mData);
	cTerrainGen2D::AddStep(2, -0.3, seg->mData);
	cTerrainGen2D::AddStep(2, -0.3, seg->mData);
	cTerrainGen2D::AddStep(2, -0.3, seg->mData);
	*/
}

double cGroundVar2D::GetMinX() const
{
	return GetMinSegment()->GetMinX();
}

double cGroundVar2D::GetMidX() const
{
	return GetMinSegment()->GetMaxX();
}

double cGroundVar2D::GetMaxX() const
{
	return GetMaxSegment()->GetMaxX();
}



//////////////////////////////////////////
// tSegment
//////////////////////////////////////////

const int cGroundVar2D::tSegment::gGridLength = 3;
const double cGroundVar2D::tSegment::gGridSpacingX = cTerrainGen2D::gVertSpacing;
const double cGroundVar2D::tSegment::gGridSpacingZ = 1;

cGroundVar2D::tSegment::tSegment()
{
	mType = eTypeStatic;
	mMinX = 0;
}

cGroundVar2D::tSegment::~tSegment()
{
	this->Clear();
}

void cGroundVar2D::tSegment::Init(const std::shared_ptr<cWorld>& world, double min_x, double friction)
{
	double world_scale = world->GetScale();
	double height_scale = 1;
	double x_scale = gGridSpacingX * world_scale;
	double z_scale = gGridSpacingZ * world_scale;

	const int up_axis = 1; // y axis
	const PHY_ScalarType data_type = PHY_FLOAT;
	const double h_pad = 0.1; // is this necessary for completely flat terrain?

	int grid_width = static_cast<int>(mData.size());
	int grid_length = GetGridLength();
	
	mMinX = min_x;

	tVector aabb_min = tVector::Zero();
	tVector aabb_max = tVector::Zero();

	aabb_min[0] = mMinX;
	aabb_max[0] = mMinX + (mData.size() - 1) * gGridSpacingX;
	aabb_min[1] = std::numeric_limits<double>::infinity();
	aabb_max[1] = -std::numeric_limits<double>::infinity();

	for (size_t i = 0; i < mData.size(); ++i)
	{
		double h = mData[i];
		aabb_min[1] = std::min(aabb_min[1], h);
		aabb_max[1] = std::max(aabb_max[1], h);

		mData[i] *= static_cast<float>(world_scale);
	}

	mData.resize(grid_width * grid_length);
	for (int i = 1; i < grid_length; ++i)
	{
		int k=0;
		for ( int j = i * grid_width; j < (i * grid_width + grid_width); j++ )
		{
			mData[j] = mData[k];
			++k;
		}
	}

	double min_height = world_scale * (aabb_min[1] - h_pad);
	double max_height = world_scale * (aabb_max[1] + h_pad);

	bool flip_quad_edges = false;
	btHeightfieldTerrainShape* height_field = new btHeightfieldTerrainShape(grid_width, grid_length, mData.data(), static_cast<btScalar>(height_scale),
		static_cast<btScalar>(min_height), static_cast<btScalar>(max_height), up_axis, data_type, flip_quad_edges);
	
	height_field->setLocalScaling(btVector3(static_cast<btScalar>(x_scale), 1, static_cast<btScalar>(z_scale)));
	mShape = std::unique_ptr<btCollisionShape>(height_field);

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mShape.get(), btVector3(0, 0, 0));
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(friction));
	
	tVector origin = 0.5 * (aabb_min + aabb_max);

	cSimObj::Init(world);
	SetPos(origin);
	UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
}

void cGroundVar2D::tSegment::Clear()
{
	RemoveFromWorld();

	delete mShape.release();
	mShape.reset();
	delete mSimBody.release();
	mSimBody.reset();

	mData.clear();
	mData.shrink_to_fit();
}

bool cGroundVar2D::tSegment::IsEmpty() const
{
	return mData.size() == 0;
}

const double cGroundVar2D::tSegment::GetMinX() const
{
	if (IsEmpty())
	{
		return std::numeric_limits<double>::infinity();
	}

	tVector aabb_min;
	tVector aabb_max;
	CalcAABB(aabb_min, aabb_max);
	return aabb_min[0];
}

const double cGroundVar2D::tSegment::GetMaxX() const
{
	if (IsEmpty())
	{
		return -std::numeric_limits<double>::infinity();
	}

	tVector aabb_min;
	tVector aabb_max;
	CalcAABB(aabb_min, aabb_max);
	return aabb_max[0];
}

int cGroundVar2D::tSegment::GetGridWidth() const
{
	return static_cast<int>(mData.size()) / GetGridLength();
}

int cGroundVar2D::tSegment::GetGridLength() const
{
	return gGridLength;
}

double cGroundVar2D::tSegment::GetWidth() const
{
	return (GetGridWidth() - 1) * gGridSpacingX;
}

double cGroundVar2D::tSegment::GetLength() const
{
	return (GetGridLength() - 1) * gGridSpacingZ;
}

tVector cGroundVar2D::tSegment::GetScaling() const
{
	double world_scale = mWorld->GetScale();
	btVector3 bt_scale = mShape->getLocalScaling();
	return tVector(bt_scale[0], bt_scale[1], bt_scale[2], 0) / world_scale;
}

tVector cGroundVar2D::tSegment::GetVertex(int i, int j) const
{
	assert(i >= 0 && i < GetGridWidth());
	assert(j >= 0 && j < GetGridLength());

	tVector origin = GetPos();
	tVector scaling = GetScaling();

	int w = GetGridWidth();
	int l = GetGridLength();

	tVector pos = origin;
	pos[0] += scaling[0] * (i - ((w - 1) * 0.5));
	pos[2] += scaling[2] * (j - ((l - 1) * 0.5));

	int idx = CalcDataIdx(i, j);
	double height = mData[idx];
	pos[1] = height * scaling[1];
	return pos;
}

int cGroundVar2D::tSegment::CalcDataIdx(int i, int j) const
{
	int w = GetGridWidth();
	int idx = j * w + i;
	return idx;
}

double cGroundVar2D::tSegment::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	tVector coord = CalcGridCoord(pos);

	double h = gInvalidHeight;
	out_valid_sample = !OutsideGrid(coord);
	coord = ClampCoord(coord);

	int i = static_cast<int>(coord[0]);
	int j = std::min(GetGridWidth() - 1, i + 1);

	double lerp = coord[0] - i;
	tVector a = GetVertex(i, 0);
	tVector b = GetVertex(j, 0);
	h = (1 - lerp) * a[1] + lerp * b[1];
	
	return h;
}

double cGroundVar2D::tSegment::GetStartHeight() const
{
	double scale = mWorld->GetScale();
	return mData[0] / scale;
}

double cGroundVar2D::tSegment::GetEndHeight() const
{
	double scale = mWorld->GetScale();
	return mData[mData.size() - 1] / scale;
}

tVector cGroundVar2D::tSegment::CalcGridCoord(const tVector& pos) const
{
	const double tol = 0.0001;
	// careful, the grid coord might be outside the range of grid cells
	int w = GetGridWidth();
	int l = GetGridLength();

	tVector origin = GetPos();
	tVector scale = GetScaling();
	tVector coord = pos - origin;
	coord[1] = coord[2];

	coord[0] /= scale[0];
	coord[1] /= scale[2];

	coord[0] += ((w - 1) * 0.5);
	coord[1] += ((l - 1) * 0.5);

	coord[2] = 0;
	coord[3] = 0;

	// if pos is just outside of the grid clamp it to the grid
	if (coord[0] > -tol && coord[0] < w - 1 + tol
		&& coord[1] > -tol && coord[1] < l - 1 + tol)
	{
		coord = ClampCoord(coord);
	}

	return coord;
}

bool cGroundVar2D::tSegment::OutsideGrid(const tVector& coord) const
{
	return (coord[0] < 0 || coord[0] > GetGridWidth() - 1
		|| coord[1] < 0 || coord[1] > GetGridLength() - 1);
}

tVector cGroundVar2D::tSegment::ClampCoord(const tVector& coord) const
{
	tVector clamped_coord = coord;
	clamped_coord[0] = cMathUtil::Clamp(coord[0], 0.0, GetGridWidth() - 1.0);
	clamped_coord[1] = cMathUtil::Clamp(coord[1], 0.0, GetGridLength() - 1.0);
	return clamped_coord;
}

bool cGroundVar2D::tSegment::ContainsPos(const tVector& pos) const
{
	return (pos[0] >= GetMinX()) && (pos[0] <= GetMaxX());
}