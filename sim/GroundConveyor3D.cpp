#include "sim/GroundConveyor3D.h"
#include "sim/TerrainGen3D.h"
#include "sim/SimBox.h"

const tVector gObstaclePosMin = tVector(-20, 0, -20, 0);
const tVector gObstaclePosMax = tVector(20, 0, 20, 0);
const double gDefaultHeight = 0;
const double gObstacleCharTurnDist = 5; // dist obstacle needs to be from the character before it can change directions

cGroundConveyor3D::tStrip::tStrip()
{
	mAnchorPos.setZero();
	mLen = 1;
	mWidth = 1;
	mHead = 0;
}

double cGroundConveyor3D::tStrip::GetSliceLength() const
{
	double len = 0;
	int num_slices = GetNumSlices();
	if (num_slices > 0)
	{
		len = mLen / num_slices;
	}
	return len;
}

int cGroundConveyor3D::tStrip::GetNumSlices() const
{
	return static_cast<int>(mSliceObstacles.size());
}

int cGroundConveyor3D::tStrip::GetTailID() const
{
	int head_idx = mHead;
	int tail_idx = head_idx - 1;
	if (tail_idx < 0)
	{
		tail_idx += GetNumSlices();
	}
	int tail_id = mSliceObstacles[tail_idx];
	return tail_id;
}

void cGroundConveyor3D::tStrip::IncHead()
{
	mHead = (mHead + 1) % GetNumSlices();
}

cGroundConveyor3D::cGroundConveyor3D()
{
}

cGroundConveyor3D::~cGroundConveyor3D()
{
}

cGroundConveyor3D::eClass cGroundConveyor3D::GetGroundClass() const
{
	return eClassConveyor3D;
}

void cGroundConveyor3D::BuildObstacles()
{
	const int num_strips = mBlendParams[cTerrainGen3D::eParamsConveyorNumStrips];
	const int num_slices = mBlendParams[cTerrainGen3D::eParamsConveyorNumSlices];
	const double strip_spacing = mBlendParams[cTerrainGen3D::eParamsConveyorSpacing];
	const double strip_len = mBlendParams[cTerrainGen3D::eParamsConveyorStripLength];
	const double min_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth0];
	const double max_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth1];
	const double min_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed0];
	const double max_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed1];
	const double speed_lerp_pow = mBlendParams[cTerrainGen3D::eParamsObstacleSpeedLerpPow];
	
	int num_obstacles = num_strips * num_slices;

	mObstacles.clear();
	mStrips.clear();
	mObstacles.reserve(num_obstacles);
	mStrips.reserve(num_strips);

	double curr_x = strip_spacing;
	for (int i = 0; i < num_strips; ++i)
	{
		double strip_width = mRand.RandDouble(min_w, max_w);
		tVector strip_pos = tVector::Zero();
		strip_pos[0] = curr_x + 0.5 * strip_width;

		double speed_lerp = mRand.RandDouble();
		speed_lerp = std::pow(speed_lerp, speed_lerp_pow);
		double speed = (1 - speed_lerp) * std::log(min_speed) + speed_lerp * std::log(max_speed);
		speed = std::exp(speed);

		tStrip strip;
		BuildStrip(num_slices, strip_width, strip_len, speed, strip_pos, strip);
		mStrips.push_back(strip);

		curr_x += strip_spacing + strip.mWidth;
	}

	SortObstacles();
}

void cGroundConveyor3D::BuildStrip(int num_slices, double strip_width, double strip_len, double speed,
									const tVector& pos, tStrip& out_strip)
{
	
	double slice_l = strip_len / num_slices;
	tObstacle::eDir dir = mRand.FlipCoin() ? tObstacle::eDirForward : tObstacle::eDirBackward;
	
	out_strip.mAnchorPos = pos;
	out_strip.mLen = strip_len;
	out_strip.mWidth = strip_width;
	out_strip.mSliceObstacles.clear();

	double dz = 0.5 * strip_len;
	out_strip.mAnchorPos[2] += (dir == tObstacle::eDirForward) ? -dz : dz;

	for (int i = 0; i < num_slices; ++i)
	{
		tVector curr_pos = out_strip.mAnchorPos;
		double dz = (0.5 + num_slices - 1 - i) * slice_l;
		dz = (dir == tObstacle::eDirForward) ? dz : -dz;
		curr_pos[2] += dz;

		tObstacle curr_obstacle;
		BuildStripSlice(curr_pos, strip_width, slice_l, dir, speed, curr_obstacle);

		int obstacles_id = static_cast<int>(mObstacles.size());
		mObstacles.push_back(curr_obstacle);
		out_strip.mSliceObstacles.push_back(obstacles_id);
	}
}

void cGroundConveyor3D::BuildStripSlice(const tVector& pos, double strip_width, double strip_len, tObstacle::eDir dir, 
										double speed, tObstacle& out_obstacle)
{
	const double h = 1;
	const double h_pad = 0.01;
	double end_dist = 1;

	tVector size = tVector(strip_width, h, strip_len, 0);
	tVector pos_start = pos;
	tVector pos_end = pos_start;
	pos_end[2] += (dir == tObstacle::eDirForward) ? end_dist : -end_dist;

	pos_start[1] += h_pad - 0.5 * h;
	pos_end[1] += h_pad - 0.5 * h;

	cSimBox::tParams params;
	params.mSize = size;
	params.mPos = pos_start;
	params.mFriction = mParams.mFriction;

	std::shared_ptr<cSimBox> box = std::shared_ptr<cSimBox>(new cSimBox());
	box->Init(mWorld, params);
	box->UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
	box->SetKinematicObject(true);

	out_obstacle.mPosStart = pos_start;
	out_obstacle.mPosEnd = pos_end;
	out_obstacle.mSpeed = speed;
	out_obstacle.mPhase = 0;
	out_obstacle.mMoveType = tObstacle::eMoveForward;
	out_obstacle.mObj = box;

	tVector curr_pos = out_obstacle.CalcPos();
	box->SetPos(curr_pos);
}

int cGroundConveyor3D::GetNumStrips() const
{
	return static_cast<int>(mStrips.size());
}

void cGroundConveyor3D::UpdateObstacles(double time_elapsed)
{
	cGroundDynamicObstacles3D::UpdateObstacles(time_elapsed);
	UpdateStrips();
}

void cGroundConveyor3D::UpdateStrips()
{
	int num_strips = GetNumStrips();
	for (int s = 0; s < num_strips; ++s)
	{
		tStrip& strip = mStrips[s];
		int num_slices = static_cast<int>(strip.mSliceObstacles.size());
		double slice_len = strip.GetSliceLength();

		for (int i = 0; i < num_slices; ++i)
		{
			int curr_id = strip.mSliceObstacles[i];

			tObstacle& curr_slice = mObstacles[curr_id];
			tVector pos = curr_slice.CalcPos();
			tVector vel = curr_slice.CalcVel();

			bool at_end = (vel[2] > 0 && (pos[2] - slice_len / 2 > strip.mLen / 2))
						|| (vel[2] < 0 && (pos[2] + slice_len / 2 < -strip.mLen / 2));

			if (at_end)
			{
				int tail_id = strip.GetTailID();

				tObstacle& tail_slice = mObstacles[tail_id];
				tVector new_pos = tail_slice.CalcPos();

				if (vel[2] > 0)
				{
					new_pos[2] -= slice_len;
				}
				else
				{
					new_pos[2] += slice_len;
				}

				curr_slice.mObj->SetPos(new_pos);
				strip.IncHead();
			}
		}
	}
}