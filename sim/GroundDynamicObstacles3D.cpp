#include "sim/GroundDynamicObstacles3D.h"
#include "sim/TerrainGen3D.h"
#include "sim/SimBox.h"

const tVector gObstaclePosMin = tVector(-20, 0, -20, 0);
const tVector gObstaclePosMax = tVector(20, 0, 20, 0);
const double gDefaultHeight = 0;
const double gObstacleCharTurnDist = 5; // dist obstacle needs to be from the character before it can change directions

cGroundDynamicObstacles3D::tObstacle::tObstacle()
{
	mPosStart.setZero();
	mPosEnd.setZero();
	mSpeed = 1;
	mPhase = 0;
	mDir = eDirForward;
	mMoveType = eMovePingPong;
}

cGroundDynamicObstacles3D::tObstacle::~tObstacle()
{
}

void cGroundDynamicObstacles3D::tObstacle::Update(double time_elapsed, const tVector& char_pos)
{
	double dphase = time_elapsed / CalcCycleDur();
	dphase = (mDir == eDirForward) ? dphase : -dphase;
	mPhase += dphase;

	tVector new_pos = tVector::Zero();
	if (mMoveType == eMovePingPong)
	{
		new_pos = (1 - mPhase) * mPosStart + mPhase * mPosEnd;
		tVector char_delta = char_pos - new_pos;
		char_delta[1] = 0;
		double char_dist = char_delta.squaredNorm();

		if (char_dist > gObstacleCharTurnDist * gObstacleCharTurnDist)
		{
			if (mPhase <= 0)
			{
				mDir = eDirForward;
			}
			else if (mPhase >= 1)
			{
				mDir = eDirBackward;
			}
		}
	}
	else
	{
		new_pos = CalcPos();
		double d = (new_pos - mPosStart).dot(mPosEnd - mPosStart);
		new_pos = mPosStart + d * (mPosEnd - mPosStart);
		new_pos += time_elapsed * CalcVel();
	}

	mObj->SetPos(new_pos);
	mObj->SetRotation(tQuaternion::Identity());
}

tVector cGroundDynamicObstacles3D::tObstacle::CalcPos() const
{
	tVector pos = mObj->GetPos();
	return pos;
}

tVector cGroundDynamicObstacles3D::tObstacle::CalcVel() const
{
	double phase = mPhase;
	tVector delta = mPosEnd - mPosStart;
	if (mDir == eDirBackward)
	{
		delta = -delta;
	}
	tVector vel = delta.normalized() * mSpeed;
	return vel;
}

double cGroundDynamicObstacles3D::tObstacle::CalcCycleDur() const
{
	double dur = (mPosEnd - mPosStart).norm() / mSpeed;
	return dur;
}


cGroundDynamicObstacles3D::cGroundDynamicObstacles3D()
{
}

cGroundDynamicObstacles3D::~cGroundDynamicObstacles3D()
{
}

void cGroundDynamicObstacles3D::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	cGroundPlane::Init(world, params);
	BuildObstacles();
}

void cGroundDynamicObstacles3D::Clear()
{
	cGroundPlane::Clear();
	ClearObstacles();
}

void cGroundDynamicObstacles3D::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	cGroundPlane::Update(time_elapsed, bound_min, bound_max);

	if (GetNumObstacles() == 0)
	{
		BuildObstacles();
	}

	UpdateObstacles(time_elapsed);
}

int cGroundDynamicObstacles3D::GetNumObstacles() const
{
	return static_cast<int>(mObstacles.size());
}

const cSimObj& cGroundDynamicObstacles3D::GetObj(int i) const
{
	return *mObstacles[i].mObj;
}

double cGroundDynamicObstacles3D::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	double h;
	tVector vel;
	SampleHeightVel(pos, h, vel, out_valid_sample);
	return h;
}

void cGroundDynamicObstacles3D::SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel,
													bool& out_valid_sample) const
{
	out_h = gDefaultHeight;
	out_vel = tVector::Zero();

	for (int i = 0; i < GetNumObstacles(); ++i)
	{
		const tObstacle& obstacle = mObstacles[i];
		const auto& obj = obstacle.mObj;
		tVector aabb_min;
		tVector aabb_max;
		obj->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = obstacle.CalcVel();
			break;
		}
	}

	out_valid_sample = true;
}

tVector cGroundDynamicObstacles3D::FindRandFlatPos(const tVector& buffer_size)
{
	const double valid_h = gDefaultHeight;
	const tVector& bound_min = gObstaclePosMin;
	const tVector& bound_max = gObstaclePosMax;

	tVector pos = tVector::Zero();
	do
	{
		double u = mRand.RandDouble(0.2, 0.8);
		double v = mRand.RandDouble(0.2, 0.8);
		pos[0] = (1 - u) * bound_min[0] + u * bound_max[0];
		pos[2] = (1 - v) * bound_min[2] + v * bound_max[2];
		pos[1] = FindMaxBoundHeight(pos - 0.5 * buffer_size, pos + 0.5 * buffer_size);
	} while (pos[1] != valid_h);

	return pos;
}

void cGroundDynamicObstacles3D::SetChar(const std::shared_ptr<cSimCharacter>& character)
{
	mChar = character;
}

cGroundDynamicObstacles3D::eClass cGroundDynamicObstacles3D::GetGroundClass() const
{
	return eClassDynamicObstacles3D;
}

int cGroundDynamicObstacles3D::GetBlendParamSize() const
{
	return static_cast<int>(cTerrainGen3D::eParamsMax);
}

tVector cGroundDynamicObstacles3D::GetCharPos() const
{
	return mChar->GetRootPos();
}

void cGroundDynamicObstacles3D::BuildObstacles()
{
	int num_obstacles = static_cast<int>(mBlendParams[cTerrainGen3D::eParamsNumObstacles]);
	mObstacles.clear();
	mObstacles.reserve(num_obstacles);

	for (int i = 0; i < num_obstacles; ++i)
	{
		tObstacle curr_obstacle;
		BuildObstacle(curr_obstacle);
		mObstacles.push_back(curr_obstacle);
	}

	SortObstacles();
}

void cGroundDynamicObstacles3D::ClearObstacles()
{
	mObstacles.clear();
}

void cGroundDynamicObstacles3D::SortObstacles()
{
	std::sort(mObstacles.begin(), mObstacles.end(), 
		[](const tObstacle& a, const tObstacle& b)
	{
		tVector aabb_min0;
		tVector aabb_max0;
		tVector aabb_min1;
		tVector aabb_max1;
		a.mObj->CalcAABB(aabb_min0, aabb_max0);
		b.mObj->CalcAABB(aabb_min1, aabb_max1);

		return aabb_max0[1] > aabb_max1[1];
	});
}

void cGroundDynamicObstacles3D::BuildObstacle(tObstacle& out_obstacle)
{
	const double min_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth0];
	const double max_w = mBlendParams[cTerrainGen3D::eParamsObstacleWidth1];
	const double min_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight0];
	const double max_h = mBlendParams[cTerrainGen3D::eParamsObstacleHeight1];
	const double min_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed0];
	const double max_speed = mBlendParams[cTerrainGen3D::eParamsObstacleSpeed1];
	const double speed_lerp_pow = mBlendParams[cTerrainGen3D::eParamsObstacleSpeedLerpPow];

	tVector size = tVector(mRand.RandDouble(min_w, max_w),
							mRand.RandDouble(min_h, max_h),
							mRand.RandDouble(min_w, max_w), 0);
	tVector pos_start = tVector(mRand.RandDouble(gObstaclePosMin[0], gObstaclePosMax[0]), 0, 
								mRand.RandDouble(gObstaclePosMin[2], gObstaclePosMax[2]), 0);
	tVector pos_end = pos_start;
	pos_end = tVector(mRand.RandDouble(gObstaclePosMin[0], gObstaclePosMax[0]), 0, 
								mRand.RandDouble(gObstaclePosMin[2], gObstaclePosMax[2]), 0);

	pos_start[1] += 0.5 * size[1];
	pos_end[1] += 0.5 * size[1];

	double speed_lerp = mRand.RandDouble();
	speed_lerp = std::pow(speed_lerp, speed_lerp_pow);
	double speed = (1 - speed_lerp) * std::log(min_speed) + speed_lerp * std::log(max_speed);
	speed = std::exp(speed);

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
	out_obstacle.mPhase = mRand.RandDouble(0, 1);
	out_obstacle.mDir = mRand.FlipCoin() ? tObstacle::eDirForward : tObstacle::eDirBackward;
	out_obstacle.mObj = box;

	tVector curr_pos = out_obstacle.CalcPos();
	box->SetPos(curr_pos);
}

void cGroundDynamicObstacles3D::UpdateObstacles(double time_elapsed)
{
	tVector char_pos = GetCharPos();
	for (int i = 0; i < GetNumObstacles(); ++i)
	{
		tObstacle& curr_obstacle = mObstacles[i];
		curr_obstacle.Update(time_elapsed, char_pos);
	}
}

double cGroundDynamicObstacles3D::FindMaxBoundHeight(const tVector& aabb_min, const tVector& aabb_max) const
{
	double h = gDefaultHeight;
	
	for (int i = 0; i < GetNumObstacles(); ++i)
	{
		const tObstacle& obstacle = mObstacles[i];
		const auto& obj = obstacle.mObj;
		tVector obj_aabb_min;
		tVector obj_aabb_max;
		obj->CalcAABB(obj_aabb_min, obj_aabb_max);

		if (cMathUtil::IntersectAABBXZ(obj_aabb_min, obj_aabb_max, aabb_min, aabb_max))
		{
			h = obj_aabb_max[1];
			break;
		}
	}

	return h;
}