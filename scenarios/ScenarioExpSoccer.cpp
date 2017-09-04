#include "scenarios/ScenarioExpSoccer.h"
#include "sim/SimSphere.h"
#include "sim/SoccerController.h"

//#define ENABLE_PHANTOM_BALL
//#define ENABLE_BOX_BALL

//const double gMaxTargetDist = 3;
//const double gMaxBallDist = 3;
const double gMaxTargetDist = 10;
const double gMaxBallDist = 10;
//const double gMaxTargetDist = 5;
//const double gMaxBallDist = 5;
const double gComBallDistThreshold = 2;

const double gBallRadius = 0.2;
const tVector gBallColor = tVector(0.8, 0.8, 0.8, 1);

double cScenarioExpSoccer::CalcReward() const
{
	const double desired_com_ball_vel = 1;
	const double desired_ball_target_vel = 1;

	double time_elapsed = mTime - mPrevTime;
	//double ball_w = 0.25;
	//double target_vel_w = 0.25;
	double com_ball_vel_w = 0.2;
	double com_ball_pos_w = 0.2;
	//double target_vel_w = 0.8;
	//double target_pos_w = 0;
	double target_vel_w = 0.4;
	double target_pos_w = 0.4;
	
	const double total_w = com_ball_vel_w + com_ball_pos_w + target_vel_w + target_pos_w;
	com_ball_vel_w /= total_w;
	com_ball_pos_w /= total_w;
	target_vel_w /= total_w;
	target_pos_w /= total_w;

	const double com_ball_vel_scale = 1.5;
	const double com_ball_pos_scale = 0.5;
	const double target_scale = 1.5;
	const double target_pos_scale = 0.5;

	double reward = 0;

	if (time_elapsed > 0)
	{
		bool fallen = HasFallen();
		if (!fallen)
		{
			tVector curr_com = mChar->CalcCOM();
			tVector ball_pos = GetBallPos();

			tVector com_delta = curr_com - mPrevCOM;
			tVector com_ball_delta = ball_pos - mPrevCOM;
			com_delta[1] = 0;
			com_ball_delta[1] = 0;
			tVector com_ball_dir = com_ball_delta.normalized();

			double com_ball_dist = com_ball_delta.squaredNorm();
			double com_ball_vel = com_ball_dir.dot(com_delta);
			com_ball_vel /= time_elapsed;
			double com_ball_vel_err = std::min(0.0, com_ball_vel - desired_com_ball_vel);
			com_ball_vel_err *= com_ball_vel_err;

			double com_ball_pos_err = com_ball_dist;

			tVector ball_delta = ball_pos - mPrevBallPos;
			tVector ball_target_delta = mTargetPos - mPrevBallPos;
			ball_delta[1] = 0;
			ball_target_delta[1] = 0;
			tVector ball_target_dir = ball_target_delta.normalized();

			double ball_target_vel = ball_target_dir.dot(ball_delta);
			ball_target_vel /= time_elapsed;
			double target_vel_err = std::min(0.0, ball_target_vel - desired_ball_target_vel);
			target_vel_err *= target_vel_err;

			double curr_ball_target_dist = (mTargetPos - ball_pos).squaredNorm();
			double target_pos_err = std::sqrt(curr_ball_target_dist);

			double com_ball_vel_reward = std::exp(-com_ball_vel_scale * com_ball_vel_err);
			double com_ball_pos_reward = std::exp(-com_ball_pos_scale * com_ball_pos_err);
			double target_vel_reward = std::exp(-target_scale * target_vel_err);
			target_vel_reward = (ball_target_vel > 0) ? target_vel_reward : 0;
			double target_pos_reward = std::exp(-target_pos_scale * target_pos_err);
			
			bool target_success = (curr_ball_target_dist < mTargetResetDist * mTargetResetDist)
									&& (com_ball_dist < gComBallDistThreshold * gComBallDistThreshold);
			if (target_success)
			{
				com_ball_vel_reward = 1;
				com_ball_pos_reward = 1;
				target_vel_reward = 1;
				target_pos_reward = 1;
			}

			reward = com_ball_vel_w * com_ball_vel_reward + com_ball_pos_w * com_ball_pos_reward 
					+ target_vel_w * target_vel_reward + target_pos_w * target_pos_reward;
		}
	}


#if defined(HACK_SOCCER_LLC)
	double hack_imitate_reward = cScenarioExpImitate::CalcReward();
	reward = 0.5 * reward + 0.5 * hack_imitate_reward;
#endif

	return reward;
}

cScenarioExpSoccer::cScenarioExpSoccer()
{
	mRandBallPosTimeMin = 200;
	mRandBallPosTimeMax = 200;
	mRandBallPosTimer = 0;
	mTargetResetDist = 0.5; // desired distance between ball and target
	mNumBallSpawns = 1;
	mRemoveBallAtGoal = false;
}

cScenarioExpSoccer::~cScenarioExpSoccer()
{
}

void cScenarioExpSoccer::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpHike::ParseArgs(parser);
	parser->ParseDouble("rand_ball_pos_time_min", mRandBallPosTimeMin);
	parser->ParseDouble("rand_ball_pos_time_max", mRandBallPosTimeMax);
	parser->ParseInt("num_ball_spawns", mNumBallSpawns);
	mNumBallSpawns = std::max(mNumBallSpawns, 1);

	parser->ParseBool("remove_ball_at_goal", mRemoveBallAtGoal);
}

void cScenarioExpSoccer::Init()
{
	cScenarioExpHike::Init();
	BuildBalls();
	ResetBallPosAll();
}

void cScenarioExpSoccer::Reset()
{
	cScenarioExpHike::Reset();
	BuildBalls();
	ResetBallPosAll();
	ResetTargetPos();
}

void cScenarioExpSoccer::Update(double time_elapsed)
{
	cScenarioExpHike::Update(time_elapsed);
	UpdateBallPos(time_elapsed);
}

std::string cScenarioExpSoccer::GetName() const
{
	return "Soccer Exploration";
}

void cScenarioExpSoccer::ResetParams()
{
	cScenarioExpHike::ResetParams();
	mPrevBallPos.setZero();
}

bool cScenarioExpSoccer::CheckResetTarget() const
{
	bool reset_target = (mRandTargetPosTimer <= 0);
	return reset_target;
}

void cScenarioExpSoccer::ClearObjs()
{
	cScenarioExpHike::ClearObjs();
	mBallObjHandles.clear();
}

void cScenarioExpSoccer::HandleNewActionUpdate()
{
	cScenarioExpHike::HandleNewActionUpdate();

	mPrevBallPos = GetBallPos();
}

bool cScenarioExpSoccer::EndEpisode() const
{
	bool is_end = cScenarioExpHike::EndEpisode();
	int num_balls = GetNumBalls();
	is_end |= (num_balls < 1);
	return is_end;
}

double cScenarioExpSoccer::GetRandTargetMaxDist() const
{
	return gMaxTargetDist;
}

double cScenarioExpSoccer::GetRandBallMaxDist() const
{
	return gMaxBallDist;
}

void cScenarioExpSoccer::BuildBalls()
{
	for (int i = 0; i < mNumBallSpawns; ++i)
	{
		int curr_handle = BuildBall();
		mBallObjHandles.push_back(curr_handle);
	}
	UpdateTargetBall();
}

int cScenarioExpSoccer::BuildBall()
{
	const double r = gBallRadius;
	const double mass = 0.1;
	const double linear_damping = 0.5;
	const double angular_damping = 0.5;
	const double friction = 0.5;

#if defined(ENABLE_BOX_BALL)
	cSimBox::tParams params;
	params.mSize = tVector(2 * r, 2 * r, 2 * r, 0);
	params.mPos = tVector(1, 1, 1, 0);
	params.mVel = tVector::Zero();
	params.mFriction = 0.2 * friction;
	params.mMass = mass;
	auto ball = std::shared_ptr<cSimBox>(new cSimBox());
#else
	cSimSphere::tParams params;
	params.mRadius = r;
	params.mPos = tVector(1, 1, 1, 0);
	params.mVel = tVector::Zero();
	params.mFriction = friction;
	params.mMass = mass;
	auto ball = std::shared_ptr<cSimSphere>(new cSimSphere());
#endif

#if defined(ENABLE_PHANTOM_BALL)
	short col_flags = cWorld::eContactFlagEnvironment;
	ball->SetColGroup(col_flags);
	ball->SetColMask(col_flags);
#endif

	ball->Init(mWorld, params);
	ball->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);
	ball->SetDamping(linear_damping, angular_damping);
	ball->DisableDeactivation();

	tObjEntry obj_entry;
	obj_entry.mObj = ball;
	obj_entry.mEndTime = std::numeric_limits<double>::infinity();
	obj_entry.mColor = gBallColor;

	int ball_handle = AddObj(obj_entry);

	return ball_handle;
}

const std::shared_ptr<cSimObj>& cScenarioExpSoccer::GetBall() const
{
	return GetBall(GetTargetBallHandle());
}

const std::shared_ptr<cSimObj>& cScenarioExpSoccer::GetBall(int ball_handle) const
{
	assert(ball_handle != gInvalidIdx);
	const tObjEntry& entry = mObjs[ball_handle];
	return entry.mObj;
}

tVector cScenarioExpSoccer::GetBallPos() const
{
	return GetBallPos(GetTargetBallHandle());
}

tVector cScenarioExpSoccer::GetBallPos(int ball_handle) const
{
	const auto& ball = GetBall(ball_handle);
	return ball->GetPos();
}

tVector cScenarioExpSoccer::CalcTargetPosDefault()
{
	//return cScenarioExpHike::CalcTargetPosDefault();
	const double max_dist = GetRandTargetMaxDist();
	const double min_dist = 0;

	tVector rand_pos = tVector::Zero();
	tVector ball_pos = tVector::Zero();
	int ball_handle = GetTargetBallHandle();
	if (ball_handle != gInvalidIdx)
	{
		ball_pos = GetBallPos(ball_handle);
	}

	double r = mRand.RandDouble(min_dist, max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	rand_pos[0] = ball_pos[0] + r * std::cos(theta);
	rand_pos[2] = ball_pos[2] + r * std::sin(theta);

	return rand_pos;
}

void cScenarioExpSoccer::UpdateBallPos(double time_elapsed)
{
	int ball_handle = GetTargetBallHandle();
	if (EnabledRandTargetPos())
	{
		mRandBallPosTimer -= time_elapsed;
		bool reset_ball = (mRandBallPosTimer <= 0);
		if (reset_ball)
		{
			ResetBallPos(ball_handle);
			mPrevBallPos = GetBallPos(ball_handle);
			ResetBallTimer();
		}
	}

	if (mRemoveBallAtGoal)
	{
		tObjEntry& obj_entry = mObjs[ball_handle];
		tVector pos = obj_entry.mObj->GetPos();
		tVector delta = pos - mTargetPos;
		delta[1] = 0;
		double dist = delta.squaredNorm();
		if (dist < mTargetResetDist * mTargetResetDist)
		{
			int handle = GetTargetBallHandle();
			RemoveObj(handle);
			
			if (EnabledRandTargetPos())
			{
				ResetTargetPos();
			}
		}
	}

#if defined(ENABLE_PHANTOM_BALL)
	const double dist_threshold = 0.2;
	tVector root_pos = mChar->GetRootPos();
	tVector ball_pos = GetBallPos();
	root_pos[1] = 0;
	ball_pos[1] = 0;

	double dist_sq = (ball_pos - root_pos).squaredNorm();
	if (dist_sq < dist_threshold)
	{
		tVector com_vel = mChar->CalcCOMVel();
		com_vel[1] = 0;
		tVector com_vel_dir = com_vel.normalized();
		ball_pos += 0.5 * com_vel_dir;
		SetBallPos(ball_pos);
	}
#endif
}

void cScenarioExpSoccer::ResetBallPosAll()
{
	for (int i = 0; i < GetNumBalls(); ++i)
	{
		ResetBallPos(mBallObjHandles[i]);
	}

	UpdateTargetBall();
	ResetBallTimer();
}

void cScenarioExpSoccer::ResetBallPos(int ball_handle)
{
	const double min_dist = 0.7;
	const double max_dist = GetRandBallMaxDist();
	assert(min_dist <= max_dist);

	tVector rand_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();
	double r = mRand.RandDouble(min_dist, max_dist);
	double theta = mRand.RandDouble(-M_PI, M_PI);
	rand_pos[0] = root_pos[0] + r * std::cos(theta);
	rand_pos[2] = root_pos[2] + r * std::sin(theta);

	SetBallPos(ball_handle, rand_pos);
}

void cScenarioExpSoccer::SetBallPos(const tVector& pos)
{
	SetBallPos(GetTargetBallHandle(), pos);
}

int cScenarioExpSoccer::GetNumBalls() const
{
	return static_cast<int>(mBallObjHandles.size());
}

void cScenarioExpSoccer::RemoveObj(int handle)
{
	int num_objs = GetNumObjs();
	cScenarioExpHike::RemoveObj(handle);

	// update ball handle since removing objects might have caused it to change
	auto end_ball = std::find(mBallObjHandles.begin(), mBallObjHandles.end(), num_objs - 1);
	if (end_ball != mBallObjHandles.end())
	{
		*end_ball = handle;

		if (handle == GetTargetBallHandle())
		{
			mBallObjHandles.pop_back();
			UpdateTargetBall();
		}
	}
}

void cScenarioExpSoccer::SetBallPos(int ball_handle, const tVector& pos)
{
	const auto& ball = GetBall(ball_handle);
	double r = gBallRadius;

	tVector ground_pos = pos;
	ground_pos[1] = r + mGround->SampleHeight(ground_pos);
	
	ball->SetPos(ground_pos);
	ball->SetRotation(tQuaternion::Identity());
	ball->SetLinearVelocity(tVector::Zero());
	ball->SetAngularVelocity(tVector::Zero());
}

void cScenarioExpSoccer::ResetBallTimer()
{
	mRandBallPosTimer = mRand.RandDouble(mRandBallPosTimeMin, mRandBallPosTimeMax);
}

int cScenarioExpSoccer::GetTargetBallHandle() const
{
	int handle = gInvalidIdx;
	int num_balls = GetNumBalls();
	if (num_balls > 0)
	{
		handle = mBallObjHandles[num_balls - 1];
	}
	return handle;
}

void cScenarioExpSoccer::UpdateTargetBall()
{
	tVector char_pos = mChar->GetRootPos();
	int num_balls = GetNumBalls();
	int idx = FindNearestBall(char_pos);
	if (idx != gInvalidIdx)
	{

		int nearest_handle = mBallObjHandles[idx];
		int last_handle = mBallObjHandles[num_balls - 1];
		mBallObjHandles[idx] = last_handle;
		mBallObjHandles[num_balls - 1] = nearest_handle;

		mPrevBallPos = GetBallPos();

		auto soccer_ctrl = std::dynamic_pointer_cast<cSoccerController>(mChar->GetController());
		if (soccer_ctrl != nullptr)
		{
			auto ball = GetBall();
			soccer_ctrl->SetBall(ball);
		}

#if defined(HACK_SOCCER_LLC)
		// hack hack
		auto tar_soccer_ctrl = std::dynamic_pointer_cast<cCtTargetSoccerController>(mChar->GetController());
		if (tar_soccer_ctrl != nullptr)
		{
			auto ball = GetBall();
			tar_soccer_ctrl->SetBall(ball);
		}
#endif
	}
	else
	{
		auto soccer_ctrl = std::dynamic_pointer_cast<cSoccerController>(mChar->GetController());
		if (soccer_ctrl != nullptr)
		{
			soccer_ctrl->SetBall(nullptr);
		}

#if defined(HACK_SOCCER_LLC)
		// hack hack
		auto tar_soccer_ctrl = std::dynamic_pointer_cast<cCtTargetSoccerController>(mChar->GetController());
		if (tar_soccer_ctrl != nullptr)
		{
			tar_soccer_ctrl->SetBall(nullptr);
		}
#endif
	}
}

int cScenarioExpSoccer::FindNearestBall(const tVector& pos) const
{
	int nearest_idx = gInvalidIdx;
	double min_dist = std::numeric_limits<double>::infinity();

	int num_balls = GetNumBalls();
	for (int i = 0; i < num_balls; ++i)
	{
		int curr_handle = mBallObjHandles[i];
		tVector ball_pos = GetBallPos(curr_handle);
		tVector delta = ball_pos - pos;
		delta[1] = 0;
		double dist = delta.squaredNorm();
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_idx = i;
		}
	}

	return nearest_idx;
}