#include "scenarios/ScenarioExpHike.h"
#include "sim/WaypointController.h"
#include "learning/CaclaTrainer.h"

const int gNumLLCWarmupCycles = 0;

double cScenarioExpHike::CalcReward() const
{
	const double target_speed = mTargetSpeed;

	const tVector& prev_com = mPrevCOM;
	double time_elapsed = mTime - mPrevTime;
	double vel_w = 0.8;
	double step_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double heading_w = 0 * ((EnableLLCFeedbackReward()) ? 0.1 : 0);
	double LLC_reward_w = 0 * ((EnableLLCFeedbackReward()) ? 0.2 : 0);
	double fixed_w = 0.2;

	const double total_w = vel_w + step_w + heading_w + LLC_reward_w + fixed_w;
	vel_w /= total_w;
	step_w /= total_w;
	heading_w /= total_w;
	LLC_reward_w /= total_w;
	fixed_w /= total_w;

	const double vel_scale = 1.5;
	const double step_scale = 5;

	double reward = 0;

	if (time_elapsed > 0)
	{
		bool fallen = HasFallen();
		if (!fallen)
		{
			tVector curr_com = mChar->CalcCOM();
			tVector com_delta = curr_com - prev_com;
			tVector target_delta = mTargetPos - prev_com;
			com_delta[1] = 0;
			target_delta[1] = 0;
			tVector target_dir = target_delta.normalized();

			double avg_vel = target_dir.dot(com_delta);
			avg_vel /= time_elapsed;
			double vel_err = std::min(0.0, avg_vel - target_speed);
			double target_dist_threshold = GetTargetResetDist();
			if (target_delta.squaredNorm() < target_dist_threshold * target_dist_threshold)
			{
				vel_err = 0;
			}
			vel_err *= vel_err;

			eStance stance = GetStance();
			int stance_foot_id = GetStanceFootJoint(stance);
			tVector stance_foot_pos = mChar->CalcJointPos(stance_foot_id);
			const tVector& step_pos = mStepPlan.mStepPos0;
			tVector step_delta = step_pos - stance_foot_pos;
			double step_err = step_delta.squaredNorm();

			double heading = mChar->CalcHeading();
			double tar_heading = mStepPlan.mRootHeading;
			double heading_err = std::abs(tar_heading - heading);
			heading_err = std::min(2 * M_PI - heading_err, heading_err);


			double vel_reward = std::exp(-vel_scale * vel_err);
			//vel_reward = (avg_vel > 0) ? vel_reward : 0;
			double step_reward = exp(-step_scale * step_err);
			double heading_reward = 0.5 * (std::cos(heading_err) + 1);
			heading_reward = std::pow(heading_reward, 4);

			reward = vel_w * vel_reward + step_w * step_reward + heading_w * heading_reward
					+ fixed_w;
		}
	}

	return reward;
}

cScenarioExpHike::cScenarioExpHike()
{
	mExpMode = eExpModeHLC;
	mTargetSpeed = 1;
	mTargetResetDist = 1;

	mLLCTupleBufferSize = mTupleBufferSize;
	mLLCExpParams.mBaseActionRate = 0.01;
	
	mCharParams.mEnableSoftContact = true;
	mLLCTupleAcceptProb = 1;
	EnableTargetPos(true);
	EnableRandTargetPos(true);
}

cScenarioExpHike::~cScenarioExpHike()
{
}

void cScenarioExpHike::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpImitateStep::ParseArgs(parser);

	parser->ParseDouble("target_speed", mTargetSpeed);

	parser->ParseDouble("llc_exp_rate", mLLCExpParams.mRate);
	parser->ParseDouble("llc_exp_temp", mLLCExpParams.mTemp);
	parser->ParseDouble("llc_exp_base_rate", mLLCExpParams.mBaseActionRate);
	parser->ParseDouble("llc_exp_noise", mLLCExpParams.mNoise);
	parser->ParseDouble("llc_exp_noise_internal", mLLCExpParams.mNoiseInternal);

	parser->ParseDouble("llc_tuple_accept_prob", mLLCTupleAcceptProb);

	mLLCTupleBufferSize = mTupleBufferSize;
}

void cScenarioExpHike::Init()
{
	cScenarioExpImitateStep::Init();

	SetExpMode(eExpModeHLC);
	mLLCTupleBuffer.resize(mLLCTupleBufferSize);
	ResetLLCTupleBuffer();
	SetLLCExpParams(mLLCExpParams);

	if (EnabledRandStateReset())
	{
		ResetTargetPos();
	}

	InitMisc();
}

void cScenarioExpHike::Reset()
{
	cScenarioExpImitateStep::Reset();

	if (EnabledRandStateReset())
	{
		ResetTargetPos();
	}

	ResetMisc();
}

void cScenarioExpHike::Clear()
{
	cScenarioExpImitateStep::Clear();
	mLLCTupleBuffer.clear();
	ResetLLCTupleBuffer();
}

void cScenarioExpHike::SetBufferSize(int size)
{
	cScenarioExpImitateStep::SetBufferSize(size);
	mLLCTupleBufferSize = mTupleBufferSize;
}


void cScenarioExpHike::ResetLLCTupleBuffer()
{
	mLLCTupleCount = 0;
}

bool cScenarioExpHike::IsLLCTupleBufferFull() const
{
	return mLLCTupleCount >= mLLCTupleBufferSize;
}

const std::vector<tExpTuple>& cScenarioExpHike::GetLLCTuples() const
{
	return mLLCTupleBuffer;
}

const cCharController::tExpParams& cScenarioExpHike::GetLLCExpParams() const
{
	return mLLCExpParams;
}

void cScenarioExpHike::SetLLCExpParams(const cCharController::tExpParams& params)
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	mLLCExpParams = params;
	auto ctrl = GetLLC();
	if (ctrl != nullptr)
	{
		ctrl->SetExpParams(params);
	}
}

void cScenarioExpHike::EnableExplore(bool enable)
{
	cScenarioExpImitateStep::EnableExplore(enable);
	SetExpMode(mExpMode);
}

void cScenarioExpHike::SetExpMode(eExpMode mode)
{
	mExpMode = mode;

	ResetTupleBuffer();
	ResetLLCTupleBuffer();

	if (mEnableExplore)
	{
		auto hlc_ctrl = mChar->GetController();
		auto llc_ctrl = GetLLC();
		bool enable_hlc_exp = (mode == eExpModeHLC);
		bool enable_llc_exp = (mode == eExpModeLLC);

		hlc_ctrl->EnableExp(enable_hlc_exp);
		if (llc_ctrl != nullptr)
		{
			llc_ctrl->EnableExp(enable_llc_exp);
		}
	}
}

cScenarioExpHike::eExpMode cScenarioExpHike::GetExpMode() const
{
	return mExpMode;
}

const cBipedStepController3D::tStepPlan& cScenarioExpHike::GetStepPlan() const
{
	auto ctrl = std::dynamic_pointer_cast<cWaypointController>(mChar->GetController());
	if (ctrl != nullptr)
	{
		return ctrl->GetStepPlan();
	}
	return cScenarioExpImitateStep::GetStepPlan();
}

std::string cScenarioExpHike::GetName() const
{
	return "Hike Exploration";
}

void cScenarioExpHike::ResetParams()
{
	cScenarioExpImitateStep::ResetParams();
}

bool cScenarioExpHike::EnableUpdateStepPlan() const
{
	return false;
}

void cScenarioExpHike::ResetKinChar()
{
	mKinChar->Reset();
	if (EnabledRandStateReset())
	{
		const double phase_offset = 0.1;

		double dur = mCtrlParams.mCycleDur;
		double rand_phase = (mRand.FlipCoin()) ? 0 : 0.5;
		rand_phase += phase_offset;
		double rand_time = rand_phase * dur;

		mKinChar->SetTime(rand_time);
		mKinChar->Pose(rand_time);
	}
}

std::shared_ptr<cTerrainRLCharController> cScenarioExpHike::GetLLC() const
{
#if defined(HACK_SOCCER_LLC)
	return nullptr;
#endif
	auto ctrl = mChar->GetController();
	auto waypoint_ctrl = std::dynamic_pointer_cast<cWaypointController>(ctrl);
	auto LLC_ctrl = std::dynamic_pointer_cast<cTerrainRLCharController>(waypoint_ctrl->GetLLC());
	return LLC_ctrl;
}

void cScenarioExpHike::PreSubstepUpdate(double time_step)
{
	cScenarioExpImitateStep::PreSubstepUpdate(time_step);
}

void cScenarioExpHike::PostSubstepUpdate(double time_step)
{
	UpdateLLCReward(mCurrLLCTuple, time_step);
	if (NewLLCActionUpdate())
	{
		HandleNewLLCActionUpdate();
	}

	cScenarioExpImitateStep::PostSubstepUpdate(time_step);
}

bool cScenarioExpHike::NewLLCActionUpdate() const
{
#if defined(HACK_SOCCER_LLC)
	return false;
#endif
	const auto& ctrl = GetLLC();
	return ctrl->NewActionUpdate();
}

void cScenarioExpHike::HandleNewActionUpdate()
{
	cScenarioExpImitateStep::HandleNewActionUpdate();

#if defined(HACK_SOCCER_LLC)
	SyncKinChar();
#else
	auto ctrl = std::dynamic_pointer_cast<cWaypointController>(mChar->GetController());
	mStepPlan = ctrl->GetStepPlan();

	SyncKinChar();
	ApplyStepPlan(mStepPlan);
#endif
}

void cScenarioExpHike::HandleNewLLCActionUpdate()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif

	// finish recording tuple from previous cycle
	RecordLLCState(mCurrLLCTuple.mStateEnd);
	RecordLLCFlagsEnd(mCurrLLCTuple);
	UpdateLLCRewardEnd(mCurrLLCTuple);

	// do something with the tuple
	if (IsValidLLCTuple(mCurrLLCTuple))
	{
		double rand_val = mRand.RandDouble();
		if (rand_val < mLLCTupleAcceptProb)
		{
			RecordLLCTuple(mCurrLLCTuple);
		}
	}

	// start recording new tuple
	mCurrLLCTuple.mStateBeg = mCurrLLCTuple.mStateEnd;
	RecordLLCAction(mCurrLLCTuple.mAction);
	mCurrLLCTuple.mActionLogp = GetLLCActionLogp();
	ClearFlags(mCurrLLCTuple);
	RecordLLCFlagsBeg(mCurrLLCTuple);

	UpdateLLCRewardBeg(mCurrLLCTuple);
}

void cScenarioExpHike::HandleFallUpdate()
{
	cScenarioExpImitateStep::HandleFallUpdate();

	if (!NewLLCActionUpdate())
	{
		HandleNewLLCActionUpdate();
	}
}

void cScenarioExpHike::RecordLLCState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetLLC();
	ctrl->RecordPoliState(out_state);
}

void cScenarioExpHike::RecordLLCAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetLLC();
	ctrl->RecordPoliAction(out_action);
}

void cScenarioExpHike::RecordLLCFlagsBeg(tExpTuple& out_tuple) const
{
	const auto ctrl = GetLLC();
	bool off_policy = ctrl->IsOffPolicy();
	out_tuple.SetFlag(off_policy, tExpTuple::eFlagOffPolicy);
}

void cScenarioExpHike::RecordLLCFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, tExpTuple::eFlagFail);
}

void cScenarioExpHike::RecordLLCTuple(const tExpTuple& tuple)
{
	int idx = mLLCTupleCount % mLLCTupleBufferSize;
	mLLCTupleBuffer[idx] = tuple;
	++mLLCTupleCount;
}


void cScenarioExpHike::UpdateLLCRewardBeg(tExpTuple& out_tuple)
{
	out_tuple.mReward = 0;
}

void cScenarioExpHike::UpdateLLCReward(tExpTuple& out_tuple, double time_step)
{
}

void cScenarioExpHike::UpdateLLCRewardEnd(tExpTuple& out_tuple)
{
	out_tuple.mReward += CalcLLCReward();
}

double cScenarioExpHike::CalcLLCReward() const
{
	auto ctrl = std::dynamic_pointer_cast<cWaypointController>(mChar->GetController());
	const auto& step_plan = ctrl->GetStepPlan();
	double reward = CalcRewardStep(step_plan);
	return reward;
}

double cScenarioExpHike::GetLLCActionLogp() const
{
	auto ctrl = GetLLC();
	return ctrl->CalcActionLogp();
}

bool cScenarioExpHike::IsValidLLCTuple(const tExpTuple& tuple) const
{
	// cycle count is determined by hlc not llc
	bool valid = mCycleCount > GetNumLLCWarmupCycles();
	return valid;
}

int cScenarioExpHike::GetNumLLCWarmupCycles() const
{
	return gNumLLCWarmupCycles;
}

void cScenarioExpHike::EnableLLCExp(bool enable)
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	auto LLC = GetLLC();
	LLC->EnableExp(enable);
}

bool cScenarioExpHike::EnableLLCFeedbackReward() const
{
	return true;
}

tVector cScenarioExpHike::CalcTargetPosDefault()
{
	const double max_dist = GetRandTargetMaxDist();

	tVector target_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();
	target_pos[0] = root_pos[0] + mRand.RandDouble(-max_dist, max_dist);
	target_pos[2] = root_pos[2] + mRand.RandDouble(-max_dist, max_dist);

	return target_pos;
}

int cScenarioExpHike::GetTargetPosTrail3dForwardSegs() const
{
	return 10;
}