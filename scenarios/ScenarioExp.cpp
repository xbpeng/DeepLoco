#include "scenarios/ScenarioExp.h"

#include <memory>
#include <ctime>
#include "sim/GroundPlane.h"
#include "sim/GroundVar2D.h"

const int gNumWarmupCycles = 1;

cScenarioExp::cScenarioExp()
{
	mTupleBufferSize = 32;
	ResetTupleBuffer();
	mCurriculumPhase = 0;

	mEnableFallReset = true;
	mEnableExplore = true;
	mExpParams.mBaseActionRate = 0.01;

	mEnableRandEpisodeTimeLim = false;
	mEpisodeTimeLimMin = std::numeric_limits<double>::infinity();
	mEpisodeTimeLimMax = std::numeric_limits<double>::infinity();
	mEpisodeTimeLimType = eTimeLimUniform;
	mEpisodeTimeLimExpLambda = 10;
	mEpisodeMaxTime = std::numeric_limits<double>::infinity();
}

cScenarioExp::~cScenarioExp()
{
}

void cScenarioExp::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	
	parser->ParseInt("tuple_buffer_size", mTupleBufferSize);
	parser->ParseDouble("exp_rate", mExpParams.mRate);
	parser->ParseDouble("exp_temp", mExpParams.mTemp);
	parser->ParseDouble("exp_base_rate", mExpParams.mBaseActionRate);
	parser->ParseDouble("exp_noise", mExpParams.mNoise);
	parser->ParseDouble("exp_noise_internal", mExpParams.mNoiseInternal);

	parser->ParseBool("enable_fall_reset", mEnableFallReset);
	parser->ParseBool("enable_rand_time_limit", mEnableRandEpisodeTimeLim);
	parser->ParseDouble("episode_max_time", mEpisodeMaxTime);

	std::string time_lim_str = "";
	parser->ParseString("episode_time_lim_type", time_lim_str);
	ParseTimeLimType(time_lim_str, mEpisodeTimeLimType);
	parser->ParseDouble("episode_time_lim_min", mEpisodeTimeLimMin);
	parser->ParseDouble("episode_time_lim_max", mEpisodeTimeLimMax);
	parser->ParseDouble("episode_time_lim_exp_lambda", mEpisodeTimeLimExpLambda);

	ParseMiscArgs(parser);
}

void cScenarioExp::Init()
{
	cScenarioSimChar::Init();

	ResetParams();
	mCurriculumPhase = 0;
	mPrevCOM = mChar->CalcCOM();
	
	mTupleBuffer.resize(mTupleBufferSize);
	ResetTupleBuffer();
	
	EnableExplore(mEnableExplore);
	SetExpParams(mExpParams);

	if (EnableRandInitAction())
	{
		// start off with random action to get more diverse initial states
		CommandRandAction();
	}

	if (EnableRandTimeLim())
	{
		SetRandTimeLim();
	}

	InitMisc();
}

void cScenarioExp::Reset()
{
	cScenarioSimChar::Reset();

	ResetParams();
	mPrevCOM = mChar->CalcCOM();

	if (EnableRandInitAction())
	{
		// start off with random action to get more diverse initial states
		CommandRandAction();
	}

	if (EnableRandTimeLim())
	{
		SetRandTimeLim();
	}

	ResetMisc();
}

void cScenarioExp::Clear()
{
	cScenarioSimChar::Clear();
	ResetParams();
	mCurriculumPhase = 0;
	mTupleBuffer.clear();
	ResetTupleBuffer();

	ClearMisc();
}

void cScenarioExp::Update(double time_elapsed)
{
	cScenarioSimChar::Update(time_elapsed);
	UpdateMisc(time_elapsed);

	if (time_elapsed > 0)
	{
		if (HasFallen())
		{
			HandleFallUpdate();
		}

		if (EndEpisode())
		{
			HandleEpisodeEnd();
			Reset();
		}
	}
}

void cScenarioExp::SetBufferSize(int size)
{
	mTupleBufferSize = size;
}

void cScenarioExp::ResetTupleBuffer()
{
	mTupleCount = 0;
}

bool cScenarioExp::IsTupleBufferFull() const
{
	return mTupleCount >= mTupleBufferSize;
}

const std::vector<tExpTuple>& cScenarioExp::GetTuples() const
{
	return mTupleBuffer;
}

std::string cScenarioExp::GetName() const
{
	return "Exploration";
}

void cScenarioExp::EnableExplore(bool enable)
{
	mEnableExplore = enable;
	auto ctrl = mChar->GetController();
	ctrl->EnableExp(mEnableExplore);
}

const cCharController::tExpParams& cScenarioExp::GetExpParams() const
{
	return mExpParams;
}

void cScenarioExp::SetExpParams(const cCharController::tExpParams& params)
{
	mExpParams = params;
	auto ctrl = mChar->GetController();
	if (ctrl != nullptr)
	{
		ctrl->SetExpParams(params);
	}
}

void cScenarioExp::SetCurriculumPhase(double phase)
{
	mCurriculumPhase = phase;
	double terrain_lerp = cMathUtil::Clamp(phase, 0.0, 1.0);
	SetGroundParamBlend(terrain_lerp);
}

double cScenarioExp::GetEpisodeMaxTime() const
{
	return mEpisodeMaxTime;
}

void cScenarioExp::ResetParams()
{
	mPrevCOM.setZero();
	mPrevTime = mTime;
	mCycleCount = 0;

	ResetMiscParams();
}


bool cScenarioExp::NewActionUpdate() const
{
	bool new_action = false;
	const auto& ctrl = mChar->GetController();
	if (ctrl != nullptr)
	{
		new_action = ctrl->NewActionUpdate();
	}
	return new_action;
}

void cScenarioExp::PostSubstepUpdate(double time_step)
{
	UpdateRewardSubstep(time_step, mCurrTuple);
	PostSubstepUpdateMisc(time_step);

	bool new_cycle = NewActionUpdate();
	if (new_cycle)
	{
		HandleNewActionUpdate();
	}
}

void cScenarioExp::HandleNewActionUpdate()
{
	// finish recording tuple from previous cycle
	RecordState(mCurrTuple.mStateEnd);
	RecordFlagsEnd(mCurrTuple);
	UpdateRewardEnd(mCurrTuple);

	// do something with the tuple
	if (IsValidTuple(mCurrTuple))
	{
		RecordTuple(mCurrTuple);
	}

	UpdateMiscRecord();
	IncCycleCount();

	// start recording new tuple
	mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
	RecordAction(mCurrTuple.mAction);
	mCurrTuple.mActionLogp = GetActionLogp();
	ClearFlags(mCurrTuple);
	RecordFlagsBeg(mCurrTuple);

	ClearReward(mCurrTuple);
	UpdateRewardBeg(mCurrTuple);

	mPrevCOM = mChar->CalcCOM();
	mPrevTime = mTime;
}

void cScenarioExp::HandleFallUpdate()
{
	if (!NewActionUpdate()) // a little weird but that's how it works
	{
		HandleNewActionUpdate();
	}
}

void cScenarioExp::IncCycleCount()
{
	++mCycleCount;
}

void cScenarioExp::ClearReward(tExpTuple& out_tuple) const
{
	out_tuple.mReward = 0;
}

void cScenarioExp::UpdateRewardBeg(tExpTuple& out_tuple) const
{
}

void cScenarioExp::UpdateRewardSubstep(double time_step, tExpTuple& out_tuple) const
{
}

void cScenarioExp::UpdateRewardEnd(tExpTuple& out_tuple) const
{
	out_tuple.mReward += CalcReward();
}

std::shared_ptr<const cNNController> cScenarioExp::GetNNController() const
{
	return std::dynamic_pointer_cast<const cNNController>(mChar->GetController());
}

std::shared_ptr<cNNController> cScenarioExp::GetNNController()
{
	return std::dynamic_pointer_cast<cNNController>(mChar->GetController());
}

void cScenarioExp::SetEpisodeTimeLimType(eTimeLimType lim_type)
{
	mEpisodeTimeLimType = lim_type;
}

void cScenarioExp::SetEpisodeTimeLim(double lim_min, double lim_max)
{
	mEpisodeTimeLimMin = lim_min;
	mEpisodeTimeLimMax = lim_max;
}

void cScenarioExp::GetEpisodeTimeLim(double& out_min, double& out_max) const
{
	out_min = mEpisodeTimeLimMin;
	out_max = mEpisodeTimeLimMax;
}

void cScenarioExp::SetEpisodeTimeLimExpLambda(double lambda)
{
	mEpisodeTimeLimExpLambda = lambda;
}

double cScenarioExp::GetEpisodeTimeLimExpLambda(double lambda) const
{
	return mEpisodeTimeLimExpLambda;
}

void cScenarioExp::SetPoliModelFile(const std::string& model_file)
{
	mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = model_file;
}

void cScenarioExp::SetCriticModelFile(const std::string& model_file)
{
	mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel] = model_file;
}

std::vector<std::string>& cScenarioExp::GetNetFiles()
{
	return mCtrlParams.mNetFiles;
}

bool cScenarioExp::EnableEvalRecord() const
{
	return false;
}

void cScenarioExp::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetNNController();
	ctrl->RecordPoliState(out_state);
}

void cScenarioExp::RecordAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetNNController();
	ctrl->RecordPoliAction(out_action);
}

double cScenarioExp::GetActionLogp() const
{
	auto ctrl = GetNNController();
	double logp = 0;
	if (ctrl != nullptr)
	{
		logp = ctrl->CalcActionLogp();
	}
	return logp;
}

bool cScenarioExp::CheckFail() const
{
	bool fail = HasFallen();
	return fail;
}


void cScenarioExp::ClearFlags(tExpTuple& out_tuple) const
{
	out_tuple.ClearFlags();
}

void cScenarioExp::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	bool first_cycle = IsFirstValidCycle();
	out_tuple.SetFlag(first_cycle, tExpTuple::eFlagStart);
}

void cScenarioExp::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, tExpTuple::eFlagFail);
}

void cScenarioExp::RecordTuple(const tExpTuple& tuple)
{
	int idx = mTupleCount % mTupleBufferSize;
	mTupleBuffer[idx] = tuple;
	++mTupleCount;
}

bool cScenarioExp::EnableRandInitAction() const
{
	return true;
}

void cScenarioExp::CommandRandAction()
{
	const auto& ctrl = mChar->GetController();
	ctrl->CommandRandAction();
}

bool cScenarioExp::IsFirstValidCycle() const
{
	return mCycleCount == GetNumWarmupCycles() + 1;
}

bool cScenarioExp::IsValidTuple(const tExpTuple& tuple) const
{
	bool valid = mCycleCount > GetNumWarmupCycles();
	return valid;
}

void cScenarioExp::SetRandTimeLim()
{
	switch (mEpisodeTimeLimType)
	{
	case eTimeLimUniform:
		mEpisodeMaxTime = mRand.RandDouble(mEpisodeTimeLimMin, mEpisodeTimeLimMax);
		break;
	case eTimeLimExp:
		mEpisodeMaxTime = mRand.RandDoubleExp(1 / mEpisodeTimeLimExpLambda);
		break;
	default:
		assert(false); // unsupported episode time limit type
		break;
	}

#if defined (ENABLE_DEBUG_PRINT)
	printf("Rand Episode Max Time: %.5f\n", mEpisodeMaxTime);
#endif
}

void cScenarioExp::ParseTimeLimType(const std::string& str, eTimeLimType& out_type) const
{
	if (str == "")
	{
	}
	else if (str == "uniform")
	{
		out_type = eTimeLimUniform;
	}
	else if (str == "exp")
	{
		out_type = eTimeLimExp;
	}
	else
	{
		printf("Unsupported episode time limite type: %s\n", str.c_str());
		assert(false);
	}
}

bool cScenarioExp::IsValidCycle() const
{
	bool valid = mCycleCount > GetNumWarmupCycles();
	return valid;
}

double cScenarioExp::CalcReward() const
{
	auto ctrl = GetNNController();
	double reward = ctrl->CalcReward();
	return reward;
}

int cScenarioExp::GetNumWarmupCycles() const
{
	return gNumWarmupCycles;
}

bool cScenarioExp::EndEpisode() const
{
	bool fallen = HasFallen();
	bool over_time = CheckEpisodeTimeLimit();
	return (mEnableFallReset && fallen) || over_time;
}

bool cScenarioExp::EnableRandTimeLim() const
{
	return mEnableRandEpisodeTimeLim;
}

bool cScenarioExp::CheckEpisodeTimeLimit() const
{
	double max_time = GetEpisodeMaxTime();
	bool is_end = (mTime >= max_time);
	return is_end;
}

void cScenarioExp::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
}

void cScenarioExp::InitMisc()
{
}

void cScenarioExp::ClearMisc()
{
}

void cScenarioExp::ResetMisc()
{
}

void cScenarioExp::ResetMiscParams()
{
}

void cScenarioExp::UpdateMisc(double time_step)
{
}

void cScenarioExp::PostSubstepUpdateMisc(double time_step)
{
}

void cScenarioExp::HandleEpisodeEnd()
{
}

void cScenarioExp::UpdateMiscRecord()
{
}