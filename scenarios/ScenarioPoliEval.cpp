#include "ScenarioPoliEval.h"
#include "ScenarioExp.h"
#include "sim/NNController.h"
#include "sim/GroundVar2D.h"
#include "sim/GroundTrail3D.h"
#include "util/FileUtil.h"

cScenarioPoliEval::cScenarioPoliEval()
{
	ResetParams();
	mAvgDist = 0;
	mEpisodeCount = 0;
	mTotalCycles = 0;
	mEnableExplore = false;

	// analysis stuff
	mRecordNNActivation = false;
	mNNActivationOutputFile = "";
	mNNActivationLayer = "";

	mRecordActions = false;;
	mActionOutputFile = "";

	mRecordCtrlForce = false;
	mCtrlForceOutputFile = "";

	mRecordActionIDState = false;
	mActionIDStateOutputFile = "";

	mRecordReward = true;
}

cScenarioPoliEval::~cScenarioPoliEval()
{
}

double cScenarioPoliEval::GetAvgDist() const
{
	return mAvgDist;
}

double cScenarioPoliEval::CalcAvgReward() const
{
	const auto& reward_log = GetRewardLog();
	size_t num_reward = reward_log.size();
	double avg_reward = 0;
	for (size_t i = 0; i < num_reward; ++i)
	{
		avg_reward += reward_log[i] / num_reward;
	}
	return avg_reward;
}

void cScenarioPoliEval::ResetAvgDist()
{
	mAvgDist = 0;
	mEpisodeCount = 0;
}

int cScenarioPoliEval::GetNumEpisodes() const
{
	return mEpisodeCount;
}

int cScenarioPoliEval::GetNumTotalCycles() const
{
	return mTotalCycles;
}

const std::vector<double>& cScenarioPoliEval::GetDistLog() const
{
	return mDistLog;
}

const std::vector<double>& cScenarioPoliEval::GetRewardLog() const
{
	return mRewardLog;
}

void cScenarioPoliEval::SetPoliModelFile(const std::string& model_file)
{
	mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel] = model_file;
}

void cScenarioPoliEval::SetCriticModelFile(const std::string& model_file)
{
	mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel] = model_file;
}

std::string cScenarioPoliEval::GetName() const
{
	return "Policy Evaluation";
}

void cScenarioPoliEval::RecordDistTraveled()
{
	double dist = CalcDistTraveled();

	mAvgDist = cMathUtil::AddAverage(mAvgDist, mEpisodeCount, dist, 1);
	mDistLog.push_back(dist);

#if defined (ENABLE_DEBUG_PRINT)
	printf("\nEpisodes: %i\n", mEpisodeCount);
	printf("Avg dist: %.5f\n", mAvgDist);
#endif
}

double cScenarioPoliEval::CalcDistTraveled() const
{
	double dist = 0;

	switch (mGround->GetGroundClass())
	{
	case cGround::eClassTrail3D:
		dist = CalcDistTraveledTrail3D();
		break;
	default:
		dist = CalcDistTraveledDefault();
		break;
	}

	return dist;
}

double cScenarioPoliEval::CalcDistTraveledDefault() const
{
	tVector curr_pos = mChar->GetRootPos();
	tVector delta = curr_pos - mPosStart;
	delta[1] = 0;
	double dist = delta.norm();
	return dist;
}

double cScenarioPoliEval::CalcDistTraveledTrail3D() const
{
	tVector curr_pos = mChar->GetRootPos();
	auto trail = std::dynamic_pointer_cast<cGroundTrail3D>(mGround);
	double dist = trail->CalcTrailDist(mPosStart, curr_pos);
	return dist;
}

void cScenarioPoliEval::ResetRecord()
{
	mAvgDist = 0;
	mEpisodeCount = 0;
	mCycleCount = 0;
	mTotalCycles = 0;
	mDistLog.clear();
	mRewardLog.clear();
	mTotalReward = 0;
}

bool cScenarioPoliEval::EnableRandInitAction() const
{
	return false;
}

bool cScenarioPoliEval::IsValidCycle() const
{
	bool valid = mCycleCount >= GetNumWarmupCycles();
	return valid;
}

void cScenarioPoliEval::EndEpisodeRecord()
{
	if (EnableRecordReward() && !NewActionUpdate())
	{
		UpdateTotalReward(mCurrTuple.mReward);
	}

	if (IsValidCycle())
	{
		RecordDistTraveled();
		++mEpisodeCount;
		if (EnableRecordReward())
		{
			RecordTotalReward();
		}
	}
}

double cScenarioPoliEval::GetCurrCumulativeReward() const
{
	return mTotalReward;
}

double cScenarioPoliEval::CalcAvgCumulativeReward() const
{
	double avg_reward = 0;
	int num_entries = static_cast<int>(mRewardLog.size());
	for (int i = 0; i < num_entries; ++i)
	{
		double r = mRewardLog[i];
		avg_reward += r / num_entries;
	}
	return avg_reward;
}

void cScenarioPoliEval::InitNNActivation(const std::string& out_file)
{
	cFileUtil::ClearFile(out_file);
}

bool cScenarioPoliEval::EnableRecordNNActivation() const
{
	return mRecordNNActivation && mNNActivationLayer != "" && mNNActivationOutputFile != "";
}

void cScenarioPoliEval::RecordNNActivation(const std::string& layer_name, const std::string& out_file)
{
	const auto& nn_ctrl = GetNNController();
	const auto& net = nn_ctrl->GetNet();

	Eigen::VectorXd data;
	net->GetLayerState(layer_name, data);

	int data_size = static_cast<int>(data.size());
	if (data_size > 0)
	{
		std::string data_str = "";
		int action_id = nn_ctrl->GetCurrActionID();
		data_str += std::to_string(action_id);

		for (int i = 0; i < data_size; ++i)
		{
			data_str += ",\t";
			data_str += std::to_string(data[i]);
		}
		data_str += "\n";

		cFileUtil::AppendText(data_str, out_file);
	}
}

void cScenarioPoliEval::InitActionRecord(const std::string& out_file) const
{
	FILE* file = cFileUtil::OpenFile(out_file, "w");
	const auto& ctrl = mChar->GetController();

	int num_actions = ctrl->GetNumActions();
	Eigen::VectorXd params;
	for (int a = 0; a < num_actions; ++a)
	{
		ctrl->BuildActionOptParams(a, params);
		fprintf(file, "%i", a);

		int param_size = static_cast<int>(params.size());
		for (int i = 0; i < param_size; ++i)
		{
			fprintf(file, ", %.5f", params[i]);
		}
		fprintf(file, "\n");
	}

	cFileUtil::CloseFile(file);
}

void cScenarioPoliEval::EnableRecordActions(bool enable)
{
	if (enable)
	{
		if (!EnableRecordActions() && mActionOutputFile != "")
		{
			InitActionRecord(mActionOutputFile);
		}
	}
	mRecordActions = enable;
}


void cScenarioPoliEval::InitCtrlForceRecord(const std::string& out_file) const
{
	cFileUtil::ClearFile(out_file);
}


void cScenarioPoliEval::RecordCtrlForce(const std::string& out_file)
{
	auto ctrl = std::static_pointer_cast<cTerrainRLCharController>(mChar->GetController());
	std::string data_str = "";
	const Eigen::VectorXd& tau = ctrl->GetTau();

	int data_size = static_cast<int>(tau.size());
	for (int i = 0; i < data_size; ++i)
	{
		if (i > 0)
		{
			data_str += ",\t";
		}
		data_str += std::to_string(tau[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

bool cScenarioPoliEval::EnableRecordActions() const
{
	return mRecordActions && mActionOutputFile != "";
}

void cScenarioPoliEval::EnableRecordCtrlForce(bool enable)
{
	if (enable)
	{
		if (!EnableRecordCtrlForce() && mCtrlForceOutputFile != "")
		{
			InitCtrlForceRecord(mCtrlForceOutputFile);
		}
	}
	mRecordCtrlForce = enable;
}

bool cScenarioPoliEval::EnableRecordCtrlForce() const
{
	return mRecordCtrlForce && mCtrlForceOutputFile != "";
}

void cScenarioPoliEval::RecordAction(const std::string& out_file)
{
	auto ctrl = std::static_pointer_cast<cTerrainRLCharController>(mChar->GetController());
	
	std::string data_str = "";
	int action_id = ctrl->GetCurrActionID();
	data_str += std::to_string(action_id);

	Eigen::VectorXd action;
	ctrl->RecordPoliAction(action);

	int data_size = static_cast<int>(action.size());
	for (int i = 0; i < data_size; ++i)
	{
		data_str += ",\t";
		data_str += std::to_string(action[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

void cScenarioPoliEval::InitActionIDState(const std::string& out_file) const
{
	cFileUtil::ClearFile(out_file);
}

bool cScenarioPoliEval::EnableRecordActionIDState() const
{
	return mRecordActionIDState && mActionIDStateOutputFile != "";
}

void cScenarioPoliEval::RecordActionIDState(const std::string& out_file)
{
	const auto& ctrl = mChar->GetController();
	auto nn_ctrl = GetNNController();

	Eigen::VectorXd state;
	nn_ctrl->RecordPoliState(state);

	std::string data_str = "";
	int action_id = ctrl->GetCurrActionID();
	data_str += std::to_string(action_id);

	for (int i = 0; i < static_cast<int>(state.size()); ++i)
	{
		data_str += ",\t";
		data_str += std::to_string(state[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

bool cScenarioPoliEval::EnableRecordReward()
{
	return mRecordReward;
}

void cScenarioPoliEval::UpdateTotalReward(double curr_reward)
{
	mTotalReward += curr_reward;
}

void cScenarioPoliEval::RecordTotalReward()
{
	mRewardLog.push_back(mTotalReward);
}

void cScenarioPoliEval::ParseMiscArgs(const std::shared_ptr<cArgParser>& parser)
{
	parser->ParseBool("record_nn_activation", mRecordNNActivation);
	parser->ParseString("nn_activation_output_file", mNNActivationOutputFile);
	parser->ParseString("nn_activation_layer", mNNActivationLayer);

	parser->ParseBool("record_actions", mRecordActions);
	parser->ParseString("action_output_file", mActionOutputFile);

	parser->ParseBool("record_ctrl_force", mRecordCtrlForce);
	parser->ParseString("ctrl_force_output_file", mCtrlForceOutputFile);

	parser->ParseBool("record_action_id_state", mRecordActionIDState);
	parser->ParseString("action_id_state_output_file", mActionIDStateOutputFile);

	parser->ParseBool("record_reward", mRecordReward);
}

void cScenarioPoliEval::InitMisc()
{
	ResetRecord();
	mPosStart = mChar->GetRootPos();

	if (EnableRecordNNActivation())
	{
		InitNNActivation(mNNActivationOutputFile);
	}

	if (EnableRecordActions())
	{
		InitActionRecord(mActionOutputFile);
	}

	if (EnableRecordActionIDState())
	{
		InitActionIDState(mActionIDStateOutputFile);
	}
}

void cScenarioPoliEval::ClearMisc()
{
	ResetRecord();
}

void cScenarioPoliEval::ResetMisc()
{
	mPosStart = mChar->GetRootPos();
}

void cScenarioPoliEval::ResetMiscParams()
{
	mPosStart.setZero();
	mTotalReward = 0;
}

void cScenarioPoliEval::PostSubstepUpdateMisc(double time_step)
{
	if (EnableRecordCtrlForce())
	{
		RecordCtrlForce(mCtrlForceOutputFile);
	}
}

void cScenarioPoliEval::HandleEpisodeEnd()
{
	EndEpisodeRecord();
}

void cScenarioPoliEval::UpdateMiscRecord()
{
	UpdateTotalReward(mCurrTuple.mReward);

	if (IsValidCycle())
	{
		if (EnableRecordNNActivation())
		{
			RecordNNActivation(mNNActivationLayer, mNNActivationOutputFile);
		}

		if (EnableRecordActions())
		{
			RecordAction(mActionOutputFile);
		}

		if (EnableRecordActionIDState())
		{
			RecordActionIDState(mActionIDStateOutputFile);
		}
		++mTotalCycles;
	}
}