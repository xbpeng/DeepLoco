#include "sim/WaypointController.h"
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"

//#define ENABLE_BILINEAR_PHASE
//#define HACK_OUTPUT_CONV_WEIGHTS

const double gMaxWaypointDist = 2;
const double gMaxStepDist0 = 2;
const double gMaxStepDist1 = 4;
const double gMaxHeading = M_PI;
const int gGroundSampleRes = 32;
const int gHeadingStateDim = 2;
const int gNumPhaseBins = 2;

cWaypointController::cWaypointController() : cCtTargetController()
{
	mViewDist = 10;
	mViewDistMin = -1;
	mCurrTimeStep = 0;
	mNewActionUpdate = false;
	mInitStepLen = 0;
	mSymmetricStep = false;
	mFlipJointOrder = nullptr;
}

cWaypointController::~cWaypointController()
{
}

void cWaypointController::Init(cSimCharacter* character)
{
	cCtTargetController::Init(character);
}

void cWaypointController::Reset()
{
	cCtTargetController::Reset();
	mLLC->Reset();
}

void cWaypointController::Clear()
{
	cCtTargetController::Clear();
	if (mLLC != nullptr)
	{
		mLLC->Clear();
	}
}

void cWaypointController::Update(double time_step)
{
	mCurrTimeStep = time_step;
	cCtTargetController::Update(time_step);
}

void cWaypointController::SetGround(std::shared_ptr<cGround> ground)
{
	cCtTargetController::SetGround(ground);
	if (mLLC != nullptr)
	{
		mLLC->SetGround(ground);
	}
}

void cWaypointController::SetInitStepLen(double step_len)
{
	mInitStepLen = step_len;
}

void cWaypointController::SetLLC(const std::shared_ptr<cBipedStepController3D>& ctrl)
{
	mLLC = ctrl;
}

const std::shared_ptr<cBipedStepController3D>& cWaypointController::GetLLC() const
{
	return mLLC;
}

int cWaypointController::GetPoliStateSize() const
{
	int size = cCtTargetController::GetPoliStateSize();
#if defined(ENABLE_BILINEAR_PHASE)
	size += GetPhaseStateSize();
#endif
	return size;
}

int cWaypointController::GetPoliActionSize() const
{
	return static_cast<int>(eActionParamMax);
}

void cWaypointController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtTargetController::BuildNNInputOffsetScaleTypes(out_types);

#if defined(ENABLE_BILINEAR_PHASE)
	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	for (int i = 0; i < phase_size; ++i)
	{
		out_types[phase_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
#endif
}

void cWaypointController::GetViewBound(tVector& out_min, tVector& out_max) const
{
	cCtTargetController::GetViewBound(out_min, out_max);

	tVector llc_min;
	tVector llc_max;
	mLLC->GetViewBound(llc_min, llc_max);
	out_min = out_min.cwiseMin(llc_min);
	out_max = out_max.cwiseMax(llc_max);
}

void cWaypointController::EnableExp(bool enable)
{
	cCtTargetController::EnableExp(enable);
	mLLC->EnableExp(enable);
}

const cBipedStepController3D::tStepPlan& cWaypointController::GetStepPlan() const
{
	return mStepPlan;
}

void cWaypointController::SetTime(double time)
{
	mLLC->SetTime(time);
	ApplyAction(mCurrAction);
}

void cWaypointController::ResetParams()
{
	cCtTargetController::ResetParams();
	mNewActionUpdate = false;
}

void cWaypointController::SetupActionBounds()
{
	int action_size = GetPoliActionSize();
	mActionBoundMin = Eigen::VectorXd::Zero(action_size);
	mActionBoundMax = Eigen::VectorXd::Zero(action_size);

	mActionBoundMin[eActionParamStepX0] = -gMaxStepDist0;
	mActionBoundMin[eActionParamStepZ0] = -gMaxStepDist0;
	mActionBoundMin[eActionParamStepX1] = -gMaxStepDist1;
	mActionBoundMin[eActionParamStepZ1] = -gMaxStepDist1;
	mActionBoundMin[eActionParamRootHeading] = -gMaxHeading;

	mActionBoundMax[eActionParamStepX0] = gMaxStepDist0;
	mActionBoundMax[eActionParamStepZ0] = gMaxStepDist0;
	mActionBoundMax[eActionParamStepX1] = gMaxStepDist1;
	mActionBoundMax[eActionParamStepZ1] = gMaxStepDist1;
	mActionBoundMax[eActionParamRootHeading] = gMaxHeading;
}

void cWaypointController::PostProcessAction(tAction& out_action) const
{
	//out_action.mParams = out_action.mParams.cwiseMax(mActionBoundMin).cwiseMin(mActionBoundMax);
}

void cWaypointController::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(action_size);
	out_scale = Eigen::VectorXd::Ones(action_size);

	out_offset[eActionParamStepX0] = -mInitStepLen;
	out_offset[eActionParamStepX1] = -2 * mInitStepLen;

	out_scale[eActionParamStepX0] = 2 / gMaxStepDist0;
	out_scale[eActionParamStepZ0] = 2 / gMaxStepDist0;
	out_scale[eActionParamStepX1] = 2 / gMaxStepDist1;
	out_scale[eActionParamStepZ1] = 2 / gMaxStepDist1;
	out_scale[eActionParamRootHeading] = 1 / gMaxHeading;
	//out_scale[eActionParamRootHeading] *= 0.5;
}

void cWaypointController::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	out_tau = Eigen::VectorXd::Zero(mChar->GetNumDof());
}

void cWaypointController::UpdateNewActionUpdate(double time_step)
{
	mNewActionUpdate = !std::isfinite(mUpdateCounter);
	auto prev_stance = mLLC->GetStance();
	auto next_stance = mLLC->PredictNextStance(time_step);
	mNewActionUpdate |= prev_stance != next_stance;
}

bool cWaypointController::NewActionUpdate() const
{
	return mNewActionUpdate;
}

void cWaypointController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	UpdateNewActionUpdate(time_step);
	mUpdateCounter = 0;

	if (NewActionUpdate())
	{
		UpdateAction();
		mFirstCycle = false;
	}

	UpdateBuildTau(time_step, out_tau);
	mLLC->UpdateCalcTau(time_step, out_tau);
}

void cWaypointController::EnableSymmetricStep(double enable)
{
	mSymmetricStep = enable;
}

void cWaypointController::DecideAction(tAction& out_action)
{
	cCtTargetController::DecideAction(out_action);
	/*
	// hack
	FILE* f = cFileUtil::OpenFile("output/terrain_map.txt", "w");
	for (int i = 0; i < mGroundSamples.size(); ++i)
	{
		double val = mGroundSamples[i];
		fprintf(f, "%.5f\t", val);
	}
	cFileUtil::CloseFile(f);
	*/

#if defined(HACK_OUTPUT_CONV_WEIGHTS)
	const auto& params = mNet->GetParams();
	//const auto& params = mCriticNet->GetParams();
	const auto& params0 = params[0];
	const auto* data = params0->cpu_data();

	FILE* f = cFileUtil::OpenFile("output/conv_filters.txt", "w");
	for (int i = 0; i < params0->count(); ++i)
	{
		double val = data[i];
		fprintf(f, "%.5f\n", val);
	}
	cFileUtil::CloseFile(f);
#endif
}

void cWaypointController::ApplyAction(const tAction& action)
{
	cCtTargetController::ApplyAction(action);
	BuildStepPlan(mStepPlan);
	mLLC->SetStepPlan(mStepPlan);
}

int cWaypointController::GetGroundSampleRes() const
{
	return gGroundSampleRes;
}

void cWaypointController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtTargetController::BuildPoliState(out_state);

#if defined(ENABLE_BILINEAR_PHASE)
	Eigen::VectorXd phase_state;
	BuildPhaseState(phase_state);

	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	out_state.segment(phase_offset, phase_size) = phase_state;
#endif
}

int cWaypointController::GetPhaseStateOffset() const
{
	return cCtTargetController::GetPoliStateSize();
}

int cWaypointController::GetPhaseStateSize() const
{
	return gNumPhaseBins;
}

void cWaypointController::BuildPhaseState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetPhaseStateSize());
	auto stance = GetStance();
	bool right_stance = (stance == cBipedStepController3D::eStanceRight);
	out_state[0] = (right_stance) ? 1 : 0;
	out_state[1] = (right_stance) ? 0 : 1;
}

void cWaypointController::BuildStepPlan(cBipedStepController3D::tStepPlan& out_plan) const
{
	const Eigen::VectorXd& params = mCurrAction.mParams;// .cwiseMax(mActionBoundMin).cwiseMin(mActionBoundMax);

	tVector step_pos0 = tVector(params[eActionParamStepX0], 0,
								params[eActionParamStepZ0], 1);
	tVector step_pos1 = tVector(params[eActionParamStepX1], 0,
								params[eActionParamStepZ1], 1);
	double heading = params[eActionParamRootHeading];
	tVector heading_dir = tVector(std::cos(heading), 0, -std::sin(heading), 0);

	tMatrix world_trans = mGroundSampleTrans;
	if (FlipStance())
	{
		world_trans.col(2) *= -1; // reflect z
	}

	step_pos0 = world_trans * step_pos0;
	step_pos0[3] = 0;
	step_pos0[1] = mGround->SampleHeight(step_pos0);

	step_pos1 = world_trans * step_pos1;
	step_pos1[3] = 0;
	step_pos1[1] = mGround->SampleHeight(step_pos1);

	heading_dir = world_trans * heading_dir;
	double world_heading = std::atan2(-heading_dir[2], heading_dir[0]);

	out_plan.mStance = GetStance();
	out_plan.mStepPos0 = step_pos0;
	out_plan.mStepPos1 = step_pos1;
	out_plan.mRootHeading = world_heading;
}

cBipedStepController3D::eStance cWaypointController::GetStance() const
{
	cBipedStepController3D::eStance stance = mLLC->PredictNextStance(mCurrTimeStep);
	return stance;
}

bool cWaypointController::EnableSymStep() const
{
	return mSymmetricStep;
}

int cWaypointController::RetargetJointID(int joint_id) const
{
	int new_joint_id = joint_id;
	if (FlipStance())
	{
		new_joint_id = (*mFlipJointOrder)[joint_id];
	}
	return new_joint_id;
}

bool cWaypointController::FlipStance() const
{
	cBipedStepController3D::eStance stance = GetStance();
	bool flip_stance = EnableSymStep();
	flip_stance &= stance == cBipedStepController3D::eStanceLeft;
	return flip_stance;
}