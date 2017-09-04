#include "CharController.h"
#include "SimCharacter.h"

cCharController::tExpParams::tExpParams()
{
	mRate = 0.2;
	mTemp = 1;
	mBaseActionRate = 0;
	mNoise = 0.2;
	mNoiseInternal = 1;
}

cCharController::cCharController()
{
	mState = 0;
	mPhase = 0;

	mEnableExp = false;
}

cCharController::~cCharController()
{
}

void cCharController::Reset()
{
	cController::Reset();
	TransitionState(0, 0);
}

void cCharController::Update(double time_step)
{
	cController::Update(time_step);
}

int cCharController::GetState() const
{
	return mState;
}

double cCharController::GetPhase() const
{
	return mPhase;
}

void cCharController::SetPhase(double phase)
{
	mPhase = phase;
}

int cCharController::GetNumStates() const
{
	return 1;
}

double cCharController::CalcNormPhase() const
{
	int state = GetState();
	double phase = GetPhase();
	phase = cMathUtil::Clamp(phase, 0.0, 1.0);

	int num_states = GetNumStates();
	double norm_phase = (state + phase) / num_states;
	norm_phase = cMathUtil::Clamp(norm_phase, 0.0, 1.0);
	return norm_phase;
}

void cCharController::TransitionState(int state)
{
	TransitionState(state, 0);
}

void cCharController::TransitionState(int state, double phase)
{
	mState = state;
	mPhase = phase;
}

bool cCharController::IsNewCycle() const
{
	return mState == 0 && mPhase == 0;
}

bool cCharController::NewActionUpdate() const
{
	return IsNewCycle();
}

void cCharController::CommandAction(int action_id)
{
}

void cCharController::CommandRandAction()
{
}

int cCharController::GetDefaultAction() const
{
	return gInvalidIdx;
}

void cCharController::SetDefaultAction(int action_id)
{
}

int cCharController::GetNumActions() const
{
	return 0;
}

int cCharController::GetCurrActionID() const
{
	return 0;
}

void cCharController::EnableExp(bool enable)
{
	mEnableExp = enable;
}

bool cCharController::EnabledExplore() const
{
	return mEnableExp;
}

const cCharController::tExpParams& cCharController::GetExpParams() const
{
	return mExpParams;
}

void cCharController::SetExpParams(const tExpParams& params)
{
	mExpParams = params;
}

double cCharController::GetViewDist() const
{
	return mViewDist;
}

void cCharController::SetViewDist(double dist)
{
	mViewDist = dist;
}

void cCharController::GetViewBound(tVector& out_min, tVector& out_max) const
{
	tVector root_pos = mChar->GetRootPos();
	root_pos[1] = 0;
	out_min = root_pos + tVector(-mViewDist, 0, -mViewDist, 0);
	out_max = root_pos + tVector(mViewDist, 0, mViewDist, 0);
}

void cCharController::BuildNormPose(Eigen::VectorXd& pose) const
{
	if (mChar != nullptr)
	{
		pose = mChar->GetPose();
	}
}

void cCharController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
}

void cCharController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
}

void cCharController::SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params)
{
}

void cCharController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
}

int cCharController::GetNumGroundSamples() const
{
	return 0;
}

tVector cCharController::GetGroundSample(int s) const
{
	return tVector::Zero();
}

tMatrix cCharController::GetGroundSampleTrans() const
{
	return tMatrix::Identity();
}

void cCharController::getInternalState(Eigen::VectorXd& state) const
{

}

void cCharController::updateInternalState(Eigen::VectorXd& state)
{

}

void cCharController::HandlePoseReset()
{
}

void cCharController::HandleVelReset()
{
}

std::string cCharController::BuildTextInfoStr() const
{
	return "";
}

#if defined(ENABLE_DEBUG_VISUALIZATION)
void cCharController::GetVisCharacterFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisTerrainFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisActionValues(Eigen::VectorXd& out_vals) const
{
	out_vals = Eigen::VectorXd();
}
#endif
