#include "CtPhaseController.h"
#include "sim/SimCharacter.h"

const int gNumPhaseBins = 4;

cCtPhaseController::cCtPhaseController() : cCtController()
{
	mCycleDur = 1;
}

cCtPhaseController::~cCtPhaseController()
{
}

void cCtPhaseController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	mPhase += time_step / GetCycleDur();
	if (mPhase >= 1)
	{
		mPhase -= static_cast<int>(mPhase);
	}
	cCtController::UpdateCalcTau(time_step, out_tau);
}

void cCtPhaseController::SetCycleDur(double dur)
{
	mCycleDur = dur;
}

double cCtPhaseController::GetCycleDur() const
{
	return mCycleDur;
}

int cCtPhaseController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize();
	int phase_size = GetPhaseStateSize();
	state_size += phase_size;
	return state_size;
}

void cCtPhaseController::SetTime(double time)
{
	mPhase = time / GetCycleDur();
	mPhase -= static_cast<int>(mPhase);
}

double cCtPhaseController::GetPhase() const
{
	return mPhase;
}

void cCtPhaseController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtController::BuildNNInputOffsetScaleTypes(out_types);

	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	for (int i = 0; i < phase_size; ++i)
	{
		out_types[phase_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

int cCtPhaseController::GetPhaseStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

int cCtPhaseController::GetPhaseStateSize() const
{
	int phase_size = 1;
#if defined(ENABLE_COS_PHASE)
	phase_size = 2;
#endif

#if defined(ENABLE_PHASE_STATE_BINS)
	phase_size += GetNumPhaseBins();
#endif
	return phase_size;
}

/*
 * If everything is coded up nicely when there ground sample size is 0 thins should not break...
 */
void cCtPhaseController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd phase_state;
	cCtController::BuildPoliState(out_state);
	BuildPhaseState(phase_state);

	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	out_state.segment(phase_offset, phase_size) = phase_state;
	/*
	int state_size = GetPoliStateSize();
	out_state.resize(state_size);

	Eigen::VectorXd phase_state;
	Eigen::VectorXd pose;
	Eigen::VectorXd ground;
	BuildPoliStateGround(ground);
	cCtController::BuildPoliState(pose);
	BuildPhaseState(phase_state);

	int ground_offset = GetPoliStateFeatureOffset(ePoliStateGround);
	int ground_size = GetPoliStateFeatureSize(ePoliStateGround);
	int pose_offset = GetPoliStateFeatureOffset(ePoliStatePose);
	int pose_size = GetPoliStateFeatureSize(ePoliStatePose);

	int phase_offset = GetPhaseStateOffset();
	int phase_size = GetPhaseStateSize();
	out_state.segment(ground_offset, ground_size) = ground;
	out_state.segment(pose_offset, pose_size) = pose;
	out_state.segment(phase_offset, phase_size) = phase_state;
	*/
}

void cCtPhaseController::BuildPhaseState(Eigen::VectorXd& out_state) const
{
	double phase = GetPhase();
	out_state = Eigen::VectorXd::Zero(GetPhaseStateSize());

#if defined(ENABLE_COS_PHASE)
	int dphase_offset = 2;
	double theta = 2 * M_PI * phase;
	out_state[0] = std::cos(theta);
	out_state[1] = std::sin(theta);
#else
	int dphase_offset = 1;
	out_state[0] = phase;
#endif

#if defined(ENABLE_PHASE_STATE_BINS)
	int bin = static_cast<int>(phase * GetNumPhaseBins());
	out_state[bin + dphase_offset] = 1;
#endif // ENABLE_PHASE_STATE_BINS
}

#if defined(ENABLE_PHASE_STATE_BINS)
int cCtPhaseController::GetNumPhaseBins() const
{
	return gNumPhaseBins;
}
#endif // ENABLE_PHASE_STATE_BINS
