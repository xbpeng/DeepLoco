#include "sim/BipedStepController3D.h"
#include "sim/SimCharacter.h"

cBipedStepController3D::tStepPlan::tStepPlan()
{
	mStance = eStanceRight;
	mStepPos0.setZero();
	mStepPos1.setZero();
	mRootHeading = 0;
}

cBipedStepController3D::cBipedStepController3D() : cCtPDPhaseController()
{
	mViewDist = 1;
	mViewDistMin = -0.2;
}

cBipedStepController3D::~cBipedStepController3D()
{
}

void cBipedStepController3D::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtPDPhaseController::Init(character, gravity, param_file);
	InitEndEffectors();
	InitPoliState();
}

const cBipedStepController3D::tStepPlan& cBipedStepController3D::GetStepPlan() const
{
	return mStepPlan;
}

void cBipedStepController3D::SetStepPlan(const tStepPlan& plan)
{
	mStepPlan = plan;
}

void cBipedStepController3D::InitEndEffectors()
{
	mEndEffectors.clear();

	int num_joints = mChar->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		if (mChar->IsEndEffector(j))
		{
			mEndEffectors.push_back(j);
		}
	}
}

int cBipedStepController3D::GetNumEndEffectors() const
{
	return static_cast<int>(mEndEffectors.size());
}

cBipedStepController3D::eStance cBipedStepController3D::PredictNextStance(double time_step) const
{
	double phase = mPhase + time_step / GetCycleDur();
	phase = std::fmod(phase, 1.0);
	return GetStance(phase);
}

void cBipedStepController3D::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cCtPDPhaseController::BuildNNInputOffsetScaleTypes(out_types);

	int contact_offset = GetContactStateOffset();
	int contact_size = GetContactStateSize();
	for (int i = 0; i < contact_size; ++i)
	{
		out_types[contact_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}

	int task_offset = GetTaskStateOffset();
	int task_size = GetTaskStateSize();
	for (int i = 0; i < task_size; ++i)
	{
		out_types[task_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

cBipedStepController3D::eStance cBipedStepController3D::GetStance() const
{
	return GetStance(mPhase);
}

cBipedStepController3D::eStance cBipedStepController3D::GetStance(double phase) const
{
	eStance stance = (phase < 0.5) ? eStanceRight : eStanceLeft;
	return stance;
}

int cBipedStepController3D::GetPoliStateSize() const
{
	int state_size = cCtPDPhaseController::GetPoliStateSize();
	state_size += GetContactStateSize();
	state_size += GetTaskStateSize();
	return state_size;
}

int cBipedStepController3D::GetContactStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize();
}

int cBipedStepController3D::GetContactStateSize() const
{
	return GetNumEndEffectors();
}

int cBipedStepController3D::GetTaskStateOffset() const
{
	return cCtPDPhaseController::GetPoliStateSize() + GetContactStateSize();
}

int cBipedStepController3D::GetTaskStateSize() const
{
	return eTaskParamMax;
}

void cBipedStepController3D::BuildPoliState(Eigen::VectorXd& out_state) const
{
	cCtPDPhaseController::BuildPoliState(out_state);

	Eigen::VectorXd contact_state;
	Eigen::VectorXd task_state;
	BuildContactState(contact_state);
	BuildTaskState(task_state);

	int contact_offset = GetContactStateOffset();
	int contact_size = GetContactStateSize();
	int task_offset = GetTaskStateOffset();
	int task_size = GetTaskStateSize();

	out_state.segment(contact_offset, contact_size) = contact_state;
	out_state.segment(task_offset, task_size) = task_state;
}

void cBipedStepController3D::BuildContactState(Eigen::VectorXd& out_state) const
{
	int num_end_effectors = GetNumEndEffectors();
	out_state.resize(num_end_effectors);
	for (int e = 0; e < num_end_effectors; ++e)
	{
		int joint_id = mEndEffectors[e];
		joint_id = RetargetJointID(joint_id);
		bool in_contact = mChar->IsInContact(joint_id);
		double val = (in_contact) ? 1 : 0;
		out_state[e] = val;
	}
}

void cBipedStepController3D::BuildTaskState(Eigen::VectorXd& out_state) const
{
	out_state = tTaskParams::Zero();
	double tar_heading = mStepPlan.mRootHeading;
	tVector heading_dir = tVector(std::cos(tar_heading), 0, -std::sin(tar_heading), 0);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix heading_trans = mChar->BuildOriginTrans();
	heading_dir = heading_trans * heading_dir;
	tar_heading = std::atan2(-heading_dir[2], heading_dir[0]);

 	eStance curr_stance = GetStance();
	//assert(curr_stance == mStepPlan.mStance);

	bool right_stance = mStepPlan.mStance == eStanceRight;
	int swing_id = (right_stance) ? mEndEffectors[eStanceLeft] : mEndEffectors[eStanceRight];
	tVector swing_pos = mChar->CalcJointPos(swing_id);
	tVector pos0 = mStepPlan.mStepPos0;
	tVector delta0 = pos0 - swing_pos;
	delta0 = heading_trans * delta0;
	delta0[1] = pos0[1] - ground_h;

	int stance_id = (right_stance) ? mEndEffectors[eStanceRight] : mEndEffectors[eStanceLeft];
	tVector stance_pos = mChar->CalcJointPos(stance_id);
	tVector pos1 = mStepPlan.mStepPos1;
	tVector delta1 = pos1 - stance_pos;
	delta1 = heading_trans * delta1;
	delta1[1] = pos1[1] - ground_h;

	tVector right_delta0;
	tVector left_delta0;
	tVector right_delta1;
	tVector left_delta1;

	if (right_stance)
	{
		right_delta0 = tVector::Zero();
		left_delta0 = delta0;
		right_delta1 = delta1;
		left_delta1 = tVector::Zero();
	}
	else
	{
		right_delta0 = delta0;
		left_delta0 = tVector::Zero();
		right_delta1 = tVector::Zero();
		left_delta1 = delta1;
	}

	out_state[eTaskParamStepRightX0] = right_delta0[0];
	out_state[eTaskParamStepRightY0] = right_delta0[1];
	out_state[eTaskParamStepRightZ0] = right_delta0[2];
	out_state[eTaskParamStepLeftX0] = left_delta0[0];
	out_state[eTaskParamStepLeftY0] = left_delta0[1];
	out_state[eTaskParamStepLeftZ0] = left_delta0[2];
	out_state[eTaskParamStepRightX1] = right_delta1[0];
	out_state[eTaskParamStepRightY1] = right_delta1[1];
	out_state[eTaskParamStepRightZ1] = right_delta1[2];
	out_state[eTaskParamStepLeftX1] = left_delta1[0];
	out_state[eTaskParamStepLeftY1] = left_delta1[1];
	out_state[eTaskParamStepLeftZ1] = left_delta1[2];
	out_state[eTaskParamRootHeading] = tar_heading;
}

// hack to get around pdphase controller being 2d, terrain should be compositional not inherited
int cBipedStepController3D::GetNumGroundSamples() const
{
    return cCtController::GetNumGroundSamples();
}

tVector cBipedStepController3D::CalcGroundSamplePos(int s) const
{
    return cCtController::CalcGroundSamplePos(s);
}

void cBipedStepController3D::GetViewBound(tVector& out_min, tVector& out_max) const
{
    cCtController::GetViewBound(out_min, out_max);
}

void cBipedStepController3D::EvalNet(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const
{
	cCtPDPhaseController::EvalNet(x, out_y);
}