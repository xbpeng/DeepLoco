#include "CtTargetController.h"
#include "SimCharacter.h"

const int gTargetPosDim = 2;

cCtTargetController::cCtTargetController() : cCtController()
{
	mTargetPos.setZero();
	mViewDist = 1;
}

cCtTargetController::~cCtTargetController()
{
}

void cCtTargetController::Init(cSimCharacter* character)
{
	cCtController::Init(character);
	InitTargetPos();
}

void cCtTargetController::SetTargetPos(const tVector& pos)
{
	mTargetPos = pos;
}

int cCtTargetController::GetPoliStateSize() const
{
	int state_size = cCtController::GetPoliStateSize();
	int tar_pos_size = GetTargetPosStateSize();
	state_size += tar_pos_size;
	return state_size;
}

void cCtTargetController::InitTargetPos()
{
	mTargetPos.setZero();
	tVector root_pos = mChar->GetRootPos();
	mTargetPos[0] = root_pos[0];
	mTargetPos[2] = root_pos[2];
}

int cCtTargetController::GetTargetPosStateOffset() const
{
	return cCtController::GetPoliStateSize();
}

int cCtTargetController::GetTargetPosStateSize() const
{
	return gTargetPosDim;
}

void cCtTargetController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd tar_pos;
	cCtController::BuildPoliState(out_state);
	BuildTargetPosState(tar_pos);

	int tar_offset = GetTargetPosStateOffset();
	int tar_size = GetTargetPosStateSize();
	out_state.segment(tar_offset, tar_size) = tar_pos;
}

void cCtTargetController::BuildTargetPosState(Eigen::VectorXd& out_state) const
{
	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());
	tVector target_pos = mTargetPos;
	target_pos[3] = 1;
	tMatrix origin_trans = mChar->BuildOriginTrans();
	target_pos = origin_trans * target_pos;
	target_pos[1] = 0;
	target_pos[3] = 0;

	if (FlipStance())
	{
		target_pos[2] = -target_pos[2];
	}

	double tar_theta = std::atan2(-target_pos[2], target_pos[0]);
	double tar_dist = target_pos.norm();

	out_state[0] = tar_theta;
	out_state[1] = tar_dist;
}