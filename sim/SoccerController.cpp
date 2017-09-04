#include "sim/SoccerController.h"
#include "sim/SimCharacter.h"

#define ENABLE_TARGET_POLAR_COORDS

cSoccerController::cSoccerController()
{
}

cSoccerController::~cSoccerController()
{
}

void cSoccerController::SetBall(const std::shared_ptr<cSimObj>& ball)
{
	mBall = ball;
}

void cSoccerController::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cWaypointController::BuildNNInputOffsetScale(out_offset, out_scale);

	//const double pos_scale = 1 / 5.0;
	//const double vel_scale = 0.5 * pos_scale;
	//const double ang_vel_scale = 1 / (10.0 * M_PI);

	const double pos_scale = 1;
	const double vel_scale = 1;
	const double ang_vel_scale = 1;

	int ball_offset = GetBallStateOffset();
	out_scale[ball_offset + eBallStatePosX] = pos_scale;
	out_scale[ball_offset + eBallStatePosY] = pos_scale;
	out_scale[ball_offset + eBallStatePosZ] = pos_scale;
	out_scale[ball_offset + eBallStateVelX] = vel_scale;
	out_scale[ball_offset + eBallStateVelY] = vel_scale;
	out_scale[ball_offset + eBallStateVelZ] = vel_scale;
	out_scale[ball_offset + eBallStateAngVelX] = ang_vel_scale;
	out_scale[ball_offset + eBallStateAngVelY] = ang_vel_scale;
	out_scale[ball_offset + eBallStateAngVelZ] = ang_vel_scale;
}

void cSoccerController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cWaypointController::BuildNNInputOffsetScaleTypes(out_types);

	int tar_offset = GetTargetPosStateOffset();
	int tar_size = GetTargetPosStateSize();
	for (int i = 0; i < tar_size; ++i)
	{
		out_types[tar_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}

	int ball_offset = GetBallStateOffset();
	int ball_size = GetBallStateSize();
	for (int i = 0; i < ball_size; ++i)
	{
		out_types[ball_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

int cSoccerController::GetGroundSampleRes() const
{
	return 0;
}


int cSoccerController::GetPoliStateSize() const
{
	int state_size = cWaypointController::GetPoliStateSize();
	int ball_state_size = GetBallStateSize();
	state_size += ball_state_size;
	return state_size;
}

int cSoccerController::GetBallStateOffset() const
{
	return cWaypointController::GetPoliStateSize();
}

int cSoccerController::GetBallStateSize() const
{
	return static_cast<int>(eBallStateMax);
}

void cSoccerController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	Eigen::VectorXd ball_state;
	cWaypointController::BuildPoliState(out_state);
	BuildBallState(ball_state);

	int ball_offset = GetBallStateOffset();
	int ball_size = GetBallStateSize();
	out_state.segment(ball_offset, ball_size) = ball_state;
}

void cSoccerController::BuildBallState(Eigen::VectorXd& out_state) const
{
	assert(mBall != nullptr);
	out_state = Eigen::VectorXd::Zero(GetBallStateSize());
	
	tVector root_pos = mChar->GetRootPos();
	tMatrix origin_trans = mChar->BuildOriginTrans();
	origin_trans(1, 3) -= mGround->SampleHeight(root_pos);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	tVector ball_pos = mBall->GetPos();
	tVector ball_vel = mBall->GetLinearVelocity();
	tVector ball_ang_vel = mBall->GetAngularVelocity();

	ball_pos[3] = 1;
	ball_pos = origin_trans * ball_pos;
	ball_vel = origin_trans * ball_vel;
	ball_ang_vel = origin_trans * ball_ang_vel;
	if (flip_stance)
	{
		ball_ang_vel = -ball_ang_vel;
	}

#if defined(ENABLE_TARGET_POLAR_COORDS)
	double ball_theta = std::atan2(-ball_pos[2], ball_pos[0]);
	double ball_dist = std::sqrt(ball_pos[0] * ball_pos[0] + ball_pos[2] * ball_pos[2]);
	double ball_h = ball_pos[1];

	out_state[eBallStatePosX] = ball_theta;
	out_state[eBallStatePosY] = ball_dist;
	out_state[eBallStatePosZ] = ball_h;
#else
	out_state[eBallStatePosX] = ball_pos[0];
	out_state[eBallStatePosY] = ball_pos[1];
	out_state[eBallStatePosZ] = ball_pos[2];
#endif

	out_state[eBallStateVelX] = ball_vel[0];
	out_state[eBallStateVelY] = ball_vel[1];
	out_state[eBallStateVelZ] = ball_vel[2];
	out_state[eBallStateAngVelX] = ball_ang_vel[0];
	out_state[eBallStateAngVelY] = ball_ang_vel[1];
	out_state[eBallStateAngVelZ] = ball_ang_vel[2];
}

void cSoccerController::BuildTargetPosState(Eigen::VectorXd& out_state) const
{
#if defined(ENABLE_TARGET_POLAR_COORDS)
	//cWaypointController::BuildTargetPosState(out_state);

	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());
	tVector target_pos = mTargetPos;
	tVector ball_pos = mBall->GetPos();
	target_pos[3] = 1;
	ball_pos[3] = 1;

	tMatrix origin_trans = mChar->BuildOriginTrans();
	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	target_pos = origin_trans * target_pos;
	ball_pos = origin_trans * ball_pos;
	target_pos[1] = 0;
	target_pos[3] = 0;
	ball_pos[1] = 0;
	ball_pos[3] = 0;

	tVector ball_tar_delta = target_pos - ball_pos;
	double ball_tar_theta = std::atan2(-ball_tar_delta[2], ball_tar_delta[0]);
	double ball_tar_dist = ball_tar_delta.norm();

	out_state[0] = ball_tar_theta;
	out_state[1] = ball_tar_dist;
#else
	out_state = Eigen::VectorXd::Zero(GetTargetPosStateSize());
	tVector target_pos = mTargetPos;
	target_pos[3] = 1;
	tMatrix origin_trans = mChar->BuildOriginTrans();
	target_pos = origin_trans * target_pos;

	out_state[0] = target_pos[0];
	out_state[1] = target_pos[2];
#endif
}