#include "ScenarioTrackMotion.h"

#include <memory>
#include <ctime>
#include "util/FileUtil.h"

cScenarioTrackMotion::cScenarioTrackMotion()
{
	mMotionFile = "";
	mTargetCtrlID = 0;
	mBuildCtrlFromPose = false;
	mTargetVelX = 3.0;
}

cScenarioTrackMotion::~cScenarioTrackMotion()
{

}

void cScenarioTrackMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser->ParseString("motion_file", mMotionFile);
	parser->ParseString("BVHMotionFile", mBVHMotionFile);
	parser->ParseInt("target_ctrl_id", mTargetCtrlID);
	parser->ParseIntArray("target_actions", mTargetActions);
	parser->ParseBool("build_ctrl_from_pose", mBuildCtrlFromPose);
	parser->ParseDouble("target_vel_x", mTargetVelX);
}

void cScenarioTrackMotion::Reset()
{
	cScenarioSimChar::Reset();
	mKinChar.Reset();
}

void cScenarioTrackMotion::Clear()
{
	cScenarioSimChar::Clear();
	mKinChar.Clear();
	mTargetCtrlID = 0;
	mTargetActions.clear();
}

void cScenarioTrackMotion::Update(double time_elapsed)
{
	cScenarioSimChar::Update(time_elapsed);

	if (time_elapsed > 0)
	{
		UpdateKinChar(time_elapsed);
	}
}

void cScenarioTrackMotion::InitTime(double time)
{
	mTime = time;
	mKinChar.SetTime(mTime);
	mKinChar.Update(0);
	SyncCharacters();
}

const cKinCharacter& cScenarioTrackMotion::GetKinCharacter() const
{
	return mKinChar;
}

double cScenarioTrackMotion::CalcPosePosErr() const
{
	Eigen::VectorXd kin_pose;
	CalcSyncKinPose(kin_pose);
	
	const Eigen::MatrixXd& joint_mat = mKinChar.GetJointMat();
	tVector kin_root_pos = cKinTree::GetRootPos(joint_mat, kin_pose);
	tVector sim_root_pos = mChar->GetRootPos();

	double total_err = 0;
	int num_joints = mKinChar.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		double w = mChar->GetJointDiffWeight(j);

		tVector kin_pos = cKinTree::CalcJointWorldPos(joint_mat, kin_pose, j);
		tVector sim_pos = mChar->CalcJointPos(j);
		kin_pos[0] -= kin_root_pos[0];
		sim_pos[0] -= sim_root_pos[0];
		tVector err = kin_pos - sim_pos;
		total_err += w * err.squaredNorm();
	}
	return total_err;
}

double cScenarioTrackMotion::CalcPoseThetaErr() const
{
	Eigen::VectorXd kin_pose;
	CalcSyncKinPose(kin_pose);

	const Eigen::MatrixXd& joint_mat = mKinChar.GetJointMat();

	double total_err = 0;
	int num_joints = mKinChar.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		double w = mChar->GetJointDiffWeight(j);

		double kin_theta;
		tVector kin_axis;
		cKinTree::CalcJointWorldTheta(joint_mat, kin_pose, j, kin_axis, kin_theta);

		double sim_theta;
		tVector sim_axis;
		mChar->CalcJointWorldRotation(j, sim_axis, sim_theta);

		double delta_theta;
		tVector delta_axis;
		cMathUtil::DeltaRot(kin_axis, kin_theta, sim_axis, sim_theta, delta_axis, delta_theta);
		total_err += w * delta_theta * delta_theta;
	}
	return total_err;
}

double cScenarioTrackMotion::CalcPoseThetaRelErr() const
{
	const auto& ctrl = mChar->GetController();

	Eigen::VectorXd kin_pose;
	Eigen::VectorXd sim_pose;
	CalcSyncKinPose(kin_pose);
	ctrl->BuildNormPose(sim_pose);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	double total_err = 0;
	int num_joints = mKinChar.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		int offset = mChar->GetParamOffset(j);
		int size = mChar->GetParamSize(j);

		double err = 0;
		err = (kin_pose - sim_pose).segment(offset, size).squaredNorm();
		double w = mChar->GetJointDiffWeight(j);
		total_err += w * err;
	}
	return total_err;
}

double cScenarioTrackMotion::CalcVelErr() const
{
	Eigen::VectorXd kin_vel = mKinChar.GetVel();
	Eigen::VectorXd sim_vel = mChar->GetVel();
	
	double total_err = 0;
	int num_joints = mKinChar.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		int offset = mChar->GetParamOffset(j);
		int size = mChar->GetParamSize(j);

		double err = (kin_vel - sim_vel).segment(offset, size).squaredNorm();
		double w = mChar->GetJointDiffWeight(j);
		total_err += w * err;
	}
	return total_err;
}

tVector cScenarioTrackMotion::CalcRootPosErr() const
{
	tVector kin_root_pos = mKinChar.GetRootPos();
	tVector sim_root_pos = mChar->GetRootPos();
	return kin_root_pos - sim_root_pos;
}

double cScenarioTrackMotion::CalcEffort() const
{
#if defined(ENABLE_TRAINING)
	double effort = mChar->CalcEffort();
#else
	double effort = 0;
#endif
	return effort;
}

double cScenarioTrackMotion::CalcTargetVelErr() const
{
	tVector vel = mChar->CalcCOMVel();
	double vel_err = mTargetVelX - vel[0];
	vel_err *= vel_err;
	return vel_err;
}

int cScenarioTrackMotion::GetTargetCtrlID() const
{
	return mTargetCtrlID;
}

const std::vector<int>& cScenarioTrackMotion::GetTargetActions() const
{
	return mTargetActions;
}

std::string cScenarioTrackMotion::GetName() const
{
	return "Track Motion";
}

bool cScenarioTrackMotion::BuildCharacter()
{
	bool succ = mKinChar.Init(mCharParams.mCharFile, mMotionFile);
	if (succ)
	{
		tVector root_pos = mKinChar.GetRootPos();
		tVector delta = tVector::Zero();
		delta[0] = mCharParams.mInitPos[0] - root_pos[0];
		mKinChar.MoveOrigin(delta);
	}

	succ &= cScenarioSimChar::BuildCharacter();
	
	if (succ)
	{
		SetupInitState();
	}
	return succ;
}

tVector cScenarioTrackMotion::GetDefaultCharPos() const
{
	//return tVector(0, 0.7, 0, 0);
	return cScenarioSimChar::GetDefaultCharPos();
}

bool cScenarioTrackMotion::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cScenarioSimChar::BuildController(out_ctrl);
	int tar_action = (mTargetActions.size() > 0) ? mTargetActions[0] : 0;
	out_ctrl->CommandAction(tar_action);
	out_ctrl->SetDefaultAction(tar_action);

	if (mBuildCtrlFromPose)
	{
		BuildCtrlParamsFromPose(mTargetCtrlID, out_ctrl);
	}
	return succ;
}

void cScenarioTrackMotion::SetupInitState()
{
	bool has_state_file = mCharParams.mStateFile != "";
	if (!has_state_file)
	{
		const Eigen::VectorXd& pose = mKinChar.GetPose0();
		const Eigen::VectorXd& vel = mKinChar.GetVel0();
		
		mChar->SetPose0(pose);
		mChar->SetPose(pose);
		mChar->SetVel0(vel);
		mChar->SetVel(vel);
	}
}

void cScenarioTrackMotion::BuildCtrlParamsFromPose(int target_ctrl_id, std::shared_ptr<cCharController>& out_ctrl)
{
	out_ctrl->BuildFromMotion(target_ctrl_id, mKinChar.GetMotion());

	// hack
	//FILE* f = cFileUtil::OpenFile("out_params.txt", "w");
	//Eigen::VectorXd opt_params;
	//out_ctrl->BuildOptParams(opt_params);
	//out_ctrl->OutputOptParams(f, opt_params);
	//cFileUtil::CloseFile(f);
}

bool cScenarioTrackMotion::IsNewCycle() const
{
	const auto& ctrl = mChar->GetController();
	return ctrl->IsNewCycle();
}

void cScenarioTrackMotion::SyncCharacters()
{
	double time = mKinChar.GetTime();
	const auto& ctrl = mChar->GetController();
	
	const cMotion& motion = mKinChar.GetMotion();
	int num_states = ctrl->GetNumStates();
	int state = 0;
	double phase = 0;

	double dur = motion.GetDuration();
	phase = time / dur * num_states;
	state = static_cast<int>(std::floor(phase));
	phase -= static_cast<int>(phase);

	state %= num_states;
	if (time < 0)
	{
		phase += 1;
		state += num_states;
		state %= num_states;
	}
	
	ctrl->TransitionState(state);
	ctrl->SetPhase(phase);
	
	const Eigen::VectorXd& char_state = mKinChar.GetPose();
	const Eigen::VectorXd& char_vel = mKinChar.GetVel();
	mChar->SetPose(char_state);
	mChar->SetVel(char_vel);

	tVector aabb_min = tVector::Zero();
	tVector aabb_max = tVector::Zero();
	mChar->CalcAABB(aabb_min, aabb_max);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	double h_delta = aabb_min[1] - ground_h;
	if (h_delta < 0)
	{
		root_pos[1] -= h_delta;
		mChar->SetRootPos(root_pos);
	}
}

void cScenarioTrackMotion::CalcSyncKinPose(Eigen::VectorXd& out_pose) const
{
	const auto& ctrl = mChar->GetController();
	double phase = ctrl->CalcNormPhase();
	double motion_dur = mKinChar.GetMotionDuration();
	double kin_time = motion_dur * phase;
	mKinChar.CalcPose(kin_time, out_pose);

	const Eigen::MatrixXd& joint_mat = mKinChar.GetJointMat();
	tVector sim_pos = mChar->GetRootPos();
	tVector kin_pos = cKinTree::GetRootPos(joint_mat, out_pose);
	kin_pos[0] = sim_pos[0];
	cKinTree::SetRootPos(joint_mat, kin_pos, out_pose);
}

void cScenarioTrackMotion::UpdateKinChar(double time_step)
{
	mKinChar.Update(time_step);
}

void cScenarioTrackMotion::CommandAction(int action_id)
{
	const auto& ctrl = mChar->GetController();
	ctrl->CommandAction(action_id);
}