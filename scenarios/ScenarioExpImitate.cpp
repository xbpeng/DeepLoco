#include "ScenarioExpImitate.h"
#include "sim/RBDUtil.h"
#include "sim/SimCharSoftFall.h"
#include "sim/CtPhaseController.h"
#include "sim/WaypointController.h"

//#define ENABLE_MAX_POSE_ERR
//#define ENABLE_HEADING_REWARD

const int gNumWarmupCycles = 0;

double cScenarioExpImitate::CalcReward() const
{
	double pose_w = 0.5;
	double vel_w = 0.05;
	double end_eff_w = 0.15;
	double root_w = 0.1;
	double com_w = 0.2;

	double total_w = pose_w + vel_w + end_eff_w + root_w + com_w;
	pose_w /= total_w;
	vel_w /= total_w;
	end_eff_w /= total_w;
	root_w /= total_w;
	com_w /= total_w;

#if defined(ENABLE_MAX_POSE_ERR)
	const double pose_scale = 20;
	const double vel_scale = 0.2;
#else
	const double pose_scale = 2;
	const double vel_scale = 0.005;
#endif
	const double end_eff_scale = 40;
	const double root_scale = 10;
	const double com_scale = 10;

	const double err_scale = 1;

	bool fallen = HasFallen();
	const auto& joint_mat = mChar->GetJointMat();
	const auto& body_defs = mChar->GetBodyDefs();
	double reward = 0;

	if (!fallen)
	{
		Eigen::VectorXd pose0 = mChar->GetPose();
		Eigen::VectorXd vel0 = mChar->GetVel();
		Eigen::VectorXd pose1 = mKinChar->GetPose();
		Eigen::VectorXd vel1 = mKinChar->GetVel();
		tMatrix origin_trans = mChar->BuildOriginTrans();

		tVector com_vel0_world = mChar->CalcCOMVel();
		tVector com_vel1_world = cRBDUtil::CalcCoMVel(joint_mat, body_defs, pose1, vel1);

		cKinTree::NormalizePoseHeading(joint_mat, pose0, vel0);
		cKinTree::NormalizePoseHeading(joint_mat, pose1, vel1);

		int root_id = mChar->GetRootID();
		tVector root_pos0 = cKinTree::GetRootPos(joint_mat, pose0);
		tVector root_pos1 = cKinTree::GetRootPos(joint_mat, pose1);
		tVector root_vel0 = cKinTree::GetRootVel(joint_mat, vel0);
		tVector root_vel1 = cKinTree::GetRootVel(joint_mat, vel1);

		tVector com0;
		tVector com_vel0;
		tVector com1;
		tVector com_vel1;
		cRBDUtil::CalcCoM(joint_mat, body_defs, pose0, vel0, com0, com_vel0);
		cRBDUtil::CalcCoM(joint_mat, body_defs, pose1, vel1, com1, com_vel1);

		double pose_err = 0;
		double vel_err = 0;
		double end_eff_err = 0;
		double root_err = 0;
		double com_err = 0;
		double heading_err = 0;

		int num_end_effs = 0;
		int num_joints = mChar->GetNumJoints();
		assert(num_joints == mJointWeights.size());

		double root_rot_w = mJointWeights[root_id];
		pose_err += root_rot_w * cKinTree::CalcRootRotErr(joint_mat, pose0, pose1);
		vel_err += root_rot_w * cKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1);

		for (int j = root_id + 1; j < num_joints; ++j)
		{
			double w = mJointWeights[j];

#if defined(ENABLE_MAX_POSE_ERR)
			tVector curr_pos0 = mChar->GetBodyPartPos(j);
			tVector curr_pos1 = cKinTree::CalcBodyPartPos(joint_mat, body_defs, pose1, j);
			tVector curr_vel0 = mChar->GetBodyPartVel(j);
			tVector curr_vel1 = cKinTree::CalcBodyPartVel(joint_mat, body_defs, pose1, vel1, j);

			curr_pos0 -= root_pos0;
			curr_pos1 -= root_pos1;

			double curr_pose_err = (curr_pos1 - curr_pos0).squaredNorm();
			double curr_vel_err = (curr_vel1 - curr_vel0).squaredNorm();
#else
			double curr_pose_err = cKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
			double curr_vel_err = cKinTree::CalcVelErr(joint_mat, j, vel0, vel1);
#endif
			pose_err += w * curr_pose_err;
			vel_err += w * curr_vel_err;

			bool is_end_eff = mChar->IsEndEffector(j);
			if (is_end_eff)
			{
				tVector pos0 = mChar->CalcJointPos(j);
				tVector pos1 = cKinTree::CalcJointWorldPos(joint_mat, pose1, j);
				double ground_h0 = mGround->SampleHeight(pos0);
				double ground_h1 = 0;

				pos0[3] = 1;
				pos0 = origin_trans * pos0;
				pos0[3] = 0;

				tVector pos_rel0 = pos0 - com0;
				tVector pos_rel1 = pos1 - com1;
				pos_rel0[1] = pos0[1] - ground_h0;
				pos_rel1[1] = pos1[1] - ground_h1;

				double curr_end_err = (pos_rel1 - pos_rel0).squaredNorm();
				end_eff_err += curr_end_err;
				++num_end_effs;
			}
		}

		if (num_end_effs > 0)
		{
			end_eff_err /= num_end_effs;
		}

		double root_ground_h0 = mGround->SampleHeight(mChar->GetRootPos());
		double root_ground_h1 = 0;
		double h0 = root_pos0[1] - root_ground_h0;
		double h1 = root_pos1[1] - root_ground_h1;;
		root_err = (h1 - h0) * (h1 - h0) + 0 * 0.01 * (root_vel1 - root_vel0).squaredNorm();

		com0[0] = 0;
		com1[0] = 0;
		com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

		double pose_reward = exp(-err_scale * pose_scale * pose_err);
		double vel_reward = exp(-err_scale * vel_scale * vel_err);
		double end_eff_reward = exp(-err_scale * end_eff_scale * end_eff_err);
		double root_reward = exp(-err_scale * root_scale * root_err);
		double com_reward = exp(-err_scale * com_scale * com_err);

		reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward
			+ root_w * root_reward + com_w * com_reward;
	}

	return reward;
}

cScenarioExpImitate::cScenarioExpImitate()
{
	mMotionFile = "";
	mEnableRandStateReset = true;
	mCharParams.mEnableFallDist = false;
	mCharParams.mEnableSoftContact = false;
}

cScenarioExpImitate::~cScenarioExpImitate()
{
}

void cScenarioExpImitate::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpCacla::ParseArgs(parser);
	parser->ParseString("motion_file", mMotionFile);
	parser->ParseBool("enable_rand_state_reset", mEnableRandStateReset);
}

void cScenarioExpImitate::Init()
{
	bool succ = BuildKinCharacter(mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}

	cScenarioExpCacla::Init();
	InitJointWeights();

	if (EnableSyncChar())
	{
		SyncCharacters();
		InitCharacterPos(mChar);
		ResolveCharGroundIntersect(mChar);
	}
}

const std::shared_ptr<cKinCharacter>& cScenarioExpImitate::GetKinChar() const
{
	return mKinChar;
}

void cScenarioExpImitate::EnableRandStateReset(bool enable)
{
	mEnableRandStateReset = enable;
}

bool cScenarioExpImitate::EnabledRandStateReset() const
{
	return mEnableRandStateReset;
}

bool cScenarioExpImitate::HasFallen() const
{
	bool fallen = cScenarioSimChar::HasFallen();
	return fallen;
}

std::string cScenarioExpImitate::GetName() const
{
	return "Imitate Exploration";
}

void cScenarioExpImitate::CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const
{
	int num_joints = character->GetNumJoints();
	out_weights = Eigen::VectorXd::Ones(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		double curr_w = character->GetJointDiffWeight(j);
		out_weights[j] = curr_w;
	}

	double sum = out_weights.lpNorm<1>();
	out_weights /= sum;
}

void cScenarioExpImitate::SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const
{
	cScenarioExpCacla::SetupControllerParams(out_params);
	out_params.mCycleDur = mKinChar->GetMotionDuration();
}

bool cScenarioExpImitate::BuildKinCharacter(std::shared_ptr<cKinCharacter>& out_char) const
{
	auto kin_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	kin_char->EnableVelUpdate(true);
	bool succ = kin_char->Init(mCharParams.mCharFile, mMotionFile);
	if (succ)
	{
		out_char = kin_char;
	}
	return succ;
}

void cScenarioExpImitate::UpdateCharacter(double time_step)
{
	UpdateKinChar(time_step);
	UpdateTrackController();
	cScenarioExpCacla::UpdateCharacter(time_step);
}

void cScenarioExpImitate::UpdateKinChar(double time_step)
{
	mKinChar->Update(time_step);
	//SyncCharacters(); // hack hack hack
}

void cScenarioExpImitate::UpdateTrackController()
{
	auto ctrl = mChar->GetController();
}

void cScenarioExpImitate::ResetParams()
{
	cScenarioExpCacla::ResetParams();
}

void cScenarioExpImitate::ResetCharacters()
{
	cScenarioExpCacla::ResetCharacters();

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
}

void cScenarioExpImitate::ResetKinChar()
{
	mKinChar->Reset();
	if (EnabledRandStateReset())
	{
		double rand_time = CalcRandKinResetTime();
		mKinChar->SetTime(rand_time);
		mKinChar->Pose(rand_time);
	}
}

void cScenarioExpImitate::SyncCharacters()
{
	const Eigen::VectorXd& pose = mKinChar->GetPose();
	const Eigen::VectorXd& vel = mKinChar->GetVel();
	
	mChar->SetPose(pose);
	mChar->SetVel(vel);

	const auto& ctrl = mChar->GetController();
	auto phase_ctrl = std::dynamic_pointer_cast<cCtPhaseController>(ctrl);
	if (phase_ctrl != nullptr)
	{
		double kin_time = mKinChar->GetTime();
		phase_ctrl->SetTime(kin_time);
	}

	auto waypoint_ctrl = std::dynamic_pointer_cast<cWaypointController>(ctrl);
	if (waypoint_ctrl != nullptr)
	{
		double kin_time = mKinChar->GetTime();
		waypoint_ctrl->SetTime(kin_time);
	}
}

bool cScenarioExpImitate::EnableSyncChar() const
{
	return true;
}

void cScenarioExpImitate::InitJointWeights()
{
	CalcJointWeights(mChar, mJointWeights);
}

int cScenarioExpImitate::GetNumWarmupCycles() const
{
	return gNumWarmupCycles;
}

bool cScenarioExpImitate::EndEpisode() const
{
	bool is_end = cScenarioExpCacla::EndEpisode();
	is_end |= mKinChar->IsMotionOver();
	return is_end;
}

double cScenarioExpImitate::CalcRandKinResetTime()
{
	double dur = mCtrlParams.mCycleDur;
	double rand_time = mRand.RandDouble(0, dur);
	return rand_time;
}