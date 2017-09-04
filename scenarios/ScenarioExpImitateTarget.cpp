#include "ScenarioExpImitateTarget.h"
#include "sim/RBDUtil.h"
#include "sim/CtTargetController.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundDynamicObstacles3D.h"

const double gMaxTargetDist = 3;

double cScenarioExpImitateTarget::CalcTargetReward(const tVector& tar_pos) const
{
	double pose_w = 0.5;
	double vel_w = 0.05;
	double end_eff_w = 0.15;
	double root_w = 0.1;
	double com_w = 0.2;
	double heading_w = 0.05;
	double target_pos_w = 0.1;
	//double heading_w = 0.1;
	//double target_pos_w = 0.2;

	double total_w = pose_w + vel_w + end_eff_w + root_w + com_w + heading_w + target_pos_w;
	pose_w /= total_w;
	vel_w /= total_w;
	end_eff_w /= total_w;
	root_w /= total_w;
	com_w /= total_w;
	heading_w /= total_w;
	target_pos_w /= total_w;

	const double max_target_dist = GetRandTargetMaxDist();

	//const double pose_scale = 10;
	//const double vel_scale = 0.05;
	//const double pose_scale = 4;
	//const double vel_scale = 0.02;
	const double pose_scale = 2;
	const double vel_scale = 0.005;
	//const double vel_scale = 0.01;
	const double end_eff_scale = 40;
	const double root_scale = 10;
	const double com_scale = 10;
	const double target_pos_scale = 1 / (0.25 * max_target_dist * max_target_dist);

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

		tVector com_vel0_world = cRBDUtil::CalcCoMVel(joint_mat, body_defs, pose0, vel0);
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
			double curr_pose_err = cKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
			double curr_vel_err = cKinTree::CalcVelErr(joint_mat, j, vel0, vel1);

			pose_err += w * curr_pose_err;
			vel_err += w * curr_vel_err;

			bool is_end_eff = mChar->IsEndEffector(j);
			if (is_end_eff)
			{
				tVector pos0 = cKinTree::CalcBodyPartPos(joint_mat, body_defs, pose0, j);
				tVector pos1 = cKinTree::CalcBodyPartPos(joint_mat, body_defs, pose1, j);
				tVector pos_rel0 = pos0 - com0;
				tVector pos_rel1 = pos1 - com1;

				double ground_h0 = mGround->SampleHeight(pos0);
				double ground_h1 = 0;
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

		tVector root_pos_world = mChar->GetRootPos();
		double root_ground_h0 = mGround->SampleHeight(root_pos_world);
		double root_ground_h1 = 0;
		double h0 = root_pos0[1] - root_ground_h0;
		double h1 = root_pos1[1] - root_ground_h1;
		root_err = (h1 - h0) * (h1 - h0) + 0 * 0.01 * (root_vel1 - root_vel0).squaredNorm();

		tVector target_delta = tar_pos - root_pos_world;
		target_delta[1] = 0;
		tVector target_dir = target_delta.normalized();
		double target_theta = std::atan2(-target_dir[2], target_dir[0]);
		tMatrix target_dir_trans = cMathUtil::RotateMat(tVector(0, 1, 0, 0), target_theta);
		double target_dist_sq = target_delta.squaredNorm();
		double target_pos_err = target_dist_sq;

		com0[0] = 0;
		com1[0] = 0;
		com_vel1_world = target_dir_trans * com_vel1_world;
		com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

		double heading0 = mChar->CalcHeading();
		//double heading1 = mKinChar->CalcHeading();
		double heading1 = target_theta;
		heading_err = std::abs(heading1 - heading0);
		heading_err = std::min(2 * M_PI - heading_err, heading_err);

		const double target_dist_threshold = GetRandTargetMaxDist();
		if (target_dist_sq < target_dist_threshold * target_dist_threshold)
		{
			com_err = 0;
			heading_err = 0;
		}

		double pose_reward = exp(-err_scale * pose_scale * pose_err);
		double vel_reward = exp(-err_scale * vel_scale * vel_err);
		double end_eff_reward = exp(-err_scale * end_eff_scale * end_eff_err);
		double root_reward = exp(-err_scale * root_scale * root_err);
		double com_reward = exp(-err_scale * com_scale * com_err);
		double heading_reward = 0.5 * (std::cos(heading_err) + 1);
		heading_reward = std::pow(heading_reward, 4);
		double target_pos_reward = exp(-err_scale * target_pos_scale * target_pos_err);

		reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward
			+ root_w * root_reward + com_w * com_reward + heading_w * heading_reward
			+ target_pos_w * target_pos_reward;
	}

	return reward;
}

cScenarioExpImitateTarget::cScenarioExpImitateTarget()
{
	mEnableRandCharPlacement = true;
	mRandTargetPosTimeMin = 1;
	mRandTargetPosTimeMax = 5;
	mRandTargetPosTimer = 0;
	mTargetResetDist = 0.5;
	mTargetPos.setZero();
	EnableRandTargetPos(true);
}

cScenarioExpImitateTarget::~cScenarioExpImitateTarget()
{
}

void cScenarioExpImitateTarget::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioExpImitate::ParseArgs(parser);
	parser->ParseBool("enable_rand_char_placement", mEnableRandCharPlacement);
	parser->ParseDouble("rand_target_pos_time_min", mRandTargetPosTimeMin);
	parser->ParseDouble("rand_target_pos_time_max", mRandTargetPosTimeMax);
	parser->ParseDouble("target_reset_dist", mTargetResetDist);
}

void cScenarioExpImitateTarget::Init()
{
	cScenarioExpImitate::Init();
	ResetTargetPos();
}

void cScenarioExpImitateTarget::Reset()
{
	cScenarioExpImitate::Reset();
	ResetTargetPos();
}

void cScenarioExpImitateTarget::Update(double time_elapsed)
{
	cScenarioExpImitate::Update(time_elapsed);
	UpdateTargetPos(time_elapsed);
}

const tVector& cScenarioExpImitateTarget::GetTargetPos() const
{
	return mTargetPos;
}

void cScenarioExpImitateTarget::SetTargetPos(const tVector& target_pos)
{
	mTargetPos = target_pos;
	UpdateTargetController();

#if defined(ENABLE_DEBUG_PRINT)
	printf("Target Pos: %.5f, %.5f, %.5f\n", target_pos[0], target_pos[1], target_pos[2]);
#endif
}

void cScenarioExpImitateTarget::EnableRandTargetPos(bool enable)
{
	mEnableRandTargetPos = enable;
}

bool cScenarioExpImitateTarget::EnabledRandTargetPos() const
{
	return mEnableRandTargetPos;
}

double cScenarioExpImitateTarget::GetTargetResetDist() const
{
	return mTargetResetDist;
}

std::string cScenarioExpImitateTarget::GetName() const
{
	return "Imitate Target Exploration";
}


bool cScenarioExpImitateTarget::CheckResetTarget() const
{
	tVector root_pos = mChar->GetRootPos();
	tVector tar_pos = mTargetPos;
	root_pos[1] = 0;
	tar_pos[1] = 0;
	double dist = (tar_pos - root_pos).squaredNorm();
	const double dist_threshold = GetTargetResetDist();

	bool reset_target = (mRandTargetPosTimer <= 0)
						|| (dist < dist_threshold * dist_threshold);
	//|| root_pos[0] + dist_threshold > tar_pos[0]; // hack

	return reset_target;
}


void cScenarioExpImitateTarget::InitCharacterPos(const std::shared_ptr<cSimCharacter>& out_char)
{
	if (mEnableRandCharPlacement)
	{
		SetCharRandPlacement(out_char);
	}
}

void cScenarioExpImitateTarget::SetCharRandPlacement(const std::shared_ptr<cSimCharacter>& out_char)
{
	tVector rand_pos = tVector::Zero();
	tQuaternion rand_rot = tQuaternion::Identity();
	CalcCharRandPlacement(rand_pos, rand_rot);
	out_char->SetRootTransform(rand_pos, rand_rot);
}

void cScenarioExpImitateTarget::CalcCharRandPlacement(tVector& out_pos, tQuaternion& out_rot)
{
	auto ground_class = mGround->GetGroundClass();
	switch (ground_class)
	{
	case cGround::eClassTrail3D:
		CalcCharRandPlacementTrail3D(out_pos, out_rot);
		break;
	case cGround::eClassObstacles3D:
		CalcCharRandPlacementObstacles3D(out_pos, out_rot);
		break;
	case cGround::eClassDynamicObstacles3D:
	case cGround::eClassConveyor3D:
		CalcCharRandPlacementDynamicObstacles3D(out_pos, out_rot);
		break;
	default:
		CalcCharRandPlacementDefault(out_pos, out_rot);
		break;
	}
}

void cScenarioExpImitateTarget::CalcCharRandPlacementTrail3D(tVector& out_pos, tQuaternion& out_rot)
{
	tQuaternion root_rot = mChar->GetRootRotation();
	auto trail = std::dynamic_pointer_cast<cGroundTrail3D>(mGround);
	trail->FindRandTrailPlacement(out_pos, out_rot);
	out_rot = out_rot * root_rot;
}

void cScenarioExpImitateTarget::CalcCharRandPlacementObstacles3D(tVector& out_pos, tQuaternion& out_rot)
{
	auto obstacles = std::dynamic_pointer_cast<cGroundObstacles3D>(mGround);
	tQuaternion root_rot = mChar->GetRootRotation();
	tVector root_pos = mChar->GetRootPos();

	const tVector axis = tVector(0, 1, 0, 0);
	double rand_theta = mRand.RandDouble(-M_PI, M_PI);

	out_pos = obstacles->FindRandFlatPos();
	out_rot = cMathUtil::AxisAngleToQuaternion(axis, rand_theta);
	out_rot = out_rot * root_rot;
}

void cScenarioExpImitateTarget::CalcCharRandPlacementDynamicObstacles3D(tVector& out_pos, tQuaternion& out_rot)
{
	const tVector buffer_size = tVector(2, 0, 2, 0);

	auto obstacles = std::dynamic_pointer_cast<cGroundDynamicObstacles3D>(mGround);
	tQuaternion root_rot = mChar->GetRootRotation();
	tVector root_pos = mChar->GetRootPos();

	const tVector axis = tVector(0, 1, 0, 0);
	double rand_theta = mRand.RandDouble(-M_PI, M_PI);

	out_pos = obstacles->FindRandFlatPos(buffer_size);
	out_rot = cMathUtil::AxisAngleToQuaternion(axis, rand_theta);
	out_rot = out_rot * root_rot;
}

void cScenarioExpImitateTarget::CalcCharRandPlacementDefault(tVector& out_pos, tQuaternion& out_rot)
{
	out_pos = mChar->GetRootPos();
	out_rot = mChar->GetRootRotation();

	double h = mGround->SampleHeight(out_pos);
	out_pos[1] += h;
}

double cScenarioExpImitateTarget::CalcReward() const
{
	return CalcTargetReward(mTargetPos);
}

void cScenarioExpImitateTarget::UpdateTargetController()
{
	auto target_ctrl = std::dynamic_pointer_cast<cCtTargetController>(mChar->GetController());
	if (target_ctrl != nullptr)
	{
		target_ctrl->SetTargetPos(mTargetPos);
	}
}

void cScenarioExpImitateTarget::UpdateTargetPos(double time_elapsed)
{
	bool reset_target = false;
	if (EnabledRandTargetPos())
	{
		mRandTargetPosTimer -= time_elapsed;

		tVector bound_min;
		tVector bound_max;
		mGround->CalcAABB(bound_min, bound_max);
		reset_target |= !cMathUtil::ContainsAABBXZ(mTargetPos, bound_min, bound_max);
		reset_target |= CheckResetTarget();
	}

	if (reset_target)
	{
		ResetTargetPos();
	}
}

double cScenarioExpImitateTarget::GetRandTargetMaxDist() const
{
	return gMaxTargetDist;
}

void cScenarioExpImitateTarget::ResetTargetPos()
{
	mRandTargetPosTimer = mRand.RandDouble(mRandTargetPosTimeMin, mRandTargetPosTimeMax);

	tVector target_pos = tVector::Zero();

	auto ground_class = mGround->GetGroundClass();
	switch (ground_class)
	{
	case cGround::eClassTrail3D:
		target_pos = CalcTargetPosTrail3D();
		break;
	case cGround::eClassObstacles3D:
		target_pos = CalcTargetPosObstacle3D();
		break;
	case cGround::eClassDynamicObstacles3D:
	case cGround::eClassConveyor3D:
		target_pos = CalcTargetPosDynamicObstacle3D();
		break;
	default:
		target_pos = CalcTargetPosDefault();
		break;
	}

	if (mGround != nullptr)
	{
		bool valid_sample;
		target_pos[1] = mGround->SampleHeight(target_pos, valid_sample);
		if (!valid_sample)
		{
			target_pos[1] = 0;
		}
	}

	SetTargetPos(target_pos);
}


tVector cScenarioExpImitateTarget::CalcTargetPosTrail3D()
{
	const int max_forward_segs = GetTargetPosTrail3dForwardSegs();

	tVector target_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();

	auto trail = std::dynamic_pointer_cast<cGroundTrail3D>(mGround);
	int num_segs = trail->GetNumSegs();
	int nearest_seg_id = trail->FindNearestSegment(root_pos);
	int target_seg_id = nearest_seg_id;
	int num_forward_segs = num_segs - nearest_seg_id - 1;
	if (num_forward_segs > 0)
	{
		num_forward_segs = std::min(num_forward_segs, max_forward_segs);
		target_seg_id += mRand.RandInt(1, num_forward_segs + 1);
	}

	const auto& target_seg = trail->GetSeg(target_seg_id);
	tVector seg_delta = target_seg.mEnd - target_seg.mStart;
	tVector seg_tan = tVector(0, 1, 0, 0).cross3(seg_delta).normalized();
	target_pos = mRand.RandDouble(0, 1) * seg_delta + target_seg.mStart;
	target_pos += mRand.RandDouble(-0.4, 0.4) * target_seg.mWidth * seg_tan;

	return target_pos;
}

tVector cScenarioExpImitateTarget::CalcTargetPosObstacle3D()
{
	auto obstacles = std::dynamic_pointer_cast<cGroundObstacles3D>(mGround);
	tVector target_pos = obstacles->FindRandFlatPos();
	return target_pos;
}

tVector cScenarioExpImitateTarget::CalcTargetPosDynamicObstacle3D()
{
	const tVector buffer_size = tVector(0.5, 0, 0.5, 0);
	auto obstacles = std::dynamic_pointer_cast<cGroundDynamicObstacles3D>(mGround);
	tVector target_pos = obstacles->FindRandFlatPos(buffer_size);
	return target_pos;
}

tVector cScenarioExpImitateTarget::CalcTargetPosDefault()
{
	const double max_dist = GetRandTargetMaxDist();
	const double min_dist = GetTargetResetDist();

	tVector target_pos = tVector::Zero();
	tVector root_pos = mChar->GetRootPos();
	// hack
	//target_pos[0] = root_pos[0] + mRand.RandDouble(-max_dist, max_dist);
	//target_pos[0] = root_pos[0] + mRand.RandDouble(0.5 * max_dist, max_dist);
	//target_pos[2] = root_pos[2] + mRand.RandDouble(-max_dist, max_dist);
	//target_pos[0] = root_pos[0] + max_dist;
	target_pos[0] = mRand.RandDouble(min_dist, max_dist);
	target_pos[0] += root_pos[0];
	target_pos[2] = 0;

	return target_pos;
}

int cScenarioExpImitateTarget::GetTargetPosTrail3dForwardSegs() const
{
	//return 10;
	return 1;
}