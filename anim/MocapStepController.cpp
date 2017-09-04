#include "anim/MocapStepController.h"
#include "anim/KinCharacter.h"
#include "util/FileUtil.h"

const double gStepsPerPeriod = 2;

const std::string gMotionFilesKey = "MotionFiles";
const std::string gClusterMotionsKey = "ClusterMotions";
const std::string gClusterDistKey = "ClusterDist";

const std::string gFeatureWeightKeys[cMocapStepController::eMotionFeatureMax] =
{
	"FeatureWeightX0",
	"FeatureWeightZ0",
	"FeatureWeightX1",
	"FeatureWeightZ1",
	"FeatureWeightX2",
	"FeatureWeightZ2",
	"FeatureWeightHeading"
};

cMocapStepController::cMocapStepController() 
{
	mCyclePeriod = 1;
	mCurrMotionID = gInvalidIdx;
	mLeftFootID = gInvalidIdx;
	mRightFootID = gInvalidIdx;
	mMotionFeatureWeights.setOnes();
	mClusterMotions = true;
	mClusterDist = 0.01;
	mEnableAutoStepUpdate = true;

	ResetParams();
}

cMocapStepController::~cMocapStepController() 
{
}

void cMocapStepController::Init(cKinCharacter* character, const std::string& param_file)
{
	cKinController::Init(character);
	ResetParams();
	InitMotionFeatureWeights(mMotionFeatureWeights);
	SetupMirrorStanceJoints();

	LoadParams(param_file);
}

void cMocapStepController::Reset()
{
	cKinController::Reset();
	ResetParams();
}

void cMocapStepController::Clear()
{
	cKinController::Clear();
	ResetParams();
}

void cMocapStepController::Update(double time_step)
{
	cKinController::Update(time_step);
	
	eStance curr_stance = GetStance();
	if (curr_stance != mPrevStance)
	{
		mPrevStance = curr_stance;

		if (mEnableAutoStepUpdate)
		{
			UpdateNewStep();
		}
	}
}

void cMocapStepController::CalcPose(double time, Eigen::VectorXd& out_pose) const
{
	CalcMotionPose(time, mCurrMotionID, out_pose);
	ApplyOriginTrans(out_pose);
}

void cMocapStepController::SetTime(double time)
{
	cKinController::SetTime(time);
	mPrevStance = cBipedStepController3D::eStanceMax;
}

void cMocapStepController::SetCyclePeriod(double period)
{
	mCyclePeriod = period;
}

void cMocapStepController::SetFootJoints(int right_end_id, int left_end_id)
{
	mRightFootID = right_end_id;
	mLeftFootID = left_end_id;
}

void cMocapStepController::SetStepPlan(const tStepPlan& step_plan)
{
	mStepPlan = step_plan;
}

cMocapStepController::eStance cMocapStepController::GetStance() const
{
	return GetStance(mTime);
}

cMocapStepController::eStance cMocapStepController::GetStance(double time) const
{
	double phase = CalcMotionPhase(time);
	int step_count = static_cast<int>(phase * gStepsPerPeriod);
	eStance stance = (step_count % 2 == 0) ? cBipedStepController3D::eStanceRight : cBipedStepController3D::eStanceLeft;
	return stance;
}

void cMocapStepController::EnableAutoStepUpdate(bool enable)
{
	mEnableAutoStepUpdate = enable;
}

void cMocapStepController::ForceStepUpdate()
{
	UpdateNewStep();
}

const cMotion& cMocapStepController::GetMotion(int m) const
{
	return mMotions[m];
}

void cMocapStepController::ResetParams()
{
	mPrevStance = cBipedStepController3D::eStanceMax;
	mOriginTrans.setZero();
	mOriginRot.setIdentity();

	if (GetNumMotions() > 0)
	{
		mCurrMotionID = 0;
	}
}

bool cMocapStepController::LoadParams(const std::string& param_file)
{
	bool succ = true;
	if (param_file != "")
	{
		std::ifstream f_stream(param_file);
		Json::Reader reader;
		Json::Value root;
		succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			mClusterMotions = root.get(gClusterMotionsKey, true).asBool();
			mClusterDist = root.get(gClusterDistKey, mClusterDist).asDouble();

			for (int f = 0; f < eMotionFeatureMax; ++f)
			{
				const std::string& curr_key = gFeatureWeightKeys[f];
				mMotionFeatureWeights[f] = root.get(curr_key, mMotionFeatureWeights[f]).asDouble();
			}
			mMotionFeatureWeights.normalize();

			if (root[gMotionFilesKey].isNull())
			{
				succ = false;
			}
			else
			{
				succ = LoadMotions(root["MotionFiles"]);
			}
		}
	}

	if (!succ)
	{
		printf("Failed to mocap step controller parameters from file %s.\n", param_file.c_str());
	}

	return succ;
}


void cMocapStepController::SetupMirrorStanceJoints()
{
	mRightLegJoints.clear();
	mLeftLegJoints.clear();

	int curr_right = mRightFootID;
	int curr_left = mLeftFootID;

	const auto& joint_mat = mChar->GetJointMat();
	while (curr_right != curr_left
		&& curr_right != gInvalidIdx
		&& curr_left != gInvalidIdx)
	{
		mRightLegJoints.push_back(curr_right);
		mLeftLegJoints.push_back(curr_left);

		curr_right = cKinTree::GetParent(joint_mat, curr_right);
		curr_left = cKinTree::GetParent(joint_mat, curr_left);
	}
}


bool cMocapStepController::LoadMotions(const Json::Value& json)
{
	bool succ = true;
	bool is_array = json.isArray();
	succ &= is_array;

	if (is_array)
	{
		std::vector<std::string> files;
		int num_files = json.size();
		files.reserve(num_files);

		for (int i = 0; i < num_files; ++i)
		{
			Json::Value json_elem = json.get(i, 0);
			std::string curr_file = json_elem.asString();
			files.push_back(curr_file);
		}
		
		LoadMotions(files);
	}
	
	return succ;
}

void cMocapStepController::LoadMotions(const std::vector<std::string>& motion_files)
{
	int num_files = static_cast<int>(motion_files.size());

	for (int f = 0; f < num_files; ++f)
	{
		const std::string& curr_file = motion_files[f];

		cMotion curr_motion;
		bool succ = curr_motion.Load(curr_file);
		if (succ)
		{
			int char_dof = mChar->GetNumDof();
			int motion_dof = curr_motion.GetNumDof();

			if (char_dof == motion_dof)
			{
				cMotion::tBlendFunc blend_func = std::bind(&cKinCharacter::BlendFrames, mChar,
					std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
				curr_motion.SetBlendFunc(blend_func);
				mMotions.push_back(curr_motion);
			}
			else
			{
				printf("DOF mismatch, char dof: %i, motion dof: %i\n", char_dof, motion_dof);
				assert(false);
			}
		}
		else
		{
			assert(false);
		}
	}

	int num_motions = GetNumMotions();
	if (mCurrMotionID == gInvalidIdx && num_motions > 0)
	{
		mCurrMotionID = 0;
	}

	ExtractMotionFeatures();

	if (num_motions > 0)
	{
		UpdateMotionFeatureWeights(mMotionFeatureWeights);
	}

	if (mClusterMotions)
	{
		ClusterMotions();
	}
}


void cMocapStepController::UpdateNewStep()
{
	mCurrMotionID = SelectNewMotion();

	if (mCurrMotionID == gInvalidIdx && GetNumMotions() > 0)
	{
		//assert(false);
		mCurrMotionID = 0;
	}

	const auto& joint_mat = mChar->GetJointMat();
	Eigen::VectorXd new_pose;
	CalcMotionPose(mTime, mCurrMotionID, new_pose);

	tQuaternion root_rot = mChar->CalcHeadingRot();
	tVector root_pos = mChar->GetRootPos();
	tQuaternion offset_root_rot = cKinTree::CalcHeadingRot(joint_mat, new_pose);
	tVector offset_root_pos = cKinTree::GetRootPos(joint_mat, new_pose);
	tQuaternion char_origin_rot = mChar->GetOriginRot();
	tVector char_origin_trans = mChar->GetOriginPos();

	mOriginRot = root_rot;
	mOriginRot = mOriginRot * offset_root_rot.inverse();
	mOriginRot = cMathUtil::QuatDiff(char_origin_rot, mOriginRot);

	mOriginTrans = root_pos - cMathUtil::QuatRotVec(root_rot * offset_root_rot.inverse(), offset_root_pos);
	mOriginTrans[1] = 0;
	mOriginTrans -= char_origin_trans;
	mOriginTrans = cMathUtil::QuatRotVec(char_origin_rot.inverse(), mOriginTrans);
}

const cMotion& cMocapStepController::GetCurrMotion() const
{
	assert(mCurrMotionID != gInvalidIdx);
	return GetMotion(mCurrMotionID);
}

int cMocapStepController::GetNumMotions() const
{
	return static_cast<int>(mMotions.size());
}

double cMocapStepController::CalcMotionPhase(double time) const
{
	double phase = time / mCyclePeriod;
	phase -= static_cast<int>(phase);
	phase = (phase < 0) ? (1 + phase) : phase;
	return phase;
}

double cMocapStepController::CalcStepPhase(double motion_phase) const
{
	double step_phase = gStepsPerPeriod * motion_phase;
	step_phase -= static_cast<int>(step_phase);
	return step_phase;
}

void cMocapStepController::CalcMotionPose(double time, int motion_id, Eigen::VectorXd& out_pose) const
{
	double curr_phase = CalcMotionPhase(mTime);
	double phase = CalcMotionPhase(time);
	double curr_step_phase = CalcStepPhase(curr_phase);
	double step_phase = CalcStepPhase(phase);

	if (time > mTime && step_phase < curr_step_phase)
	{
		step_phase = 1;
	}
	else if (time < mTime && step_phase > curr_step_phase)
	{
		step_phase = 0;
	}

	const cMotion& motion = GetMotion(motion_id);
	double dur = motion.GetDuration();
	double motion_time = dur * step_phase;
	out_pose = motion.CalcFrame(motion_time);

	if (ShouldMirrorPose(mTime))
	{
		MirrorPoseStance(out_pose);
	}
}

void cMocapStepController::ApplyOriginTrans(Eigen::VectorXd& out_pose) const
{
	const auto& joint_mat = mChar->GetJointMat();
	tVector root_pos = cKinTree::GetRootPos(joint_mat, out_pose);
	tQuaternion root_rot = cKinTree::GetRootRot(joint_mat, out_pose);

	root_rot = mOriginRot * root_rot;
	root_pos = cMathUtil::QuatRotVec(mOriginRot, root_pos);
	root_pos += mOriginTrans;

	cKinTree::SetRootPos(joint_mat, root_pos, out_pose);
	cKinTree::SetRootRot(joint_mat, root_rot, out_pose);
}

bool cMocapStepController::ShouldMirrorPose(double time) const
{
	double phase = CalcMotionPhase(time);
	int step_count = static_cast<int>(phase * gStepsPerPeriod);
	return (step_count % 2 == 1);
}

void cMocapStepController::MirrorPoseStance(Eigen::VectorXd& out_pose) const
{
	const auto& joint_mat = mChar->GetJointMat();
	cKinTree::MirrorPoseStance(joint_mat, mLeftLegJoints, mRightLegJoints, out_pose);
}

void cMocapStepController::InitMotionFeatureWeights(tMotionFeatures& out_weights) const
{
	out_weights[eMotionFeatureX0] = 0.03;
	out_weights[eMotionFeatureZ0] = 0.07;
	out_weights[eMotionFeatureX1] = 0.02;
	out_weights[eMotionFeatureZ1] = 0.02;
	out_weights[eMotionFeatureX2] = 0.25;
	out_weights[eMotionFeatureZ2] = 0.46;
	out_weights[eMotionFeatureHeading] = 1;
	out_weights.normalize();
}

void cMocapStepController::UpdateMotionFeatureWeights(tMotionFeatures& out_weights) const
{
	int num_motions = GetNumMotions();
	tMotionFeatures mean = tMotionFeatures::Zero();
	tMotionFeatures stdev = tMotionFeatures::Zero();
	for (int m = 0; m < num_motions; ++m)
	{
		const tMotionFeatures& features = mMotionFeatures[m];
		mean += features / num_motions;
	}

	for (int m = 0; m < num_motions; ++m)
	{
		const tMotionFeatures& features = mMotionFeatures[m];
		tMotionFeatures diff =(features - mean);
		diff = diff.cwiseProduct(diff);
		stdev += diff / (num_motions - 1);
	}

	tMotionFeatures scale = stdev.cwiseSqrt();
	for (int i = 0; i < stdev.size(); ++i)
	{
		scale[i] = (scale[i] == 0) ? 0 : (1 / scale[i]);
	}

	out_weights = mMotionFeatureWeights.cwiseProduct(scale);
	out_weights.normalize();
}

void cMocapStepController::ExtractMotionFeatures()
{
	int num_motions = GetNumMotions();
	mMotionFeatures.resize(num_motions);

	for (int m = 0; m < num_motions; ++m)
	{
		const cMotion& curr_motion = GetMotion(m);
		tMotionFeatures& features = mMotionFeatures[m];
		ExtractMotionFeatures(curr_motion, features);
	}
}

void cMocapStepController::ExtractMotionFeatures(const cMotion& motion, tMotionFeatures& out_features) const
{
	ExtractMotionFeatures(motion, 0, out_features);
}

void cMocapStepController::ExtractMotionFeatures(const cMotion& motion, double phase_beg, tMotionFeatures& out_features) const
{
	const Eigen::VectorXd& pose_beg = motion.CalcFramePhase(phase_beg);
	const Eigen::VectorXd& pose_end = motion.GetFrame(motion.GetNumFrames() - 1);
	ExtractMotionFeatures(pose_beg, pose_end, out_features);
}

void cMocapStepController::ExtractMotionFeatures(const Eigen::VectorXd& pose_beg, const Eigen::VectorXd& pose_end,
												tMotionFeatures& out_features) const
{
	const auto& joint_mat = mChar->GetJointMat();
	tVector right_pos_beg = cKinTree::CalcJointWorldPos(joint_mat, pose_beg, mRightFootID);
	tVector left_pos_beg = cKinTree::CalcJointWorldPos(joint_mat, pose_beg, mLeftFootID);
	tVector left_pos_end = cKinTree::CalcJointWorldPos(joint_mat, pose_end, mLeftFootID);
	double heading = cKinTree::CalcHeading(joint_mat, pose_end);

	out_features[eMotionFeatureX0] = right_pos_beg[0];
	out_features[eMotionFeatureZ0] = right_pos_beg[2];
	out_features[eMotionFeatureX1] = left_pos_beg[0];
	out_features[eMotionFeatureZ1] = left_pos_beg[2];
	out_features[eMotionFeatureX2] = left_pos_end[0];
	out_features[eMotionFeatureZ2] = left_pos_end[2];
	out_features[eMotionFeatureHeading] = heading;
}

int cMocapStepController::SelectNewMotion() const
{
	tMotionFeatures pose_features;
	BuildPoseMotionFeatures(pose_features);

	int motion_id = gInvalidIdx;
	double min_dist = std::numeric_limits<double>::infinity();
	FindNearestMotion(pose_features, motion_id, min_dist);

	return motion_id;
}

void cMocapStepController::BuildPoseMotionFeatures(tMotionFeatures& out_features) const
{
	const auto& joint_mat = mChar->GetJointMat();
	Eigen::VectorXd curr_pose = mChar->GetPose();
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, curr_pose);
	cKinTree::NormalizePoseHeading(joint_mat, curr_pose);

	bool mirror_pose = ShouldMirrorPose(mTime);
	if (mirror_pose)
	{
		MirrorPoseStance(curr_pose);
	}

	tVector curr_stance_pos = cKinTree::CalcJointWorldPos(joint_mat, curr_pose, mRightFootID);
	tVector curr_swing_pos = cKinTree::CalcJointWorldPos(joint_mat, curr_pose, mLeftFootID);

	double heading = mStepPlan.mRootHeading;
	tVector heading_dir = tVector(std::cos(heading), 0, -std::sin(heading), 0);
	heading_dir = origin_trans * heading_dir;
	
	tVector next_step = mStepPlan.mStepPos0;
	next_step[3] = 1;
	next_step = origin_trans * next_step;

	if (mStepPlan.mStance == cBipedStepController3D::eStanceLeft)
	{
		next_step[2] *= -1;
		heading_dir[2] *= -1;
	}

	out_features[eMotionFeatureX0] = curr_stance_pos[0];
	out_features[eMotionFeatureZ0] = curr_stance_pos[2];
	out_features[eMotionFeatureX1] = curr_swing_pos[0];
	out_features[eMotionFeatureZ1] = curr_swing_pos[2];
	out_features[eMotionFeatureX2] = next_step[0];
	out_features[eMotionFeatureZ2] = next_step[2];
	out_features[eMotionFeatureHeading] = std::atan2(-heading_dir[2], heading_dir[0]);
}

void cMocapStepController::FindNearestMotion(const tMotionFeatures& pose_features, int& out_motion_id, double& out_min_dist) const
{
	int motion_id = gInvalidIdx;
	double min_dist = std::numeric_limits<double>::infinity();

	double phase = CalcMotionPhase(mTime);
	for (int m = 0; m < GetNumMotions(); ++m)
	{
		const tMotionFeatures& curr_features = GetPhaseMotionFeature(phase, m);
		double dist = CalcMotionFeatureDist(curr_features, pose_features);
		if (dist < min_dist)
		{
			min_dist = dist;
			motion_id = m;
		}
	}

	out_motion_id = motion_id;
	out_min_dist = min_dist;
}

const cMocapStepController::tMotionFeatures& cMocapStepController::GetPhaseMotionFeature(double phase, int motion_id) const
{
	return mMotionFeatures[motion_id];
}

double cMocapStepController::CalcMotionFeatureDist(const tMotionFeatures& feature0, const tMotionFeatures& feature1) const
{
	tMotionFeatures diff = feature1 - feature0;
	diff = diff.cwiseProduct(mMotionFeatureWeights);
	double dist = diff.squaredNorm();
	return dist;
}

void cMocapStepController::ClusterMotions()
{
	int num_motions = GetNumMotions();
	if (num_motions > 1)
	{
		Eigen::VectorXd min_dists = Eigen::VectorXd::Zero(num_motions);
		Eigen::VectorXi min_ids = Eigen::VectorXi::Zero(num_motions);
		Eigen::VectorXi accepted = Eigen::VectorXi::Zero(num_motions);
		int num_accepted = 1;
		accepted[0] = 1;

		for (int m = 1; m < num_motions; ++m)
		{
			const tMotionFeatures& curr_features = mMotionFeatures[m];

			int min_id = gInvalidIdx;
			double min_dist = std::numeric_limits<double>::infinity();
			for (int i = 0; i < m; ++i)
			{
				bool is_accepted = accepted[i] != 0;
				if (is_accepted)
				{
					const tMotionFeatures& other_features = mMotionFeatures[i];
					double dist = CalcMotionFeatureDist(curr_features, other_features);
					if (dist < min_dist)
					{
						min_dist = dist;
						min_id = i;
					}
				}
			}

			min_dists[m] = min_dist;
			min_ids[m] = min_id;

			if (min_dist > mClusterDist)
			{
				accepted[m] = 1;
				++num_accepted;
			}
		}

		int idx = 0;
		for (int m = 0; m < num_motions; ++m)
		{
			bool is_accepted = accepted[m] != 0;
			if (is_accepted)
			{
				mMotions[idx] = mMotions[m];
				mMotionFeatures[idx] = mMotionFeatures[m];
				++idx;
			}
		}
		mMotions.resize(num_accepted);
		mMotionFeatures.resize(num_accepted);
	}
}