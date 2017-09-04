#include "scenarios/ScenarioRetargetMotion.h"
#include "util/FileUtil.h"

cScenarioRetargetMotion::cScenarioRetargetMotion()
{
}

cScenarioRetargetMotion::~cScenarioRetargetMotion()
{
}

void cScenarioRetargetMotion::Init()
{
	BuildCharacter(mSrcCharFile, mSrcChar);
	BuildCharacter(mDstCharFile, mDstChar);
}

void cScenarioRetargetMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	parser->ParseString("src_char_file", mSrcCharFile);
	parser->ParseString("dst_char_file", mDstCharFile);
	parser->ParseStringArray("motion_files", mMotionFiles);
	parser->ParseString("output_path", mOutputPath);
	parser->ParseIntArray("retarget_src_joints", mRetargetSrcJoints);
	parser->ParseIntArray("retarget_dst_joints", mRetargetDstJoints);
}

void cScenarioRetargetMotion::Clear()
{
	cScenario::Clear();

	mSrcCharFile = "";
	mDstCharFile = "";
	mMotionFiles.clear();
}

void cScenarioRetargetMotion::Run()
{
	RetargetMotions(mMotionFiles, mSrcChar, mDstChar);
}

std::string cScenarioRetargetMotion::GetName() const
{
	return "Retarget Motion";
}

bool cScenarioRetargetMotion::BuildCharacter(const std::string& char_file, cKinCharacter& out_char) const
{
	bool succ = out_char.Init(char_file, "");
	if (!succ)
	{
		printf("Failed to load character from %s\n", char_file.c_str());
	}
	return succ;
}

void cScenarioRetargetMotion::RetargetMotions(const std::vector<std::string>& motion_files, cKinCharacter& src_char, cKinCharacter& dst_char)
{
	size_t num_files = motion_files.size();
	for (size_t f = 0; f < num_files; ++f)
	{
		const std::string& curr_motion_file = motion_files[f];
		bool succ = src_char.LoadMotion(curr_motion_file);
		if (!succ)
		{
			printf("Failed to load motion from %s\n", curr_motion_file.c_str());
		}
		else
		{
			const cMotion& src_motion = src_char.GetMotion();
			cMotion dst_motion;
			int num_frames = src_motion.GetNumFrames();
			int dst_pose_dim = dst_char.GetNumDof();
			dst_motion.Init(num_frames, dst_pose_dim);

			for (int f = 0; f < num_frames; ++f)
			{
				double curr_frame_time = src_motion.GetFrameTime(f);
				src_char.Pose(curr_frame_time);
				Eigen::VectorXd dst_pose;
				RetargetPose(src_char, dst_char, dst_pose);

				dst_motion.SetFrame(f, dst_pose);
				dst_motion.SetFrameTime(f, curr_frame_time);
			}

			std::string ext = cFileUtil::GetExtension(curr_motion_file);
			std::string filename = cFileUtil::GetFilename(curr_motion_file);
			std::string dst_motion_file = mOutputPath + filename + "_retargeted." + ext;
			dst_motion.Output(dst_motion_file);
		}
	}
}

void cScenarioRetargetMotion::RetargetPose(const cKinCharacter& src_char, cKinCharacter& dst_char, Eigen::VectorXd& out_pose) const
{
	int dst_pose_dim = dst_char.GetNumDof();
	out_pose = Eigen::VectorXd::Zero(dst_pose_dim);

	int num_dst_joints = dst_char.GetNumJoints();
	int num_dst_retarget_joints = static_cast<int>(mRetargetDstJoints.size());
	int num_src_retarget_joints = static_cast<int>(mRetargetSrcJoints.size());
	assert(num_dst_retarget_joints == num_dst_joints);
	assert(num_dst_retarget_joints == num_src_retarget_joints);

	for (size_t j = 0; j < num_dst_retarget_joints; ++j)
	{
		int src_j = mRetargetSrcJoints[j];
		int dst_j = mRetargetDstJoints[j];

		int dst_param_offset = dst_char.GetParamOffset(dst_j);
		int dst_param_size = dst_char.GetParamSize(dst_j);

		Eigen::VectorXd dst_joint_pose;
		ConvertJointPose(src_char, src_j, dst_char, dst_j, dst_joint_pose);
		assert(dst_joint_pose.size() == dst_param_size);
		out_pose.segment(dst_param_offset, dst_param_size) = dst_joint_pose;
	}
}

void cScenarioRetargetMotion::ConvertJointPose(const cKinCharacter& src_char, int src_j, const cKinCharacter& dst_char, int dst_j, Eigen::VectorXd& out_dst_joint_pose) const
{
	const auto& src_joint_mat = src_char.GetJointMat();
	const auto& dst_joint_mat = dst_char.GetJointMat();

	int src_param_offset = src_char.GetParamOffset(src_j);
	int src_param_size = src_char.GetParamSize(src_j);
	int dst_param_offset = dst_char.GetParamOffset(dst_j);
	int dst_param_size = dst_char.GetParamSize(dst_j);

	const Eigen::VectorXd& src_pose = src_char.GetPose();
	auto src_joint_pose = src_pose.segment(src_param_offset, src_param_size);
	
	out_dst_joint_pose = Eigen::VectorXd::Zero(dst_param_size);

	cKinTree::eJointType src_type = cKinTree::GetJointType(src_joint_mat, src_j);
	cKinTree::eJointType dst_type = cKinTree::GetJointType(dst_joint_mat, dst_j);

	if (src_type == dst_type)
	{
		out_dst_joint_pose = src_joint_pose;
	}
	else if (src_type == cKinTree::eJointTypeSpherical
		&& dst_type == cKinTree::eJointTypeRevolute)
	{
		const tVector ref_axis = tVector(0, 0, 1, 0);

		tQuaternion src_quat = cMathUtil::VecToQuat(src_joint_pose);
		tVector src_axis;
		double src_theta;
		cMathUtil::QuaternionToAxisAngle(src_quat, src_axis, src_theta);
		src_theta = (src_axis.dot(ref_axis) < 0) ? -src_theta : src_theta;

		out_dst_joint_pose[0] = src_theta;
	}
	else
	{
		assert(false); // unsupported joint type conversion
		out_dst_joint_pose = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(dst_param_size);
	}
}