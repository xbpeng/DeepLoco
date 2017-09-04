#include "scenarios/ScenarioSegmentMotion.h"
#include "util/FileUtil.h"

cScenarioSegmentMotion::cScenarioSegmentMotion()
{
	mMirrorClipMode = eMirrorClipNone;
}

cScenarioSegmentMotion::~cScenarioSegmentMotion()
{
}

void cScenarioSegmentMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	parser->ParseString("character_file", mCharFile);
	parser->ParseString("motion_file", mMotionFile);
	parser->ParseString("cut_annotation_file", mCutAnnotationFile);
	parser->ParseString("output_path", mOutputPath);

	parser->ParseIntArray("left_joints", mLeftJoints);
	parser->ParseIntArray("right_joints", mRightJoints);

	std::string mirror_str;
	parser->ParseString("mirror_clip_mode", mirror_str);
	ParseMirrorClip(mirror_str, mMirrorClipMode);
}

void cScenarioSegmentMotion::Init()
{
	cScenario::Init();
	BuildCharacter(mCharFile, mMotionFile, mChar);
}

void cScenarioSegmentMotion::Run()
{
	SegmentMotion(mOutputPath);
}

std::string cScenarioSegmentMotion::GetName() const
{
	return "Segment Motion";
}

void cScenarioSegmentMotion::ParseMirrorClip(const std::string& str, eMirrorClip& out_mirror) const
{
	if (str == "" || str == "none")
	{
		out_mirror = eMirrorClipNone;
	}
	else if (str == "odd")
	{
		out_mirror = eMirrorClipOdd;
	}
	else if (str == "even")
	{
		out_mirror = eMirrorClipEven;
	}
	else
	{
		printf("Invalid mirror clip mode %s\n", str.c_str());
		assert(false);
	}
}

bool cScenarioSegmentMotion::BuildCharacter(const std::string& char_file, const std::string& motion_file, 
											cKinCharacter& out_char) const
{
	bool succ = out_char.Init(char_file, motion_file);
	if (!succ)
	{
		printf("Failed to load character from %s\n", char_file.c_str());
	}
	return succ;
}

void cScenarioSegmentMotion::SegmentMotion(const std::string& out_path)
{
	if (mChar.HasMotion())
	{
		std::vector<double> cut_times;
		bool succ = LoadCutAnnotation(mCutAnnotationFile, cut_times);

		if (succ)
		{
			const cMotion& motion = mChar.GetMotion();

			std::sort(cut_times.begin(), cut_times.end());
			std::vector<int> cut_indices;
			FindCutFrames(motion, cut_times, cut_indices);
			CutMotion(mChar, cut_indices, mMotionFile, out_path);
		}
	}
	else
	{
		printf("Failed to load motion from %s\n", mMotionFile.c_str());
	}
}

bool cScenarioSegmentMotion::LoadCutAnnotation(const std::string& cut_file, std::vector<double>& out_data) const
{
	Eigen::MatrixXd data;
	bool succ = cFileUtil::ReadMatrix(cut_file, data);
	if (succ)
	{
		out_data.resize(data.size());
		for (size_t i = 0; i < out_data.size(); ++i)
		{
			out_data[i] = data(i, 0);
		}
	}
	else
	{
		printf("Failed to load annotations from %s\n", cut_file.c_str());
	}
	return succ;
}

void cScenarioSegmentMotion::FindCutFrames(const cMotion& motion, const std::vector<double>& cut_times, std::vector<int>& out_frames)
{
	// cut_times must be sorted!
	const double time_tol = 0.01;

	int num_frames = motion.GetNumFrames();
	int num_cuts = static_cast<int>(cut_times.size());
	int time_idx = 0;

	out_frames.clear();
	for (int f = 0; f < num_frames; ++f)
	{
		double curr_cut_time = cut_times[time_idx];
		double frame_time = motion.GetFrameTime(f);
		if (std::abs(frame_time - curr_cut_time) < time_tol ||
			curr_cut_time <= frame_time)
		{
			out_frames.push_back(f);
			++time_idx;

			if (time_idx >= num_cuts)
			{
				break;
			}
		}
	}
}

void cScenarioSegmentMotion::CutMotion(const cKinCharacter& kin_char, std::vector<int>& cut_indices, const std::string& motion_file, const std::string& out_path)
{
	const cMotion& motion = kin_char.GetMotion();

	int num_cuts = static_cast<int>(cut_indices.size());
	for (int c = 0; c <= num_cuts; ++c)
	{
		int start_idx = (c > 1) ? cut_indices[c - 1] : 0;
		int end_idx = (c < num_cuts) ? cut_indices[c] : motion.GetNumFrames() - 1;
		
		cMotion curr_motion;
		ExtractMotionClip(kin_char, start_idx, end_idx, curr_motion);

		bool mirror_stance = (mMirrorClipMode == eMirrorClipEven && (c % 2 == 0))
							|| (mMirrorClipMode == eMirrorClipOdd && (c % 2 == 1));
		if (mirror_stance)
		{
			MirrorMotionStance(kin_char, curr_motion);
		}

		char str_buffer[32];
		std::sprintf(str_buffer, "%05d", c);

		std::string ext = cFileUtil::GetExtension(motion_file);
		std::string filename = cFileUtil::GetFilename(motion_file);
		std::string dst_motion_file = out_path + filename + "_" + str_buffer + "." + ext;
		curr_motion.Output(dst_motion_file);
	}
}

void cScenarioSegmentMotion::ExtractMotionClip(const cKinCharacter& kin_char, int start_idx, int end_idx, cMotion& out_motion) const
{
	const cMotion& motion = kin_char.GetMotion();
	int curr_num_frames = end_idx - start_idx + 1;
	int num_dofs = motion.GetNumDof();
	out_motion.Init(curr_num_frames, num_dofs);

	const auto& joint_mat = kin_char.GetJointMat();
	Eigen::VectorXd frame0 = motion.GetFrame(start_idx);
	double heading = cKinTree::CalcHeading(joint_mat, frame0);
	tQuaternion rot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), -heading);
	tVector trans = -cKinTree::GetRootPos(joint_mat, frame0);
	trans[1] = 0;

	double start_time = motion.GetFrameTime(start_idx);
	for (int f = 0; f < curr_num_frames; ++f)
	{
		int idx = start_idx + f;
		Eigen::VectorXd curr_frame = motion.GetFrame(idx);
		tQuaternion root_rot = cKinTree::GetRootRot(joint_mat, curr_frame);
		tVector root_pos = cKinTree::GetRootPos(joint_mat, curr_frame);

		root_rot = rot * root_rot;
		root_pos += trans;
		root_pos = cMathUtil::QuatRotVec(rot, root_pos);

		cKinTree::SetRootRot(joint_mat, root_rot, curr_frame);
		cKinTree::SetRootPos(joint_mat, root_pos, curr_frame);

		double curr_time = motion.GetFrameTime(idx);

		out_motion.SetFrame(f, curr_frame);
		out_motion.SetFrameTime(f, curr_time - start_time);
	}
}

void cScenarioSegmentMotion::MirrorMotionStance(const cKinCharacter& kin_char, cMotion& out_motion) const
{
	const auto& joint_mat = kin_char.GetJointMat();
	int num_frames = out_motion.GetNumFrames();
	for (int f = 0; f < num_frames; ++f)
	{
		Eigen::VectorXd curr_frame = out_motion.GetFrame(f);
		MirrorPoseStance(joint_mat, curr_frame);
		out_motion.SetFrame(f, curr_frame);
	}
}

void cScenarioSegmentMotion::MirrorPoseStance(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose) const
{
	cKinTree::MirrorPoseStance(joint_mat, mLeftJoints, mRightJoints, out_pose);
}