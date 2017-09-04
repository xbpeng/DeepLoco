#pragma once

#include <vector>
#include "scenarios/Scenario.h"
#include "anim/KinCharacter.h"
#include "util/MathUtil.h"

class cScenarioSegmentMotion : public cScenario
{
public:

	cScenarioSegmentMotion();
	virtual ~cScenarioSegmentMotion();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Run();

	virtual std::string GetName() const;

protected:

	enum eMirrorClip
	{
		eMirrorClipNone,
		eMirrorClipOdd,
		eMirrorClipEven,
		eMirrorClipMax
	};

	std::string mCharFile;
	std::string mMotionFile;
	std::string mCutAnnotationFile;
	std::string mOutputPath;

	cKinCharacter mChar;
	std::vector<int> mLeftJoints;
	std::vector<int> mRightJoints;
	eMirrorClip mMirrorClipMode;

	virtual void ParseMirrorClip(const std::string& str, eMirrorClip& out_mirror) const;

	virtual bool BuildCharacter(const std::string& char_file, const std::string& motion_file, 
								cKinCharacter& out_char) const;
	
	virtual void SegmentMotion(const std::string& out_path);
	virtual bool LoadCutAnnotation(const std::string& cut_file, std::vector<double>& out_data) const;
	virtual void FindCutFrames(const cMotion& motion, const std::vector<double>& cut_times, std::vector<int>& out_frames);
	virtual void CutMotion(const cKinCharacter& kin_char, std::vector<int>& cut_indices, const std::string& motion_file, const std::string& out_path);
	virtual void ExtractMotionClip(const cKinCharacter& kin_char, int start_idx, int end_idx, cMotion& out_motion) const;

	virtual void MirrorMotionStance(const cKinCharacter& kin_char, cMotion& out_motion) const;
	virtual void MirrorPoseStance(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose) const;
};