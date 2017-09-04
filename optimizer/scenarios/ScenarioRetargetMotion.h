#pragma once

#include "scenarios/Scenario.h"
#include "anim/KinCharacter.h"

class cScenarioRetargetMotion : public cScenario
{
public:

	cScenarioRetargetMotion();
	virtual ~cScenarioRetargetMotion();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Clear();
	virtual void Run();

	virtual std::string GetName() const;

protected:

	std::string mSrcCharFile;
	std::string mDstCharFile;
	std::vector<std::string> mMotionFiles;
	std::string mOutputPath;

	cKinCharacter mSrcChar;
	cKinCharacter mDstChar;

	std::vector<int> mRetargetSrcJoints;
	std::vector<int> mRetargetDstJoints;

	virtual bool BuildCharacter(const std::string& char_file, cKinCharacter& out_char) const;
	virtual void RetargetMotions(const std::vector<std::string>& motion_files, cKinCharacter& src_char, cKinCharacter& dst_char);
	virtual void RetargetPose(const cKinCharacter& src_char, cKinCharacter& dst_char, Eigen::VectorXd& out_pose) const;
	virtual void ConvertJointPose(const cKinCharacter& src_char, int src_j, const cKinCharacter& dst_char, int dst_j, Eigen::VectorXd& out_dst_joint_pose) const;
};