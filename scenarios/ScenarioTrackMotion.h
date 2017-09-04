#pragma once

#include "scenarios/ScenarioSimChar.h"
#include "anim/KinCharacter.h"

class cScenarioTrackMotion : public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrackMotion();
	virtual ~cScenarioTrackMotion();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void InitTime(double time);

	virtual const cKinCharacter& GetKinCharacter() const;

	virtual double CalcPosePosErr() const;
	virtual double CalcPoseThetaErr() const;
	virtual double CalcPoseThetaRelErr() const;
	virtual double CalcVelErr() const;
	virtual tVector CalcRootPosErr() const;
	virtual double CalcEffort() const;
	virtual double CalcTargetVelErr() const;

	virtual int GetTargetCtrlID() const;
	virtual const std::vector<int>& GetTargetActions() const;

	virtual std::string GetName() const;

protected:
	std::string mMotionFile;
	std::string mBVHMotionFile;
	bool mBuildCtrlFromPose;
	double mTargetVelX;

	int mTargetCtrlID;
	std::vector<int> mTargetActions;

	cKinCharacter mKinChar;

	virtual bool BuildCharacter();
	virtual tVector GetDefaultCharPos() const;
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual void SetupInitState();

	virtual void BuildCtrlParamsFromPose(int target_ctrl_id, std::shared_ptr<cCharController>& out_ctrl);
	virtual void SyncCharacters();
	virtual bool IsNewCycle() const;

	virtual void UpdateKinChar(double time_step);
	
	virtual void CalcSyncKinPose(Eigen::VectorXd& out_pose) const;
	virtual void CommandAction(int action_id);
};