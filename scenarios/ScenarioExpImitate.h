#pragma once

#include "scenarios/ScenarioExpCacla.h"
#include "anim/KinCharacter.h"
#include "sim/Ground.h"

class cScenarioExpImitate : virtual public cScenarioExpCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpImitate();
	virtual ~cScenarioExpImitate();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;
	virtual void EnableRandStateReset(bool enable);
	virtual bool EnabledRandStateReset() const;
	virtual bool HasFallen() const;

	virtual std::string GetName() const;

protected:

	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;
	Eigen::VectorXd mJointWeights;
	bool mEnableRandStateReset;

	virtual void CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const;
	virtual void SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const;
	virtual bool BuildKinCharacter(std::shared_ptr<cKinCharacter>& out_char) const;
	virtual void UpdateCharacter(double time_step);
	virtual void UpdateKinChar(double time_step);
	virtual void UpdateTrackController();

	virtual void ResetParams();
	virtual void ResetCharacters();
	virtual void ResetKinChar();
	virtual void SyncCharacters();
	virtual bool EnableSyncChar() const;

	virtual void InitJointWeights();

	virtual int GetNumWarmupCycles() const;
	virtual double CalcReward() const;
	virtual bool EndEpisode() const;

	virtual double CalcRandKinResetTime();
};