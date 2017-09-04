#pragma once
#include "scenarios/ScenarioTrainCacla.h"

class cScenarioImitate : public cScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitate();
	virtual ~cScenarioImitate();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void EnableRandStateReset(bool enable);

	virtual std::string GetName() const;

protected:

	double mInitEpisodeTimeLimMin;
	double mInitEpisodeTimeLimMax;
	double mEpisodeTimeLimMin;
	double mEpisodeTimeLimMax;
	
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;

	virtual void UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void CalcEpisodeTimeLim(int iters, double& out_min, double& out_max) const;
	virtual void PrintLearnerInfo(int exp_id) const;
};