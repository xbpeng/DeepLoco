#pragma once
#include "scenarios/DrawScenarioImitateStepEval.h"

class cDrawScenarioHikeEval : public cDrawScenarioImitateStepEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioHikeEval(cCamera& cam);
	virtual ~cDrawScenarioHikeEval();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void ResetCallback();
};