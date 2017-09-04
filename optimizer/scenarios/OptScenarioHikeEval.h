#pragma once

#include "OptScenarioImitateStepEval.h"

class cOptScenarioHikeEval : public cOptScenarioImitateStepEval
{
public:
	cOptScenarioHikeEval();
	virtual ~cOptScenarioHikeEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};