#pragma once

#include "OptScenarioImitateEval.h"

class cOptScenarioImitateStepEval : public cOptScenarioImitateEval
{
public:
	cOptScenarioImitateStepEval();
	virtual ~cOptScenarioImitateStepEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};