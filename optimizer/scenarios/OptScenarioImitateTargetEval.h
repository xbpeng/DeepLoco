#pragma once

#include "OptScenarioImitateEval.h"

class cOptScenarioImitateTargetEval : public cOptScenarioImitateEval
{
public:
	cOptScenarioImitateTargetEval();
	virtual ~cOptScenarioImitateTargetEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};