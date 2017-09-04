#pragma once

#include "OptScenarioHikeEval.h"

class cOptScenarioSoccerEval : public cOptScenarioHikeEval
{
public:
	cOptScenarioSoccerEval();
	virtual ~cOptScenarioSoccerEval();

protected:
	
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
};