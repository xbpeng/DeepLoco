#include "OptScenarioHikeEval.h"
#include "scenarios/ScenarioHikeEval.h"

cOptScenarioHikeEval::cOptScenarioHikeEval()
{
}

cOptScenarioHikeEval::~cOptScenarioHikeEval()
{
}

void cOptScenarioHikeEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioHikeEval>(new cScenarioHikeEval());
	out_scene = eval_scene;
}