#include "OptScenarioSoccerEval.h"
#include "scenarios/ScenarioSoccerEval.h"

cOptScenarioSoccerEval::cOptScenarioSoccerEval()
{
}

cOptScenarioSoccerEval::~cOptScenarioSoccerEval()
{
}

void cOptScenarioSoccerEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioSoccerEval>(new cScenarioSoccerEval());
	out_scene = eval_scene;
}