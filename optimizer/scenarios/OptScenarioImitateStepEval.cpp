#include "OptScenarioImitateStepEval.h"
#include "scenarios/ScenarioImitateStepEval.h"

cOptScenarioImitateStepEval::cOptScenarioImitateStepEval()
{
}

cOptScenarioImitateStepEval::~cOptScenarioImitateStepEval()
{
}

void cOptScenarioImitateStepEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioImitateStepEval>(new cScenarioImitateStepEval());
	out_scene = eval_scene;
}