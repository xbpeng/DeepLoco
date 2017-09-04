#include "scenarios/DrawScenarioHikeEval.h"
#include "scenarios/ScenarioHikeEval.h"

cDrawScenarioHikeEval::cDrawScenarioHikeEval(cCamera& cam)
	: cDrawScenarioImitateStepEval(cam)
{
}

cDrawScenarioHikeEval::~cDrawScenarioHikeEval()
{
}

void cDrawScenarioHikeEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioHikeEval>(new cScenarioHikeEval());
}

void cDrawScenarioHikeEval::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->EnableRandTargetPos(true);
}