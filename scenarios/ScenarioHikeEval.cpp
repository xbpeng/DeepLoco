#include "scenarios/ScenarioHikeEval.h"
#include "scenarios/ScenarioExpHike.h"

cScenarioHikeEval::cScenarioHikeEval() :
	cScenarioExpHike(),
	cScenarioImitateStepEval()
{
	EnableTargetPos(true);
	EnableRandTargetPos(true);
}

cScenarioHikeEval::~cScenarioHikeEval()
{
}

std::string cScenarioHikeEval::GetName() const
{
	return "Hike Evaluation";
}

bool cScenarioHikeEval::EnableLLCFeedbackReward() const
{
	return false;
}