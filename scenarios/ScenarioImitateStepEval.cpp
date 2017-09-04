#include "ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExpImitateStep.h"

cScenarioImitateStepEval::cScenarioImitateStepEval() :
	cScenarioExpImitateStep(),
	cScenarioImitateTargetEval()
{
	EnableTargetPos(false);
	EnableRandTargetPos(false);
}

cScenarioImitateStepEval::~cScenarioImitateStepEval()
{
}

std::string cScenarioImitateStepEval::GetName() const
{
	return "Imitate Step Evaluation";
}

bool cScenarioImitateStepEval::HasFallen() const
{
	bool fallen = cScenarioImitateTargetEval::HasFallen();
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	fallen = false;
#endif
	return fallen;
}