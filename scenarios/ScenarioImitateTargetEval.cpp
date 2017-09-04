#include "ScenarioImitateTargetEval.h"
#include "scenarios/ScenarioExpImitateTarget.h"
#include "sim/CtTargetController.h"
#include "sim/CtPhaseController.h"

cScenarioImitateTargetEval::cScenarioImitateTargetEval() :
	cScenarioExpImitateTarget(),
	cScenarioImitateEval()
{
}

cScenarioImitateTargetEval::~cScenarioImitateTargetEval()
{
}

std::string cScenarioImitateTargetEval::GetName() const
{
	return "Imitate Target Evaluation";
}