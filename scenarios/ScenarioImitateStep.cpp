#include "ScenarioImitateStep.h"
#include "ScenarioExpImitateStep.h"

cScenarioImitateStep::cScenarioImitateStep()
{
}

cScenarioImitateStep::~cScenarioImitateStep()
{
}

std::string cScenarioImitateStep::GetName() const
{
	return "Train Imitate Step";
}

void cScenarioImitateStep::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpImitateStep>(new cScenarioExpImitateStep());
}