#include "ScenarioImitateTarget.h"
#include "ScenarioExpImitateTarget.h"

cScenarioImitateTarget::cScenarioImitateTarget()
{
}

cScenarioImitateTarget::~cScenarioImitateTarget()
{
}

std::string cScenarioImitateTarget::GetName() const
{
	return "Train Imitate Target";
}

void cScenarioImitateTarget::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpImitateTarget>(new cScenarioExpImitateTarget());
}