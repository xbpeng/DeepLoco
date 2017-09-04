#include "scenarios/ScenarioTrainSoccer.h"
#include "scenarios/ScenarioExpSoccer.h"

cScenarioTrainSoccer::cScenarioTrainSoccer()
{
}

cScenarioTrainSoccer::~cScenarioTrainSoccer()
{
}

std::string cScenarioTrainSoccer::GetName() const
{
	return "Train Soccer";
}

void cScenarioTrainSoccer::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpSoccer>(new cScenarioExpSoccer());
}