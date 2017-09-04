#include "scenarios/ScenarioSoccerEval.h"

cScenarioSoccerEval::cScenarioSoccerEval() :
					cScenarioExpSoccer(),
					cScenarioHikeEval()
{
}

cScenarioSoccerEval::~cScenarioSoccerEval()
{
}

std::string cScenarioSoccerEval::GetName() const
{
	return "Soccer Evaluation";
}