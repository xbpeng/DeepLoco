#include "DrawScenarioTrainHike.h"
#include "scenarios/ScenarioTrainHike.h"

cDrawScenarioTrainHike::cDrawScenarioTrainHike(cCamera& cam)
	: cDrawScenarioImitateStep(cam)
{
}

cDrawScenarioTrainHike::~cDrawScenarioTrainHike()
{
}

void cDrawScenarioTrainHike::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainHike>(new cScenarioTrainHike());
}