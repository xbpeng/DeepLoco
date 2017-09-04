#pragma once
#include "DrawScenarioImitateStep.h"

class cDrawScenarioTrainHike: public cDrawScenarioImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainHike(cCamera& cam);
	virtual ~cDrawScenarioTrainHike();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};