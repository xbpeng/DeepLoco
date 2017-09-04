#pragma once
#include "DrawScenarioTrainHike.h"

class cDrawScenarioTrainSoccer: public cDrawScenarioTrainHike
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainSoccer(cCamera& cam);
	virtual ~cDrawScenarioTrainSoccer();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void HandleRayTestBall(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);
};