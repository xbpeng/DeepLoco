#pragma once
#include "scenarios/DrawScenarioHikeEval.h"

class cDrawScenarioSoccerEval : public cDrawScenarioHikeEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioSoccerEval(cCamera& cam);
	virtual ~cDrawScenarioSoccerEval();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void HandleRayTestBall(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);
};