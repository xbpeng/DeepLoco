#pragma once
#include "DrawScenarioImitate.h"

class cDrawScenarioImitateTarget: public cDrawScenarioImitate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateTarget(cCamera& cam);
	virtual ~cDrawScenarioImitateTarget();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void DrawMisc() const;
	virtual void DrawTargetPos() const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void SetTargetPos(const tVector& pos);

	virtual bool IsGround(const cSimObj* obj) const;
};