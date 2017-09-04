#pragma once
#include "DrawScenarioImitateEval.h"

class cDrawScenarioImitateTargetEval : public cDrawScenarioImitateEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateTargetEval(cCamera& cam);
	virtual ~cDrawScenarioImitateTargetEval();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawMisc() const;
	virtual void DrawTargetPos() const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void SetTargetPos(const tVector& pos);

	virtual bool EnableRandTargetPos() const;
	virtual void ToggleRandTargetPos();
};