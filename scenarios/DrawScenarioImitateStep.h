#pragma once
#include "scenarios/DrawScenarioImitateTarget.h"
#include "sim/BipedStepController3D.h"

class cDrawScenarioImitateStep: public cDrawScenarioImitateTarget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateStep(cCamera& cam);
	virtual ~cDrawScenarioImitateStep();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void DrawMisc() const;
	virtual void DrawStepPlan() const;
	virtual void DrawStepPos(const tVector& pos, const tVector& col) const;
	virtual void DrawRootHeading(const cBipedStepController3D::tStepPlan& step_plan, const tVector& col) const;
};