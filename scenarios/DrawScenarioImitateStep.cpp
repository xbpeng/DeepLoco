#include "DrawScenarioImitateStep.h"
#include "scenarios/ScenarioImitateStep.h"
#include "scenarios/ScenarioExpImitateStep.h"
#include "render/DrawUtil.h"

cDrawScenarioImitateStep::cDrawScenarioImitateStep(cCamera& cam)
	: cDrawScenarioImitateTarget(cam)
{
}

cDrawScenarioImitateStep::~cDrawScenarioImitateStep()
{
}

void cDrawScenarioImitateStep::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateStep>(new cScenarioImitateStep());
}

void cDrawScenarioImitateStep::DrawMisc() const
{
	cDrawScenarioImitateTarget::DrawMisc();
	DrawStepPlan();
}

void cDrawScenarioImitateStep::DrawStepPlan() const
{
	const tVector& pos0_col = tVector(0, 1, 0, 0.5);
	const tVector& pos1_col = tVector(0, 0.5, 0, 0.5);
	const tVector& heading_col = tVector(1, 0, 0, 0.5);

	auto step_scene = std::dynamic_pointer_cast<cScenarioExpImitateStep>(mScene);
	const auto& step_plan = step_scene->GetStepPlan();

	DrawStepPos(step_plan.mStepPos0, pos0_col);
	DrawStepPos(step_plan.mStepPos1, pos1_col);
	DrawRootHeading(step_plan, heading_col);
}

void cDrawScenarioImitateStep::DrawStepPos(const tVector& pos, const tVector& col) const
{
	const double r = 0.075;

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);
	cDrawUtil::PopMatrix();
}

void cDrawScenarioImitateStep::DrawRootHeading(const cBipedStepController3D::tStepPlan& step_plan, const tVector& col) const
{
	const tVector offset = tVector(0, 0.02, 0, 0);
	const double arrow_size = 0.2;
	const double arrow_len = 0.5;

	double theta = step_plan.mRootHeading;

	tVector start = 0.5 * (step_plan.mStepPos0 + step_plan.mStepPos1);
	start += offset;
	tVector dir = tVector(std::cos(theta), std::sin(theta), 0, 0);
	tVector end = arrow_len * dir;
	
	cDrawUtil::SetColor(col);
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(start);
	cDrawUtil::Rotate(-0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawArrow2D(tVector::Zero(), end, arrow_size);
	cDrawUtil::PopMatrix();
}