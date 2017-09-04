#include "DrawScenarioImitateTarget.h"
#include "scenarios/ScenarioImitateTarget.h"
#include "scenarios/ScenarioExpImitateTarget.h"
#include "sim/GroundVar2D.h"
#include "sim/GroundVar3D.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"

cDrawScenarioImitateTarget::cDrawScenarioImitateTarget(cCamera& cam)
	: cDrawScenarioImitate(cam)
{
}

cDrawScenarioImitateTarget::~cDrawScenarioImitateTarget()
{
}

void cDrawScenarioImitateTarget::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateTarget>(new cScenarioImitateTarget());
}

void cDrawScenarioImitateTarget::DrawMisc() const
{
	cDrawScenarioImitate::DrawMisc();
	DrawTargetPos();
}

void cDrawScenarioImitateTarget::DrawTargetPos() const
{
	const double r = 0.1;
	const double line_h = 10;
	const double line_w = 3;
	const tVector col = tVector(1, 0, 0, 0.5);

	auto scene = std::dynamic_pointer_cast<cScenarioExpImitateTarget>(mScene);
	const tVector& target_pos = scene->GetTargetPos();
	double reset_dist = scene->GetTargetResetDist();

	cDrawUtil::PushMatrix();

	cDrawUtil::Translate(target_pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);

	cDrawUtil::Translate(tVector(0, 0.1, 0, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);

	cDrawUtil::PopMatrix();

	cDrawUtil::SetLineWidth(line_w);
	cDrawUtil::DrawLine(target_pos, target_pos + tVector(0, line_h, 0, 0));
	cDrawUtil::SetLineWidth(0.5 * line_w);

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(target_pos + tVector(0, 0.1, 0, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);
	cDrawUtil::PopMatrix();
}

void cDrawScenarioImitateTarget::HandleRayTest(const cWorld::tRayTestResult& result)
{
	cDrawScenarioImitate::HandleRayTest(result);
	if (result.mObj != nullptr)
	{
		bool is_ground = IsGround(result.mObj);
		if (is_ground)
		{
			SetTargetPos(result.mHitPos);
		}
	}
}

void cDrawScenarioImitateTarget::SetTargetPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpImitateTarget>(mScene);
	scene->SetTargetPos(pos);
}

bool cDrawScenarioImitateTarget::IsGround(const cSimObj* obj) const
{
	const auto* ground = dynamic_cast<const cGround*>(obj);
	const auto* seg2d = dynamic_cast<const cGroundVar2D::tSegment*>(obj);
	const auto* slab3d = dynamic_cast<const cGroundVar3D::tSlab*>(obj);
	return (ground != nullptr) || (seg2d != nullptr) || (slab3d != nullptr);
}