#include "DrawScenarioImitateTargetEval.h"
#include "scenarios/ScenarioImitateTargetEval.h"
#include "sim/Ground.h"
#include "render/DrawUtil.h"

cDrawScenarioImitateTargetEval::cDrawScenarioImitateTargetEval(cCamera& cam)
	: cDrawScenarioImitateEval(cam)
{
}

cDrawScenarioImitateTargetEval::~cDrawScenarioImitateTargetEval()
{
}

void cDrawScenarioImitateTargetEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioImitateEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'n':
		ToggleRandTargetPos();
		break;
	default:
		break;
	}
}

void cDrawScenarioImitateTargetEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateTargetEval>(new cScenarioImitateTargetEval());
}

void cDrawScenarioImitateTargetEval::DrawMisc() const
{
	cDrawScenarioImitateEval::DrawMisc();
	DrawTargetPos();
}

void cDrawScenarioImitateTargetEval::DrawTargetPos() const
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
}

void cDrawScenarioImitateTargetEval::HandleRayTest(const cWorld::tRayTestResult& result)
{
	cDrawScenarioImitateEval::HandleRayTest(result);
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeStatic)
		{
			SetTargetPos(result.mHitPos);
		}
	}
}

void cDrawScenarioImitateTargetEval::SetTargetPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateTargetEval>(mScene);
	scene->SetTargetPos(pos);
}

bool cDrawScenarioImitateTargetEval::EnableRandTargetPos() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateTargetEval>(mScene);
	bool enable = scene->EnabledRandTargetPos();
	return enable;
}

void cDrawScenarioImitateTargetEval::ToggleRandTargetPos()
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateTargetEval>(mScene);
	bool enable = scene->EnabledRandTargetPos();
	scene->EnableRandTargetPos(!enable);

	if (scene->EnabledRandTargetPos())
	{
		printf("Enabled random target position\n");
	}
	else
	{
		printf("Disabled random target position\n");
	}
}