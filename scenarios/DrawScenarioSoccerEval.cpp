#include "scenarios/DrawScenarioSoccerEval.h"
#include "scenarios/ScenarioSoccerEval.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

cDrawScenarioSoccerEval::cDrawScenarioSoccerEval(cCamera& cam)
	: cDrawScenarioHikeEval(cam)
{
}

cDrawScenarioSoccerEval::~cDrawScenarioSoccerEval()
{
}

void cDrawScenarioSoccerEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioSoccerEval>(new cScenarioSoccerEval());
}

void cDrawScenarioSoccerEval::HandleRayTest(const cWorld::tRayTestResult& result)
{
	if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
	{
		HandleRayTestBall(result);
	}
	else
	{
		cDrawScenarioHikeEval::HandleRayTest(result);
	}
}

void cDrawScenarioSoccerEval::HandleRayTestBall(const cWorld::tRayTestResult& result)
{
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeStatic)
		{
			SetBallPos(result.mHitPos);
		}
	}
}

void cDrawScenarioSoccerEval::SetBallPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioSoccerEval>(mScene);
	scene->SetBallPos(pos);
}