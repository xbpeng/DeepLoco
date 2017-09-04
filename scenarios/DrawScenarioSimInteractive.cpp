#include "DrawScenarioSimInteractive.h"
#include "render/DrawUtil.h"
#include "render/DrawPerturb.h"

const tVector gCamFocus0 = tVector(0, 0.75, 0, 0);

cDrawScenarioSimInteractive::cDrawScenarioSimInteractive(cCamera& cam)
	: cDrawScenarioTerrainRL(cam)
{
	cam.TranslateFocus(gCamFocus0);
	ResetUI();
}

cDrawScenarioSimInteractive::~cDrawScenarioSimInteractive()
{
}

void cDrawScenarioSimInteractive::Init()
{
	cDrawScenarioTerrainRL::Init();
	ResetUI();
}

void cDrawScenarioSimInteractive::Reset()
{
	cDrawScenarioTerrainRL::Reset();
	ResetUI();
}

void cDrawScenarioSimInteractive::Clear()
{
	cDrawScenarioTerrainRL::Clear();
	ResetUI();
}

void cDrawScenarioSimInteractive::UpdateScene(double time_elapsed)
{
	ApplyUIForce(time_elapsed);
}

void cDrawScenarioSimInteractive::MouseClick(int button, int state, double x, double y)
{
	const double ray_max_dist = 1000;
 	cDrawScenarioTerrainRL::MouseClick(button, state, x, y);
	
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mClickScreenPos = tVector(x, y, 0, 0);
			mDragScreenPos = mClickScreenPos;
			tVector start = mCam.ScreenToWorldPos(mClickScreenPos);
			tVector dir = mCam.GetRayCastDir(start);
			tVector end = start + dir * ray_max_dist;

			cWorld::tRayTestResult raytest_result;
			RayTest(start, end, raytest_result);
			HandleRayTest(raytest_result);
		}
		else if (state == GLUT_UP)
		{
			ResetUI();
		}
	}
}

void cDrawScenarioSimInteractive::MouseMove(double x, double y)
{
	cDrawScenarioTerrainRL::MouseMove(x, y);

	if (ObjectSelected())
	{
		mDragScreenPos = tVector(x, y, 0, 0);
	}
}

void cDrawScenarioSimInteractive::ResetUI()
{
	mClickScreenPos.setZero();
	mDragScreenPos.setZero();
	mSelectObjLocalPos.setZero();
	mSelectedObj = nullptr;
}


tVector cDrawScenarioSimInteractive::GetDefaultCamFocus() const
{
	return gCamFocus0;
}

void cDrawScenarioSimInteractive::RayTest(const tVector& start, const tVector& end, cWorld::tRayTestResult& out_result)
{
	return GetScene()->RayTest(start, end, out_result);
}

bool cDrawScenarioSimInteractive::ObjectSelected() const
{
	return mSelectedObj != nullptr;
}

void cDrawScenarioSimInteractive::HandleRayTest(const cWorld::tRayTestResult& result)
{
	if (result.mObj != nullptr)
	{
		cSimObj::eType obj_type = result.mObj->GetType();
		if (obj_type == cSimObj::eTypeDynamic)
		{
			mSelectedObj = result.mObj;
			if (ObjectSelected())
			{
				mSelectObjLocalPos = mSelectedObj->WorldToLocalPos(result.mHitPos);
			}
		}
	}
}

void cDrawScenarioSimInteractive::ApplyUIForce(double time_step)
{
	if (ObjectSelected())
	{
		const double force_scale = 1 / cDrawPerturb::gForceScale;
		tVector start = mCam.ScreenToWorldPos(mClickScreenPos);
		tVector end = mCam.ScreenToWorldPos(mDragScreenPos);
		start = mCam.ProjectToFocalPlane(start);
		end = mCam.ProjectToFocalPlane(end);

		tVector force = end - start;
		force *= force_scale;

		tPerturb perturb = tPerturb(tPerturb::ePerturbForce, mSelectedObj, mSelectObjLocalPos,
									force, time_step);
		GetScene()->AddPerturb(perturb);
	}
}