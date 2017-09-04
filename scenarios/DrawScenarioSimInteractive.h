#pragma once
#include <memory>

#include "DrawScenarioTerrainRL.h"
#include "ScenarioSimChar.h"

class cDrawScenarioSimInteractive : public cDrawScenarioTerrainRL
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~cDrawScenarioSimInteractive();

	virtual void Init();

	virtual void Reset();
	virtual void Clear();
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

protected:
	cDrawScenarioSimInteractive(cCamera& cam);

	// UI stuff
	tVector mClickScreenPos;
	tVector mDragScreenPos;
	tVector mSelectObjLocalPos;
	cSimObj* mSelectedObj;

	virtual void UpdateScene(double time_elapsed);
	virtual void ResetUI();
	virtual tVector GetDefaultCamFocus() const;

	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const = 0;
	virtual void RayTest(const tVector& start, const tVector& end, cWorld::tRayTestResult& out_result);
	virtual bool ObjectSelected() const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void ApplyUIForce(double time_step);
};