#pragma once
#include "scenarios/DrawScenarioImitateTargetEval.h"
#include "sim/BipedStepController3D.h"

//#define ENABLE_HACK_RECORD_ACTION_PERTURB
//#define ENABLE_HACK_ADJUST_PERIOD

class cDrawScenarioImitateStepEval : public cDrawScenarioImitateTargetEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateStepEval(cCamera& cam);
	virtual ~cDrawScenarioImitateStepEval();

	virtual void Init();

protected:

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual bool EnableTargetPos() const;
	virtual void SetTargetPos(const tVector& pos);

	virtual void ResetCallback();

	virtual void DrawMisc() const;
	virtual void DrawStepPlan() const;
	virtual void DrawStepPos(const tVector& pos, const tVector& col) const;
	virtual void DrawRootHeading(const cBipedStepController3D::tStepPlan& step_plan, const tVector& col) const;

	virtual tVector GetCamTrackPos() const;

#if defined(ENABLE_HACK_LLC_LERP)
public:
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual std::string BuildTextInfoStr() const;

protected:
	virtual void AdjustHackLerp(double delta);
#endif

#if defined(ENABLE_HACK_RECORD_ACTION_PERTURB)
public:
	virtual void Keyboard(unsigned char key, int x, int y);

protected:
	bool mHackEnableRecordActionPerturb;
	double mHackRecordActionPerturbPhase;

	virtual void HackRecordActionPerturb();
#endif

#if defined(ENABLE_HACK_ADJUST_PERIOD)
public:
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual std::string BuildTextInfoStr() const;

protected:
	virtual void HackAdjustCtrlPeriod(double delta);
#endif
};