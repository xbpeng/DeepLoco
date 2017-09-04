#pragma once

// hack hack hack
// get rid of this ASAP
//#define HACK_SOCCER_LLC

#include "scenarios/ScenarioExpImitateStep.h"

class cScenarioExpHike : virtual public cScenarioExpImitateStep
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eExpMode
	{
		eExpModeHLC,
		eExpModeLLC,
		eTExpModeMax
	};
	
	cScenarioExpHike();
	virtual ~cScenarioExpHike();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void SetBufferSize(int size);
	virtual bool IsLLCTupleBufferFull() const;
	virtual void ResetLLCTupleBuffer();
	virtual const std::vector<tExpTuple>& GetLLCTuples() const;

	virtual const cCharController::tExpParams& GetLLCExpParams() const;
	virtual void SetLLCExpParams(const cCharController::tExpParams& params);
	virtual void EnableExplore(bool enable);

	virtual void SetExpMode(eExpMode mode);
	virtual eExpMode GetExpMode() const;
	virtual const cBipedStepController3D::tStepPlan& GetStepPlan() const;

	virtual std::string GetName() const;

protected:
	
	eExpMode mExpMode;

	cCharController::tExpParams mLLCExpParams;
	int mLLCTupleBufferSize;
	int mLLCTupleCount;
	tExpTuple mCurrLLCTuple;
	std::vector<tExpTuple> mLLCTupleBuffer;
	double mLLCTupleAcceptProb;

	double mTargetSpeed;

	virtual void ResetParams();
	virtual bool EnableUpdateStepPlan() const;
	virtual void ResetKinChar();

	virtual double CalcReward() const;
	virtual std::shared_ptr<cTerrainRLCharController> GetLLC() const;

	virtual void PreSubstepUpdate(double time_step);
	virtual void PostSubstepUpdate(double time_step);
	virtual void HandleNewActionUpdate();
	virtual bool NewLLCActionUpdate() const;
	virtual void HandleNewLLCActionUpdate();
	virtual void HandleFallUpdate();

	virtual void RecordLLCState(Eigen::VectorXd& out_state) const;
	virtual void RecordLLCAction(Eigen::VectorXd& out_action) const;
	
	virtual void RecordLLCFlagsBeg(tExpTuple& out_tuple) const;
	virtual void RecordLLCFlagsEnd(tExpTuple& out_tuple) const;
	virtual void RecordLLCTuple(const tExpTuple& tuple);

	virtual void UpdateLLCRewardBeg(tExpTuple& out_tuple);
	virtual void UpdateLLCReward(tExpTuple& out_tuple, double time_step);
	virtual void UpdateLLCRewardEnd(tExpTuple& out_tuple);
	virtual double CalcLLCReward() const;
	virtual double GetLLCActionLogp() const;

	virtual bool IsValidLLCTuple(const tExpTuple& tuple) const;
	virtual int GetNumLLCWarmupCycles() const;
	virtual void EnableLLCExp(bool enable);

	virtual bool EnableLLCFeedbackReward() const;
	virtual tVector CalcTargetPosDefault();
	virtual int GetTargetPosTrail3dForwardSegs() const;
};