#pragma once

#include "Controller.h"
#include "anim/Motion.h"

class cCharController : public cController
{
public:
	struct tExpParams
	{
		double mRate;
		double mTemp;
		double mBaseActionRate;
		double mNoise;
		double mNoiseInternal;

		tExpParams();
	};

	virtual ~cCharController();

	virtual void Reset();
	virtual void Update(double time_step);
	virtual int GetState() const;
	virtual double GetPhase() const;
	virtual void SetPhase(double phase);
	virtual int GetNumStates() const;
	virtual double CalcNormPhase() const;

	virtual void TransitionState(int state);
	virtual void TransitionState(int state, double phase);
	virtual bool IsNewCycle() const;
	virtual bool NewActionUpdate() const;

	virtual void CommandAction(int action_id);
	virtual void CommandRandAction();
	virtual int GetDefaultAction() const;
	virtual void SetDefaultAction(int action_id);
	virtual int GetNumActions() const;
	virtual int GetCurrActionID() const;

	virtual void EnableExp(bool enable);
	virtual bool EnabledExplore() const;
	virtual const tExpParams& GetExpParams() const;
	virtual void SetExpParams(const tExpParams& params);

	virtual double GetViewDist() const;
	virtual void SetViewDist(double dist);
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;

	virtual void BuildNormPose(Eigen::VectorXd& pose) const;

	virtual void BuildFromMotion(int ctrl_params_idx, const cMotion& motion);
	virtual void BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const;
	virtual void SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params);
	virtual void BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const;

	virtual int GetNumGroundSamples() const;
	virtual tVector GetGroundSample(int s) const;
	virtual tMatrix GetGroundSampleTrans() const;

	virtual void getInternalState(Eigen::VectorXd&) const;
	virtual void updateInternalState(Eigen::VectorXd& state);

	virtual void HandlePoseReset();
	virtual void HandleVelReset();

	virtual std::string BuildTextInfoStr() const;

protected:
	int mState;
	double mPhase;

	bool mEnableExp;
	tExpParams mExpParams;

	double mViewDist;

	cCharController();

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	virtual void GetVisCharacterFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisTerrainFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;
#endif
};
