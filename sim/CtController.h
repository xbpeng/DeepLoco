#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/CtCtrlUtil.h"

#define ENABLE_MAX_STATE

class cCtController : public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCtController();
	virtual ~cCtController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual bool NewActionUpdate() const;

	virtual void SetUpdatePeriod(double period);
	virtual double GetUpdatePeriod() const;
	virtual void SetActionParams(const Eigen::VectorXd& params);

	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau);
	
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	virtual void GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual int GetNumActions() const;
	virtual int GetNumParams() const;
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;

	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionExpCovar(Eigen::VectorXd& out_covar) const;
	virtual void ForceActionUpdate();

	virtual int GetNumGroundSamples() const;
	virtual void SetGroundSampleRes(int sample_res);

protected:
	
	double mUpdatePeriod;
	double mUpdateCounter;
	double mViewDistMin;
    int mGroundSampleRes;

	Eigen::VectorXd mActionBoundMin;
	Eigen::VectorXd mActionBoundMax;

	virtual void ResetParams();
	virtual int GetPosFeatureDim() const;
	virtual int GetRotFeatureDim() const;

	virtual bool LoadControllers(const std::string& file);
	virtual bool IsCurrActionCyclic() const;

	virtual void InitCurrAction();
	virtual bool ShouldExplore() const;
	virtual tVector CalcGroundSamplePos(int s) const;
	virtual tVector CalcGroundSampleOrigin() const;
	virtual tVector CalcGroundSampleSize() const;
	virtual int GetGroundSampleRes() const;
	virtual double GetMaxViewDist() const;
	virtual double GetMinViewDist() const;

	virtual void SetupActionBounds();

	virtual void UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateAction();
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action);
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action);

	virtual void PostProcessAction(tAction& out_action) const;
	virtual void RecordVal();
	virtual void ApplyExpNoise(tAction& out_action);

	virtual void ApplyAction(int action_id);
	virtual void ApplyAction(const tAction& action);

	virtual void BuildBaseAction(int action_id, tAction& out_action) const;

	virtual int GetPoseFeatureSize() const;
	virtual int GetVelFeatureSize() const;
	virtual void BuildPoliStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoliStateVel(Eigen::VectorXd& out_vel) const;

	virtual void BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual cWorld::eSimMode GetSimMode() const;
	virtual bool Is3D() const;

	virtual bool FlipStance() const;
	virtual int RetargetJointID(int joint_id) const;
};
