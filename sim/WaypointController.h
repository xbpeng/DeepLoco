#pragma once

#include "sim/CtTargetController.h"
#include "sim/BipedStepController3D.h"

class cWaypointController : public virtual cCtTargetController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	enum eActionParam
	{
		eActionParamStepX0,
		eActionParamStepZ0,
		eActionParamStepX1,
		eActionParamStepZ1,
		eActionParamRootHeading,
		eActionParamMax
	};
	typedef Eigen::Matrix<double, eActionParamMax, 1> tActionParams;

	cWaypointController();
	virtual ~cWaypointController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual void SetGround(std::shared_ptr<cGround> ground);
	virtual void SetInitStepLen(double step_len);
	virtual void SetLLC(const std::shared_ptr<cBipedStepController3D>& ctrl);
	virtual const std::shared_ptr<cBipedStepController3D>& GetLLC() const;
	
	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;
	virtual void EnableExp(bool enable);
	virtual const cBipedStepController3D::tStepPlan& GetStepPlan() const;
	virtual void SetTime(double time);

	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void EnableSymmetricStep(double enable);

protected:

	std::shared_ptr<cBipedStepController3D> mLLC;
	cBipedStepController3D::tStepPlan mStepPlan;
	double mCurrTimeStep;
	bool mNewActionUpdate;

	double mInitStepLen;
	bool mSymmetricStep;
	const std::vector<int>* mFlipJointOrder;

	virtual void ResetParams();

	virtual void SetupActionBounds();
	virtual void PostProcessAction(tAction& out_action) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateNewActionUpdate(double time_step);
	virtual bool NewActionUpdate() const;
	virtual void DecideAction(tAction& out_action);
	virtual void ApplyAction(const tAction& action);

	virtual int GetGroundSampleRes() const;

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual int GetPhaseStateOffset() const;
	virtual int GetPhaseStateSize() const;
	virtual void BuildPhaseState(Eigen::VectorXd& out_state) const;

	virtual void BuildStepPlan(cBipedStepController3D::tStepPlan& out_plan) const;
	virtual cBipedStepController3D::eStance GetStance() const;

	virtual bool EnableSymStep() const;
	virtual int RetargetJointID(int joint_id) const;
	virtual bool FlipStance() const;
};