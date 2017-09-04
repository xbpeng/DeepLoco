#pragma once
#include "sim/CtPDPhaseController.h"

class cBipedStepController3D : public virtual cCtPDPhaseController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	enum eStance
	{
		eStanceRight,
		eStanceLeft,
		eStanceMax
	};

	struct tStepPlan
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eStance mStance;
		tVector mStepPos0;
		tVector mStepPos1;
		double mRootHeading;

		tStepPlan();
	};

	enum eTaskParam
	{
		eTaskParamStepRightX0,
		eTaskParamStepRightY0,
		eTaskParamStepRightZ0,
		eTaskParamStepLeftX0,
		eTaskParamStepLeftY0,
		eTaskParamStepLeftZ0,
		eTaskParamStepRightX1,
		eTaskParamStepRightY1,
		eTaskParamStepRightZ1,
		eTaskParamStepLeftX1,
		eTaskParamStepLeftY1,
		eTaskParamStepLeftZ1,
		eTaskParamRootHeading,
		eTaskParamMax
	};
	typedef Eigen::Matrix<double, eTaskParamMax, 1> tTaskParams;

	cBipedStepController3D();
	virtual ~cBipedStepController3D();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual const tStepPlan& GetStepPlan() const;
	virtual void SetStepPlan(const tStepPlan& plan);

	virtual eStance PredictNextStance(double time_step) const;
	virtual eStance GetStance() const;
	virtual eStance GetStance(double phase) const;
	
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;

    // Hack to get 3D terrain working, someone should fix the inheritance
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;
	virtual int GetNumGroundSamples() const;
	virtual tVector CalcGroundSamplePos(int s) const;


protected:

	std::vector<int> mEndEffectors;
	tStepPlan mStepPlan;

	virtual void InitEndEffectors();
	virtual int GetNumEndEffectors() const;

	virtual int GetPoliStateSize() const;
	virtual int GetContactStateOffset() const;
	virtual int GetContactStateSize() const;
	virtual int GetTaskStateOffset() const;
	virtual int GetTaskStateSize() const;

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildContactState(Eigen::VectorXd& out_state) const;
	virtual void BuildTaskState(Eigen::VectorXd& out_state) const;

	virtual void EvalNet(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const;
};
