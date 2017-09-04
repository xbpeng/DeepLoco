#pragma once

#include "sim/CtController.h"

//#define ENABLE_COS_PHASE
#define ENABLE_PHASE_STATE_BINS

class cCtPhaseController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPhaseController();
	virtual ~cCtPhaseController();

	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau);

	virtual void SetCycleDur(double dur);
	virtual double GetCycleDur() const;
	virtual int GetPoliStateSize() const;
	virtual void SetTime(double time);
	virtual double GetPhase() const;

	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;

protected:

	double mCycleDur;

	virtual int GetPhaseStateOffset() const;
	virtual int GetPhaseStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildPhaseState(Eigen::VectorXd& out_state) const;

#if defined(ENABLE_PHASE_STATE_BINS)
	virtual int GetNumPhaseBins() const;
#endif
};