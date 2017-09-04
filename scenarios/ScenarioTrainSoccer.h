#pragma once
#include "scenarios/ScenarioTrainHike.h"

class cScenarioTrainSoccer : public cScenarioTrainHike
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainSoccer();
	virtual ~cScenarioTrainSoccer();

	virtual std::string GetName() const;

protected:

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
};