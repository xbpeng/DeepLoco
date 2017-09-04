#pragma once
#include "scenarios/ScenarioImitateTarget.h"

class cScenarioImitateStep : public cScenarioImitateTarget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateStep();
	virtual ~cScenarioImitateStep();

	virtual std::string GetName() const;

protected:

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
};