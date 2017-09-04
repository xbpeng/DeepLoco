#pragma once
#include "scenarios/ScenarioImitate.h"

class cScenarioImitateTarget : public cScenarioImitate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioImitateTarget();
	virtual ~cScenarioImitateTarget();

	virtual std::string GetName() const;

protected:

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
};