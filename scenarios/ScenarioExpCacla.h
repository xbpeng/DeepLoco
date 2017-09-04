#pragma once

#include "scenarios/ScenarioExp.h"

class cScenarioExpCacla : virtual public cScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpCacla();
	virtual ~cScenarioExpCacla();

	virtual std::string GetName() const;

protected:
	
	virtual bool IsValidTuple(const tExpTuple& tuple) const;
	virtual bool IsOffPolicyTuple(const tExpTuple& tuple) const;

	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
};