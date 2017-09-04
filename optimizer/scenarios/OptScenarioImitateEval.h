#pragma once

#include "OptScenarioPoliEval.h"

class cOptScenarioImitateEval : public cOptScenarioPoliEval
{
public:
	cOptScenarioImitateEval();
	virtual ~cOptScenarioImitateEval();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:
	std::string mOutputPoseErrFile;

	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
	virtual void OutputResults() const;
	virtual void OutputPoseErr(const std::string& out_file) const;
};