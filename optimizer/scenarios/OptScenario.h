#pragma once

#include <string>

#include "scenarios/Scenario.h"
#include "util/ArgParser.h"
#include "opt/OptFunc.h"
#include "opt/Optimizer.h"

class cOptScenario : public cScenario, public cOptFunc,
	public std::enable_shared_from_this<cOptFunc>
{
public:
	static const double gTimeStep;

	virtual ~cOptScenario();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();

	virtual void Run();

	virtual const std::string& GetOutputPath() const;
	virtual void SetOutputPath(const std::string& path);

	virtual const tPoint& GetSolution() const;
	virtual void OutputSolution(const std::string& path) const;

protected:
	cOptimizer& mOptimizer;
	std::string mOutputPath;
	std::string mIntOutputPath;
	tPoint mSoln;

	cOptScenario(cOptimizer& optimizer);

	virtual void InitOptimizer(cOptimizer& opt);
	virtual cOptFunc::tPoint Optimize(const cOptimizer::tOptParams& opt_params, cOptimizer& opt);
};