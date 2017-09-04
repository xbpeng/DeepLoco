#include "OptScenario.h"

const double cOptScenario::gTimeStep = 1.0 / 30;

cOptScenario::cOptScenario(cOptimizer& optimizer)
	: mOptimizer(optimizer)
{
	mOutputPath = "";
	mIntOutputPath = "";
}

cOptScenario::~cOptScenario()
{
}

void cOptScenario::Init()
{
	cScenario::Init();
}

void cOptScenario::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	parser->ParseString("output_path", mOutputPath);
	parser->ParseString("int_output_path", mIntOutputPath);
}

void cOptScenario::Reset()
{
	cScenario::Reset();
}

void cOptScenario::Run()
{
	cOptimizer::tOptParams opt_params;
	opt_params.mOutputFile = mOutputPath;
	opt_params.mIntOutputPath = mIntOutputPath;

	mSoln = Optimize(opt_params, mOptimizer);
}

const std::string& cOptScenario::GetOutputPath() const
{
	return mOutputPath;
}

void cOptScenario::SetOutputPath(const std::string& path)
{
	mOutputPath = path;
}

const cOptFunc::tPoint& cOptScenario::GetSolution() const
{
	return mSoln;
}

void cOptScenario::OutputSolution(const std::string& path) const
{
	OutputPt(path, mSoln);
}

void cOptScenario::InitOptimizer(cOptimizer& opt)
{
	auto this_ptr = shared_from_this();
	opt.Init(this_ptr);
}

cOptFunc::tPoint cOptScenario::Optimize(const cOptimizer::tOptParams& opt_params, cOptimizer& opt)
{
	InitOptimizer(opt);
	cOptFunc::tPoint result = opt.Optimize(opt_params);
	return result;
}