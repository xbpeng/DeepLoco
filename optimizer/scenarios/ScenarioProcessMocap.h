#pragma once

#include "scenarios/Scenario.h"
#include "util/MathUtil.h"
#include "util/BVHReader.h"

class cScenarioProcessMocap : public cScenario
{
public:

	cScenarioProcessMocap();
	virtual ~cScenarioProcessMocap();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Clear();
	virtual void Run();

	virtual std::string GetName() const;

protected:

	std::vector<std::string> mInputFiles;
	std::string mOutputPath;
	double mTargetFramerate;

	virtual void ConvertMocapFiles(const std::vector<std::string>& input_files, const std::string& output_path);
	virtual void OutputSkeleton(const cBVHReader& reader, const std::string& out_filepath) const;
	virtual void OutputMotion(const cBVHReader& reader, const std::string& out_filepath) const;
};