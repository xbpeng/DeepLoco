#pragma once

#include "scenarios/ScenarioSimChar.h"
#include "scenarios/ScenarioExpImitateStep.h"

class cScenarioSampleAction : public cScenario
{
public:
	cScenarioSampleAction();
	virtual ~cScenarioSampleAction();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Clear();
	virtual void Run();

	virtual void SetTimeStep(double time_step);

	virtual std::string GetName() const;

protected:

	typedef cScenarioExpImitateStep tTarScene;

	std::shared_ptr<cArgParser> mArgParser;
	std::unique_ptr<cScenarioSimChar> mScene;
	std::string mOutputPath;

	double mTimeStep;
	double mSceneWarmupTime;
	int mNumActionSamples;

	virtual void BuildScene(std::unique_ptr<cScenarioSimChar>& out_scene);
	virtual void WarmupScene(double warmup_time);
	virtual void SampleActions(int num_samples);

	virtual bool HasOutputFile() const;
};