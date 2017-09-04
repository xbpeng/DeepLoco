#pragma once

#include <string>
#include <mutex>
#include "scenarios/ScenarioPoliEval.h"

class cOptScenarioPoliEval : public cScenario
{
public:
	cOptScenarioPoliEval();
	virtual ~cOptScenarioPoliEval();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Run();
	virtual void SetPoolSize(int size);
	virtual void SetTimeStep(double time_step);
	virtual void ResetRecord();

	virtual std::string GetName() const;

protected:
	struct tEvalParams
	{
		int mMaxEpisodes;
		int mMaxCycles;

		tEvalParams();
	};

	std::shared_ptr<cArgParser> mArgParser;
	std::vector<std::shared_ptr<cScenarioPoliEval>> mEvalPool;
	
	int mPoolSize;
	double mTimeStep;
	int mMaxEpisodes;
	int mMaxCycleCount;
	int mEpisodeCount;
	int mCycleCount;
	double mAvgDist;

	std::mutex mUpdateMutex;

	std::string mOutputFile;
	std::string mRewardOutputFile;

	virtual void BuildScenePool();
	virtual void BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const;
	virtual int GetPoolSize() const;
	virtual int CalcHelperMaxEpisode(int id) const;
	virtual int CalcHelperMaxCycles(int id) const;

	virtual void EvalHelper(tEvalParams eval_params, std::shared_ptr<cScenarioPoliEval> eval);
	virtual void UpdateRecord(double avg_dist, int num_episodes, int num_cycles);

	virtual void OutputResults() const;
	virtual void OutputDistLog(const std::string& out_file) const;
	virtual void OutputRewardLog(const std::string& out_file) const;
};