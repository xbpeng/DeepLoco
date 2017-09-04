#include "OptScenarioPoliEval.h"
#include <thread>
#include <chrono>

#include "util/FileUtil.h"

cOptScenarioPoliEval::tEvalParams::tEvalParams()
{
	mMaxEpisodes = std::numeric_limits<int>::max();
	mMaxCycles = std::numeric_limits<int>::max();
}

cOptScenarioPoliEval::cOptScenarioPoliEval()
{
	mTimeStep = 1 / 30.0;
	mMaxEpisodes = std::numeric_limits<int>::max();
	mMaxCycleCount = std::numeric_limits<int>::max();

	ResetRecord();
}

cOptScenarioPoliEval::~cOptScenarioPoliEval()
{
}

void cOptScenarioPoliEval::Init()
{
	cScenario::Init();
	BuildScenePool();
	ResetRecord();
}

void cOptScenarioPoliEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenario::ParseArgs(parser);
	mArgParser = parser;

	parser->ParseInt("poli_eval_max_episodes", mMaxEpisodes);
	parser->ParseInt("poli_eval_max_cycles", mMaxCycleCount);
	parser->ParseString("output_path", mOutputFile);
	parser->ParseString("reward_output_path", mRewardOutputFile);
}

void cOptScenarioPoliEval::Reset()
{
	cScenario::Reset();

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mEvalPool[i]->Reset();
	}

	ResetRecord();
}

void cOptScenarioPoliEval::Clear()
{
	cScenario::Clear();

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mEvalPool[i]->Clear();
	}

	ResetRecord();
}

void cOptScenarioPoliEval::Run()
{
	int num_threads = GetPoolSize();
	std::vector<std::thread> threads(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		int max_episodes = CalcHelperMaxEpisode(i);
		int max_cycles = CalcHelperMaxCycles(i);

		tEvalParams eval_params;
		eval_params.mMaxEpisodes = max_episodes;
		eval_params.mMaxCycles = max_cycles;

		std::thread& curr_thread = threads[i];
		curr_thread = std::thread(&cOptScenarioPoliEval::EvalHelper, this, eval_params, mEvalPool[i]);
	}

	for (int i = 0; i < num_threads; ++i)
	{
		threads[i].join();
	}

	OutputResults();
}

void cOptScenarioPoliEval::SetPoolSize(int size)
{
	mPoolSize = size;
	mPoolSize = std::max(1, mPoolSize);
}

void cOptScenarioPoliEval::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

std::string cOptScenarioPoliEval::GetName() const
{
	if (mEvalPool.size() > 0)
	{
		return mEvalPool[0]->GetName();
	}
	else
	{
		return "No Name";
	}
}

void cOptScenarioPoliEval::ResetRecord()
{
	mEpisodeCount = 0;
	mCycleCount = 0;
	mAvgDist = 0;
}

void cOptScenarioPoliEval::BuildScenePool()
{
	mEvalPool.resize(mPoolSize);

	cRand rand;
	bool valid_seed = false;

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto& curr_scene = mEvalPool[i];
		BuildScene(i, curr_scene);
		curr_scene->ParseArgs(mArgParser);

		if (i == 0)
		{
			valid_seed = curr_scene->HasRandSeed();
			if (valid_seed)
			{
				unsigned long rand_seed = curr_scene->GetRandSeed();
				rand.Seed(rand_seed);
			}
		}
		
		if (valid_seed)
		{
			unsigned long curr_seed = static_cast<unsigned long>(std::abs(rand.RandInt()));
			curr_scene->SetRandSeed(curr_seed);
		}

		curr_scene->Init();
	}
}

void cOptScenarioPoliEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioPoliEval>(new cScenarioPoliEval());
}

int cOptScenarioPoliEval::GetPoolSize() const
{
	return static_cast<int>(mEvalPool.size());
}

int cOptScenarioPoliEval::CalcHelperMaxEpisode(int id) const
{
	int num_threads = GetPoolSize();
	int max_episodes = mMaxEpisodes / num_threads;
	int episode_remainder = mMaxEpisodes % num_threads;
	if (id < episode_remainder)
	{
		++max_episodes;
	}
	return max_episodes;
}

int cOptScenarioPoliEval::CalcHelperMaxCycles(int id) const
{
	int num_threads = GetPoolSize();
	int max_cycles = mMaxCycleCount / num_threads;
	int cycle_remainder = mMaxCycleCount % num_threads;
	if (id < cycle_remainder)
	{
		++max_cycles;
	}
	return max_cycles;
}

void cOptScenarioPoliEval::EvalHelper(tEvalParams eval_params, std::shared_ptr<cScenarioPoliEval> eval)
{
	const int num_episodes_per_update = 10;

	int num_episodes = 0;
	int num_cycles = 0;
	int prev_cycles = num_cycles;

	while (num_episodes < eval_params.mMaxEpisodes
		&& num_cycles < eval_params.mMaxCycles)
	{
		eval->Update(mTimeStep);

		num_cycles = eval->GetNumTotalCycles();
		int curr_num_episodes = eval->GetNumEpisodes();

		if (curr_num_episodes >= num_episodes_per_update
			|| curr_num_episodes + num_episodes >= eval_params.mMaxEpisodes)
		{
			double avg_dist = eval->GetAvgDist();
			int delta_cycles = num_cycles - prev_cycles;
			UpdateRecord(avg_dist, curr_num_episodes, delta_cycles);

			eval->ResetAvgDist();
			num_episodes += curr_num_episodes;
			prev_cycles = num_cycles;
		}
	}
}

void cOptScenarioPoliEval::UpdateRecord(double avg_dist, int num_episodes, int num_cycles)
{
	std::lock_guard<std::mutex> lock_update(mUpdateMutex);

	mAvgDist = cMathUtil::AddAverage(mAvgDist, mEpisodeCount, avg_dist, num_episodes);
	mEpisodeCount += num_episodes;
	mCycleCount += num_cycles;

	printf("\nEpisodes: %i\n", mEpisodeCount);
	printf("Cycles: %i\n", mCycleCount);
	printf("Avg dist: %.5f\n", mAvgDist);
}

void cOptScenarioPoliEval::OutputResults() const
{
	if (mOutputFile != "")
	{
		OutputDistLog(mOutputFile);
	}

	if (mRewardOutputFile != "")
	{
		OutputRewardLog(mRewardOutputFile);
	}
}

void cOptScenarioPoliEval::OutputDistLog(const std::string& out_file) const
{
	std::string str = "";

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		const std::vector<double>& dist_log = mEvalPool[i]->GetDistLog();
		for (size_t j = 0; j < dist_log.size(); ++j)
		{
			double curr_dist = dist_log[j];
			if (str != "")
			{
				str += ", ";
			}
			str += std::to_string(curr_dist);
		}
	}

	str += "\n";

	bool succ = cFileUtil::AppendText(str, out_file);

	if (!succ)
	{
		printf("Failed to output results to %s\n", out_file.c_str());
	}
}

void cOptScenarioPoliEval::OutputRewardLog(const std::string& out_file) const
{
	std::string str = "";

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		if (mEvalPool[i]->EnableRecordReward())
		{
			const std::vector<double>& reward_log = mEvalPool[i]->GetRewardLog();
			for (size_t j = 0; j < reward_log.size(); ++j)
			{
				double curr_reward = reward_log[j];
				if (str != "")
				{
					str += ", ";
				}
				str += std::to_string(curr_reward);
			}
		}
	}

	str += "\n";

	bool succ = cFileUtil::AppendText(str, out_file);

	if (!succ)
	{
		printf("Failed to output results to %s\n", out_file.c_str());
	}
}