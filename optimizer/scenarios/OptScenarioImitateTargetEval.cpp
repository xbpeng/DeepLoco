#include "OptScenarioImitateTargetEval.h"
#include "scenarios/ScenarioImitateTargetEval.h"

cOptScenarioImitateTargetEval::cOptScenarioImitateTargetEval()
{
}

cOptScenarioImitateTargetEval::~cOptScenarioImitateTargetEval()
{
}

void cOptScenarioImitateTargetEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioImitateTargetEval>(new cScenarioImitateTargetEval());

	int total_episodes = mMaxEpisodes;
	int num_episodes = CalcHelperMaxEpisode(id);
	int episode_offset = 0;
	for (int i = 0; i < id; ++i)
	{
		episode_offset += CalcHelperMaxEpisode(i);
	}

	int pool_size = GetPoolSize();
	double min_phase = static_cast<double>(episode_offset) / total_episodes;
	double max_phase = static_cast<double>(episode_offset + num_episodes) / total_episodes;
	
	eval_scene->SetResetPhaseSamples(num_episodes);
	eval_scene->SetResetPhase(min_phase, max_phase);

	out_scene = eval_scene;
}