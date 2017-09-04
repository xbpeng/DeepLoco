#include "ScenarioImitate.h"
#include "ScenarioExpImitate.h"
#include "learning/CaclaTrainer.h"

cScenarioImitate::cScenarioImitate()
{
	mInitEpisodeTimeLimMin = std::numeric_limits<double>::infinity();
	mInitEpisodeTimeLimMax = std::numeric_limits<double>::infinity();
	mEpisodeTimeLimMin = std::numeric_limits<double>::infinity();
	mEpisodeTimeLimMax = std::numeric_limits<double>::infinity();
}

cScenarioImitate::~cScenarioImitate()
{
}

void cScenarioImitate::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioTrainCacla::ParseArgs(parser);

	parser->ParseDouble("init_episode_time_lim_min", mInitEpisodeTimeLimMin);
	parser->ParseDouble("init_episode_time_lim_max", mInitEpisodeTimeLimMax);
	parser->ParseDouble("episode_time_lim_min", mEpisodeTimeLimMin);
	parser->ParseDouble("episode_time_lim_max", mEpisodeTimeLimMax);
}

void cScenarioImitate::EnableRandStateReset(bool enable)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto scene = std::dynamic_pointer_cast<cScenarioExpImitate>(mExpPool[i]);
		scene->EnableRandStateReset(enable);
	}
}


std::string cScenarioImitate::GetName() const
{
	return "Train Imitate";
}

void cScenarioImitate::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpImitate>(new cScenarioExpImitate());
}

void cScenarioImitate::UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	cScenarioTrainCacla::UpdateExpSceneRates(exp_id, out_exp);

	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();
	double lim_min = 0;
	double lim_max = 0;
	CalcEpisodeTimeLim(iters, lim_min, lim_max);
	out_exp->SetEpisodeTimeLim(lim_min, lim_max);
}

void cScenarioImitate::CalcEpisodeTimeLim(int iters, double& out_min, double& out_max) const
{
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	lerp = std::pow(lerp, 2);
	out_min = (1 - lerp) * mInitEpisodeTimeLimMin + lerp * mEpisodeTimeLimMin;
	out_max = (1 - lerp) * mInitEpisodeTimeLimMax + lerp * mEpisodeTimeLimMax;
}

void cScenarioImitate::PrintLearnerInfo(int exp_id) const
{
	cScenarioTrain::PrintLearnerInfo(exp_id);

	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();

	double lim_min = 0;
	double lim_max = 0;
	mExpPool[exp_id]->GetEpisodeTimeLim(lim_min, lim_max);
	printf("Episode Time Limit: %.5f, %.5f\n", lim_min, lim_max);
}