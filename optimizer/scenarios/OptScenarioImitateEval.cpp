#include "OptScenarioImitateEval.h"
#include "scenarios/ScenarioImitateEval.h"
#include "util/FileUtil.h"

cOptScenarioImitateEval::cOptScenarioImitateEval()
{
	mOutputPoseErrFile = "";
}

cOptScenarioImitateEval::~cOptScenarioImitateEval()
{
}

void cOptScenarioImitateEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cOptScenarioPoliEval::ParseArgs(parser);
	parser->ParseString("pose_err_file", mOutputPoseErrFile);
}

void cOptScenarioImitateEval::BuildScene(int id, std::shared_ptr<cScenarioPoliEval>& out_scene) const
{
	auto eval_scene = std::shared_ptr<cScenarioImitateEval>(new cScenarioImitateEval());

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

void cOptScenarioImitateEval::OutputResults() const
{
	cOptScenarioPoliEval::OutputResults();

	if (mOutputPoseErrFile != "")
	{
		OutputPoseErr(mOutputPoseErrFile);
	}
}

void cOptScenarioImitateEval::OutputPoseErr(const std::string& out_file) const
{
	double tau_err = 0;
	int total_count = 0;
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		double curr_err = 0;
		int curr_count = 0;
		auto imitate_scene = std::dynamic_pointer_cast<cScenarioImitateEval>(mEvalPool[i]);
		imitate_scene->GetPoseErrResult(curr_err, curr_count);

		tau_err = cMathUtil::AddAverage(tau_err, total_count, curr_err, curr_count);
		total_count += curr_count;
	}

	if (total_count > 0)
	{
		std::string str = std::to_string(tau_err);
		str += "\n";
		bool succ = cFileUtil::AppendText(str, out_file);

		if (!succ)
		{
			printf("Failed to output results to %s\n", out_file.c_str());
		}
	}

}