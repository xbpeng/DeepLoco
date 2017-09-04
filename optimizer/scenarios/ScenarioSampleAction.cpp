#include "scenarios/ScenarioSampleAction.h"
#include "util/JsonUtil.h"
#include "util/FileUtil.h"

cScenarioSampleAction::cScenarioSampleAction()
{
	mSceneWarmupTime = 0;
	mTimeStep = 1.0 / 30;
	mNumActionSamples = 256;
	mOutputPath = "";
}

cScenarioSampleAction::~cScenarioSampleAction()
{
}

void cScenarioSampleAction::Init()
{
	cScenario::Init();

	BuildScene(mScene);
}

void cScenarioSampleAction::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenario::ParseArgs(parser);
	mArgParser = parser;

	parser->ParseDouble("scene_warmup_time", mSceneWarmupTime);
	parser->ParseString("output_path", mOutputPath);
	parser->ParseInt("num_action_samples", mNumActionSamples);
}


void cScenarioSampleAction::Clear()
{
	cScenario::Clear();
	mScene.reset();
}

void cScenarioSampleAction::Run()
{
	if (HasOutputFile())
	{
		WarmupScene(mSceneWarmupTime);
		SampleActions(mNumActionSamples);
	}
	else
	{
		printf("No output path specified.\n");
		assert(false);
	}
}

void cScenarioSampleAction::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

std::string cScenarioSampleAction::GetName() const
{
	return "Sample Actions";
}

void cScenarioSampleAction::BuildScene(std::unique_ptr<cScenarioSimChar>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioSimChar>(new tTarScene());
	out_scene->ParseArgs(mArgParser);
	out_scene->Init();
}

void cScenarioSampleAction::WarmupScene(double warmup_time)
{
	mScene->Reset();

	bool warmed_up = false;
	do
	{
		mScene->Update(mTimeStep);
		double time = mScene->GetTime();
		warmed_up = (time >= mSceneWarmupTime);
	} while (!warmed_up);
}

void cScenarioSampleAction::SampleActions(int num_samples)
{
	const auto& character = mScene->GetCharacter();
	const auto& ctrl = std::dynamic_pointer_cast<cTerrainRLCharController>(character->GetController());
	assert(ctrl != nullptr);

	cTerrainRLCharController::tAction action;
	cTerrainRLCharController::tAction mean_action = ctrl->GetCurrAction();

	for (int i = 0; i < num_samples; ++i)
	{
		ctrl->SampleAction(action);
		Eigen::VectorXd action_delta = action.mParams - mean_action.mParams;
		std::string action_str = cJsonUtil::BuildVectorString(action_delta);
		action_str += "\n";
		cFileUtil::AppendText(action_str, mOutputPath);
	}
}

bool cScenarioSampleAction::HasOutputFile() const
{
	return mOutputPath != "";
}