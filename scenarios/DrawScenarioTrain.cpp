#include "DrawScenarioTrain.h"

cDrawScenarioTrain::cDrawScenarioTrain(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
}

cDrawScenarioTrain::~cDrawScenarioTrain()
{
}

void cDrawScenarioTrain::Init()
{
	BuildTrainScene(mTrain);
	SetupTrainScene(mTrain, mScene);

	cDrawScenarioSimInteractive::Init();

	mEnableTrace = false;
	InitTracer();
	InitRenderResources();

	BuildGroundDrawMesh();
	mPrevGroundUpdateCount = 0;
}

void cDrawScenarioTrain::Reset()
{
	cDrawScenarioSimInteractive::Reset();
	mTrain->Reset();
}

void cDrawScenarioTrain::Clear()
{
	cDrawScenarioSimInteractive::Clear();
	mTrain->Clear();
}

void cDrawScenarioTrain::Update(double time_elapsed)
{
	cDrawScenarioSimChar::Update(time_elapsed);
}

void cDrawScenarioTrain::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimChar::Keyboard(key, x, y);

	switch (key)
	{
	case 't':
		ToggleTraining();
		break;
	default:
		break;
	}
}

void cDrawScenarioTrain::Shutdown()
{
	mTrain->Shutdown();
}

void cDrawScenarioTrain::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrain>(new cScenarioTrain());
}

void cDrawScenarioTrain::SetupTrainScene(std::shared_ptr<cScenarioTrain>& out_train_scene, std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_train_scene->SetExpPoolSize(1);
	out_train_scene->ParseArgs(mArgParser);
	out_train_scene->Init();

	out_scene = mTrain->GetExpScene(0);
	tCallbackFunc func = std::bind(&cDrawScenarioTrain::ResetCallback, this);
	out_scene->SetResetCallback(func);
}

void cDrawScenarioTrain::ToggleTraining()
{
	mTrain->ToggleTraining();

	bool training = mTrain->TrainingEnabled();
	if (training)
	{
		printf("Training enabled\n");
	}
	else
	{
		printf("Training disabled\n");
	}
}

void cDrawScenarioTrain::UpdateScene(double time_elapsed)
{
	cDrawScenarioSimInteractive::UpdateScene(time_elapsed);
	mTrain->Update(time_elapsed);
}

std::string cDrawScenarioTrain::GetName() const
{
	return mTrain->GetName();
}