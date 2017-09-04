#include <memory>
#include <signal.h>

#include "scenarios/ScenarioTrain.h"
#include "scenarios/ScenarioTrainCacla.h"
#include "scenarios/ScenarioImitate.h"
#include "scenarios/ScenarioImitateTarget.h"
#include "scenarios/ScenarioImitateStep.h"
#include "scenarios/ScenarioTrainHike.h"
#include "scenarios/ScenarioTrainSoccer.h"
#include "scenarios/OptScenario.h"
#include "scenarios/OptScenarioPoliEval.h"
#include "scenarios/OptScenarioImitateEval.h"
#include "scenarios/OptScenarioImitateTargetEval.h"
#include "scenarios/OptScenarioImitateStepEval.h"
#include "scenarios/OptScenarioHikeEval.h"
#include "scenarios/OptScenarioSoccerEval.h"
#include "scenarios/ScenarioProcessMocap.h"
#include "scenarios/ScenarioRetargetMotion.h"
#include "scenarios/ScenarioSegmentMotion.h"
#include "scenarios/ScenarioSampleAction.h"
#include "util/ArgParser.h"
#include "util/FileUtil.h"
#include "util/Util.h"

std::shared_ptr<cArgParser> gArgParser = nullptr;
std::shared_ptr<cScenario> gScenario = nullptr;
int gArgc = 0;
char** gArgv = nullptr;
int gNumThreads = 1;

void ParseArgs(int argc, char** argv)
{
	gArgParser = std::shared_ptr<cArgParser>(new cArgParser(argv, argc));

	std::string arg_file = "";
	gArgParser->ParseString("arg_file", arg_file);
	if (arg_file != "")
	{
		// append the args from the file to the ones from the commandline
		// this allows the cmd args to overwrite the file args
		gArgParser->AppendArgs(arg_file);
	}
	gArgParser->ParseInt("num_threads", gNumThreads);
}

void SetupScenario()
{
	std::string scenario_name = "";
	gArgParser->ParseString("scenario", scenario_name);
	if (scenario_name == "imitate_step")
	{
		auto scene = std::shared_ptr<cScenarioImitateStep>(new cScenarioImitateStep());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetExpPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "train_hike")
	{
		auto scene = std::shared_ptr<cScenarioTrainHike>(new cScenarioTrainHike());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetExpPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "train_soccer")
	{
		auto scene = std::shared_ptr<cScenarioTrainSoccer>(new cScenarioTrainSoccer());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetExpPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "poli_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioPoliEval>(new cOptScenarioPoliEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "imitate_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioImitateEval>(new cOptScenarioImitateEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "imitate_target_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioImitateTargetEval>(new cOptScenarioImitateTargetEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "imitate_step_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioImitateStepEval>(new cOptScenarioImitateStepEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "hike_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioHikeEval>(new cOptScenarioHikeEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else if (scenario_name == "soccer_eval")
	{
		auto scene = std::shared_ptr<cOptScenarioSoccerEval>(new cOptScenarioSoccerEval());
		scene->SetTimeStep(cOptScenario::gTimeStep);
		scene->SetPoolSize(gNumThreads);

		gScenario = scene;
	}
	else
	{
		printf("No valid scenario specified\n");
	}

	if (gScenario != NULL)
	{
		gScenario->ParseArgs(gArgParser);
		gScenario->Init();
		printf("Loaded scenario: %s\n", gScenario->GetName().c_str());
	}
}

void RunScene()
{
	if (gScenario != nullptr)
	{
		gScenario->Run();
	}
}

void CleanUp()
{
	gScenario.reset();
	gArgParser->Clear();
}

void SigHandler(int sig)
{
	if (gScenario != nullptr)
	{
		gScenario->Shutdown();
	}
}

/*
void HackTest()
{
	cNeuralNet net0;
	net0.LoadNet("data/policies/biped3d/nets/biped3d_step_dphase_terr_actor_net.prototxt");
	net0.LoadSolver("data/policies/biped3d/nets/biped3d_step_dphase_terr_actor_solver.prototxt");
	//net0.LoadModel("output/test_net_model.h5");
	//net.OutputModel("output/test_net_model.h5");

	cRand rand(0);

	int input_size = net0.GetInputSize();
	int output_size = net0.GetOutputSize();
	int batch_size = net0.GetBatchSize();

	cNeuralNet::tProblem prob;
	prob.mX.resize(batch_size, input_size);
	prob.mY.resize(batch_size, output_size);
	prob.mW.resize(batch_size, output_size);
	prob.mW.setOnes();

	//FILE* f = cFileUtil::OpenFile("output/test_net_results.txt", "w");

	for (int i = 0; i < 10; ++i)
	{
		for (int b = 0; b < batch_size; ++b)
		{
			for (int j = 0; j < input_size; ++j)
			{
				prob.mX(b, j) = rand.RandDouble();
			}
			for (int j = 0; j < output_size; ++j)
			{
				prob.mY(b, j) = rand.RandDouble();
			}
		}

		net0.Train(prob);

		//auto train_net = net0.GetTrainNet();
		//auto train_params = train_net->learnable_params();
		//for (size_t i = 0; i < train_params.size(); ++i)
		{
			//auto train_blob = train_params[i];
			//auto train_diff = train_blob->cpu_data();
			//for (int j = 0; j < train_blob->count(); ++j)
			{
				//double val = train_diff[j];
				//fprintf(f, "%.5f\n", val);
			}
		}
	}

	return;

	cNeuralNet net;
	net.LoadNet("data/policies/biped3d/nets/biped3d_step_dphase_terr_actor_net.prototxt");
	net.CopyModel(net0);

	for (int i = 0; i < 10; ++i)
	{
		Eigen::VectorXd x = Eigen::VectorXd(input_size);
		Eigen::VectorXd y;

		for (int j = 0; j < input_size; ++j)
		{
			x[j] = rand.RandDouble();
		}

		net.Eval(x, y);
		for (int j = 0; j < output_size; ++j)
		{
			//fprintf(f, "%.5f\n", y[j]);
		}
	}
	//cFileUtil::CloseFile(f);
}
*/

int main(int argc, char** argv)
{
	signal(SIGINT, &SigHandler);

	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	//HackTest();

	SetupScenario();
	RunScene();

	CleanUp();
}