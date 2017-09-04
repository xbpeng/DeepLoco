#include "DrawScenarioPoliEval.h"
#include "scenarios/ScenarioPoliEval.h"

cDrawScenarioPoliEval::cDrawScenarioPoliEval(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
	mDrawPoliInfo = true;
}

cDrawScenarioPoliEval::~cDrawScenarioPoliEval()
{
}

void cDrawScenarioPoliEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimChar::Keyboard(key, x, y);

	if (key >= '1' && key <= '9')
	{
		CommandAction(key - '1');
	}
	else
	{
		const std::shared_ptr<cCharController>& char_c = this->GetScene()->GetCharacter()->GetController();
		if (char_c != nullptr)
		{
			Eigen::VectorXd state;
			char_c->getInternalState(state);
			// state.resize(1);
			// const std::shared_ptr<cDogControllerCaclaDQ>& char_c_dogdq = dynamic_cast<const std::shared_ptr<cDogControllerCaclaDQ>&>(char_c);
			switch (key)
			{
			case 'a':
				ToggleRecordActions();
				break;
			case 'o':
				std::cout << "Internal State: " << state(0) << std::endl;
				state(0) = state(0) + 0.2;
				char_c->updateInternalState(state);
				break;
			case 'O':
				std::cout << "Internal State: " << state(0) << std::endl;
				state(0) = state(0) - 0.2;
				char_c->updateInternalState(state);
				break;
			case 't':
				ToggleRecordCtrlForce();
				break;
			default:
				break;
			}
		}
	}
}

std::string cDrawScenarioPoliEval::BuildTextInfoStr() const
{
	const auto& character = mScene->GetCharacter();
	tVector com = character->CalcCOM();
	tVector com_vel = character->CalcCOMVel();
	char buffer[128];

	std::string info_str = cDrawScenarioSimChar::BuildTextInfoStr();

	auto poli_eval = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	int num_samples = poli_eval->GetNumEpisodes();
	//double avg_dist = poli_eval->GetAvgDist();
	//sprintf(buffer, "Avg Dist: %.2f (%i)\n", avg_dist, num_samples);
	//info_str += buffer;

	double total_reward = poli_eval->GetCurrCumulativeReward();
	sprintf(buffer, "Total Reward: %.2f\n", total_reward);
	info_str += buffer;

	double avg_reward = poli_eval->CalcAvgReward();
	sprintf(buffer, "Avg Reward: %.2f (%i)\n", avg_reward, num_samples);
	info_str += buffer;

	return info_str;
}

void cDrawScenarioPoliEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioPoliEval>(new cScenarioPoliEval());
}

void cDrawScenarioPoliEval::CommandAction(int a)
{
	std::shared_ptr<cScenarioSimChar> sim_scene = std::dynamic_pointer_cast<cScenarioSimChar>(mScene);
	const auto& character = sim_scene->GetCharacter();
	const std::shared_ptr<cCharController>& ctrl = character->GetController();
	ctrl->CommandAction(a);
}

void cDrawScenarioPoliEval::ToggleRecordActions()
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	bool enable = eval_scene->EnableRecordActions();
	eval_scene->EnableRecordActions(!enable);

	enable = eval_scene->EnableRecordActions();
	if (enable)
	{
		printf("Record Actions Enabled\n");
	}
	else
	{
		printf("Record Actions Disabled\n");
	}
}

void cDrawScenarioPoliEval::ToggleRecordCtrlForce()
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	bool enable = eval_scene->EnableRecordCtrlForce();
	eval_scene->EnableRecordCtrlForce(!enable);

	enable = eval_scene->EnableRecordCtrlForce();
	if (enable)
	{
		printf("Record Torques Enabled\n");
	}
	else
	{
		printf("Record Torques Disabled\n");
	}
}