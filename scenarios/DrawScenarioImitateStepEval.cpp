#include "scenarios/DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawUtil.h"
#include "util/FileUtil.h"

cDrawScenarioImitateStepEval::cDrawScenarioImitateStepEval(cCamera& cam)
	: cDrawScenarioImitateTargetEval(cam)
{
#if defined(ENABLE_HACK_RECORD_ACTION_PERTURB)
	mHackEnableRecordActionPerturb = false;
	mHackRecordActionPerturbPhase = 0;
#endif
}

cDrawScenarioImitateStepEval::~cDrawScenarioImitateStepEval()
{
}

void cDrawScenarioImitateStepEval::Init()
{
	cDrawScenarioImitateTargetEval::Init();
}

void cDrawScenarioImitateStepEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateStepEval>(new cScenarioImitateStepEval());
}

bool cDrawScenarioImitateStepEval::EnableTargetPos() const
{
	auto step_scene = std::dynamic_pointer_cast<cScenarioExpImitateStep>(mScene);
	return step_scene->EnableTargetPos();
}

void cDrawScenarioImitateStepEval::SetTargetPos(const tVector& pos)
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->SetTargetPos(pos);
	scene->EnableTargetPos(true);
	scene->EnableRandTargetPos(false);
}

void cDrawScenarioImitateStepEval::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	//auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	//scene->EnableTargetPos(false);
	//scene->EnableRandTargetPos(true);
}

void cDrawScenarioImitateStepEval::DrawMisc() const
{
	cDrawScenarioImitateEval::DrawMisc();
	DrawStepPlan();

	if (EnableTargetPos())
	{
		DrawTargetPos();
	}
}

void cDrawScenarioImitateStepEval::DrawStepPlan() const
{
	const tVector& pos0_col = tVector(0, 1, 0, 0.5);
	const tVector& pos1_col = tVector(0, 0.5, 0, 0.5);
	const tVector& heading_col = tVector(1, 0, 0, 0.5);

	auto step_scene = std::dynamic_pointer_cast<cScenarioExpImitateStep>(mScene);
	const auto& step_plan = step_scene->GetStepPlan();

	DrawStepPos(step_plan.mStepPos0, pos0_col);
	DrawStepPos(step_plan.mStepPos1, pos1_col);
	DrawRootHeading(step_plan, heading_col);
}

void cDrawScenarioImitateStepEval::DrawStepPos(const tVector& pos, const tVector& col) const
{
	const double r = 0.075;

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);
	cDrawUtil::PopMatrix();
}

void cDrawScenarioImitateStepEval::DrawRootHeading(const cBipedStepController3D::tStepPlan& step_plan, const tVector& col) const
{
	const tVector offset = tVector(0, 0.02, 0, 0);
	const double arrow_size = 0.2;
	const double arrow_len = 0.5;

	double theta = step_plan.mRootHeading;

	tVector start = 0.5 * (step_plan.mStepPos0 + step_plan.mStepPos1);
	start += offset;
	tVector dir = tVector(std::cos(theta), std::sin(theta), 0, 0);
	tVector end = arrow_len * dir;

	cDrawUtil::SetColor(col);
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(start);
	cDrawUtil::Rotate(-0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawArrow2D(tVector::Zero(), end, arrow_size);
	cDrawUtil::PopMatrix();
}

tVector cDrawScenarioImitateStepEval::GetCamTrackPos() const
{
#if defined(ENABLE_KIN_CONTROLLER_TEST)
	const auto& kin_char = GetKinChar();
	return kin_char->GetRootPos();
#endif
	return cDrawScenarioImitateTargetEval::GetCamTrackPos();
}

#if defined(ENABLE_HACK_LLC_LERP)
void cDrawScenarioImitateStepEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioImitateTargetEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		AdjustHackLerp(-0.1);
		break;
	case 'i':
		AdjustHackLerp(0.1);
		break;
	default:
		break;
	}
}

std::string cDrawScenarioImitateStepEval::BuildTextInfoStr() const
{
	std::string str = cDrawScenarioImitateTargetEval::BuildTextInfoStr();

	const auto& character = mScene->GetCharacter();
	const auto& ctrl = character->GetController();
	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(ctrl);
	if (step_ctrl != nullptr)
	{
		double lerp = step_ctrl->GetHackLerp();
		str += "Action Lerp: " + std::to_string(lerp) + "\n";
	}

	return str;
}

void cDrawScenarioImitateStepEval::AdjustHackLerp(double delta)
{
	const auto& character = mScene->GetCharacter();
	const auto& ctrl = character->GetController();
	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(ctrl);
	if (step_ctrl != nullptr)
	{
		double lerp = step_ctrl->GetHackLerp();
		lerp += delta;
		step_ctrl->SetHackLerp(lerp);

		printf("Hack Lerp: %.5f\n", lerp);
	}
}
#endif



#if defined(ENABLE_HACK_RECORD_ACTION_PERTURB)
// hack hack hack
// get rid of these
void cDrawScenarioImitateStepEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioImitateTargetEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		HackRecordActionPerturb();
		break;
	default:
		break;
	}
}


void cDrawScenarioImitateStepEval::HackRecordActionPerturb()
{
	const std::string hack_file = "output/hack_record_action_perturb.txt";

	const int num_samples = 10;
	if (!mHackEnableRecordActionPerturb)
	{
		mHackEnableRecordActionPerturb = true;
		mHackRecordActionPerturbPhase = 0;
	}

	printf("hack record action perturb phase: %.5f\n", mHackRecordActionPerturbPhase);

	const auto& character = mScene->GetCharacter();
	auto ctrl = std::dynamic_pointer_cast<cCtController>(character->GetController());

	const int waist_id = 1;
	const double waist_theta0 = M_PI / 2;
	const double waist_theta1 = -M_PI / 2;
	const double root_vel0 = -2;
	const double root_vel1 = 2;

	Eigen::VectorXd pose = character->GetPose();
	Eigen::VectorXd vel = character->GetVel();

	int param_offset = character->GetParamOffset(waist_id);
	int param_size = character->GetParamSize(waist_id);

	double theta = (1 - mHackRecordActionPerturbPhase) * waist_theta0 + mHackRecordActionPerturbPhase * waist_theta1;
	tQuaternion waist_rot = cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0), theta);
	pose.segment(param_offset, param_size) = cMathUtil::QuatToVec(waist_rot);
	character->SetPose(pose);

	double v = (1 - mHackRecordActionPerturbPhase) * root_vel0 + mHackRecordActionPerturbPhase * root_vel1;
	tVector root_vel = tVector(v, 0, 0, 0);
	vel.segment(0, 3) = root_vel.segment(0, 3);
	//character->SetVel(vel);

	Eigen::VectorXd action;
	ctrl->ForceActionUpdate();
	ctrl->RecordPoliAction(action);

	std::string action_str = std::to_string(theta) + "\t";
	//std::string action_str = std::to_string(v) + "\t";
	for (int i = 0; i < action.size(); ++i)
	{
		action_str += std::to_string(action[i]) + "\t";
	}
	action_str += "\n";

	cFileUtil::AppendText(action_str, hack_file);

	mHackRecordActionPerturbPhase += 1.0 / num_samples;
}

#endif

#if defined(ENABLE_HACK_ADJUST_PERIOD)
void cDrawScenarioImitateStepEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioImitateTargetEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		HackAdjustCtrlPeriod(-0.1);
		break;
	case 'i':
		HackAdjustCtrlPeriod(0.1);
		break;
	default:
		break;
	}
}

std::string cDrawScenarioImitateStepEval::BuildTextInfoStr() const
{
	std::string str = cDrawScenarioImitateTargetEval::BuildTextInfoStr();
	auto ctrl = std::dynamic_pointer_cast<cCtPhaseController>(mScene->GetCharacter()->GetController());
	if (ctrl != nullptr)
	{
		double dur = ctrl->GetCycleDur();
		str += "Cycle Period: " + std::to_string(dur) + "\n";
	}

	return str;
}

void cDrawScenarioImitateStepEval::HackAdjustCtrlPeriod(double delta)
{
	auto ctrl = std::dynamic_pointer_cast<cCtPhaseController>(mScene->GetCharacter()->GetController());
	if (ctrl != nullptr)
	{
		double period = ctrl->GetCycleDur();
		period += delta;
		ctrl->SetCycleDur(period);
	}
}

#endif