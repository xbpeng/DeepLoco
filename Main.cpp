#include <iostream>
#include <caffe/caffe.hpp>

#include "util/FileUtil.h"
#include "util/ArgParser.h"
#include "scenarios/DrawScenarioSimChar.h"
#include "scenarios/DrawScenarioTrackMotion.h"
#include "scenarios/DrawScenarioTrain.h"
#include "scenarios/DrawScenarioTrainCacla.h"
#include "scenarios/DrawScenarioPoliEval.h"
#include "scenarios/DrawScenarioImitateEval.h"
#include "scenarios/DrawScenarioImitateTargetEval.h"
#include "scenarios/DrawScenarioHikeEval.h"
#include "scenarios/DrawScenarioSoccerEval.h"
#include "scenarios/DrawScenarioImitateStepEval.h"
#include "scenarios/DrawScenarioImitate.h"
#include "scenarios/DrawScenarioImitateTarget.h"
#include "scenarios/DrawScenarioImitateStep.h"
#include "scenarios/DrawScenarioTrainHike.h"
#include "scenarios/DrawScenarioTrainSoccer.h"

#include "render/Camera.h"
#include "render/DrawUtil.h"
#include "render/TextureDesc.h"

#include "util/BVHReader.h"
#include "util/MotionDB.h"

// Dimensions of the window we are drawing into.
int gWinWidth = 800;
int gWinHeight = static_cast<int>(gWinWidth * 9.0 / 16.0);
//int gWinWidth = 720;
//int gWinHeight = 480;
bool gReshaping = false;

const tVector gBKGColor = tVector(0.97, 0.97, 1, 0);
//const tVector gBKGColor = tVector(1, 1, 1, 0);

// camera attributes
double gViewWidth = 4.5;
//double gViewWidth = 6.5;
double gViewHeight = (gViewWidth * gWinHeight) / gWinWidth;
double gViewNearZ = 2;
//double gViewNearZ = 25;
double gViewFarZ = 500;

// intermediate frame buffers
std::unique_ptr<cTextureDesc> gDefaultFrameBuffer;
std::shared_ptr<cTextureDesc> gIntermediateFrameBuffer;

tVector gCameraPosition = tVector(0, 0, 30, 0);
tVector gCameraFocus = tVector(gCameraPosition[0], gCameraPosition[1], 0.0, 0.0);
tVector gCameraUp = tVector(0, 1, 0, 0);

cCamera gCamera;

// anim
const double gFPS = 30.0;
const double gAnimStep = 1.0 / gFPS;
const int gDisplayAnimTime = static_cast<int>(gAnimStep * 1000);
bool gAnimate = true;

int gPrevTime = 0;
double gUpdatesPerSec = 0;

double gPlaybackSpeed = 1.0;
const double gPlaybackDelta = 0.05;

bool gForceClear = false;
bool gRenderFilmStrip = false;
bool gDrawInfo = true;
const double gFilmStripPeriod = 0.5;
tVector gPrevCamPos = tVector::Zero();

// arg parser
std::shared_ptr<cArgParser> gArgParser = nullptr;
std::shared_ptr<cDrawScenario> gScenario = nullptr;
int gArgc = 0;
char** gArgv = nullptr;


void SetupCamProjection()
{
	gCamera.SetupGLProj();
}

void ResizeCamera()
{
	double prev_view_w = gCamera.GetWidth();
	double prev_view_h = gCamera.GetHeight();
	double new_view_h = prev_view_h;
	double new_view_w = (prev_view_h * gWinWidth) / gWinHeight;
	gCamera.Resize(new_view_w, new_view_h);

	gViewWidth = gCamera.GetWidth();
	gViewHeight = gCamera.GetHeight();
	gForceClear = true;
}

void InitCamera()
{
	gCamera = cCamera(gCameraPosition, gCameraFocus, gCameraUp,
						gViewWidth, gViewHeight, gViewNearZ, gViewFarZ);
	gCamera.SetProj(cCamera::eProjOrtho);
	ResizeCamera();
}

void ClearScenario()
{
	gScenario = nullptr;
}

void SetupScenario()
{
	InitCamera();
	ClearScenario();

	std::string scenario_name = "";
	gArgParser->ParseString("scenario", scenario_name);

	if (scenario_name == "sim_char")
	{
		gScenario = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
	}
	else if (scenario_name == "track_motion")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrackMotion>(new cDrawScenarioTrackMotion(gCamera));
	}
	else if (scenario_name == "train")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrain>(new cDrawScenarioTrain(gCamera));
	}
	else if (scenario_name == "train_cacla")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainCacla>(new cDrawScenarioTrainCacla(gCamera));
	}
	else if (scenario_name == "train_cacla_dq")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainCacla>(new cDrawScenarioTrainCacla(gCamera));
	}
	else if (scenario_name == "imitate")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitate>(new cDrawScenarioImitate(gCamera));
	}
	else if (scenario_name == "imitate_target")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitateTarget>(new cDrawScenarioImitateTarget(gCamera));
	}
	else if (scenario_name == "imitate_step")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitateStep>(new cDrawScenarioImitateStep(gCamera));
	}
	else if (scenario_name == "train_hike")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainHike>(new cDrawScenarioTrainHike(gCamera));
	}
	else if (scenario_name == "train_soccer")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainSoccer>(new cDrawScenarioTrainSoccer(gCamera));
	}
	else if (scenario_name == "poli_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioPoliEval>(new cDrawScenarioPoliEval(gCamera));
	}
	else if (scenario_name == "imitate_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitateEval>(new cDrawScenarioImitateEval(gCamera));
	}
	else if (scenario_name == "imitate_target_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitateTargetEval>(new cDrawScenarioImitateTargetEval(gCamera));
	}
	else if (scenario_name == "imitate_step_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioImitateStepEval>(new cDrawScenarioImitateStepEval(gCamera));
	}
	else if (scenario_name == "hike_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioHikeEval>(new cDrawScenarioHikeEval(gCamera));
	}
	else if (scenario_name == "soccer_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioSoccerEval>(new cDrawScenarioSoccerEval(gCamera));
	}

	if (gScenario != NULL)
	{
		auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(gScenario);
		if (sim_char_scene != nullptr)
		{
			sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
		}

		gScenario->ParseArgs(gArgParser);
		gScenario->Init();
		printf("Loaded scenario: %s\n", gScenario->GetName().c_str());
	}
}

void UpdateScenario(double time_step)
{
	if (gScenario != NULL)
	{
		int num_steps = 1;
		if (gRenderFilmStrip)
		{
			num_steps = static_cast<int>(gFilmStripPeriod / time_step);
		}

		for (int i = 0; i < num_steps; ++i)
		{
			gScenario->Update(time_step);
		}
	}
}

void DrawInfo()
{
	if (!gRenderFilmStrip && gDrawInfo)
	{
		glMatrixMode(GL_PROJECTION);
		cDrawUtil::PushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_MODELVIEW);
		cDrawUtil::PushMatrix();
		glLoadIdentity();

		const double aspect = gCamera.GetAspectRatio();
		const double text_size = 0.09;
		const tVector scale = tVector(text_size / aspect, text_size, 1, 0);
		const double line_offset = text_size;

		cDrawUtil::SetLineWidth(1.5);
		cDrawUtil::SetColor(tVector(0, 0, 0, 0.5));

		cDrawUtil::Translate(tVector(-0.96, 0.88, -1, 0));
		double curr_fps = gUpdatesPerSec;

		char buffer[128];
		sprintf(buffer, "FPS: %.2f (%.2fx)\n", curr_fps, gPlaybackSpeed);

		std::string text_info = std::string(buffer);
		if (gScenario != nullptr)
		{
			text_info += gScenario->BuildTextInfoStr();
		}

		cDrawUtil::DrawString(text_info.c_str(), scale);

		glMatrixMode(GL_PROJECTION);
		cDrawUtil::PopMatrix();

		glMatrixMode(GL_MODELVIEW);
		cDrawUtil::PopMatrix();
	}
}

void ClearFrame()
{
	bool clear = true;

	if (gRenderFilmStrip && !gForceClear)
	{
		const tVector& cam_pos = gCamera.GetPosition();
		if (cam_pos != gPrevCamPos)
		{
			gPrevCamPos = cam_pos;
		}
		else
		{
			clear = false;
		}
	}

	if (clear)
	{
		cDrawUtil::ClearColor(gBKGColor);
		cDrawUtil::ClearDepth(1);
	}

	gForceClear = false;
}

void DrawScene()
{
	if (gScenario != NULL)
	{
		gScenario->Draw();
	}
}

void CopyFrame()
{
	cDrawUtil::CopyTexture(*gIntermediateFrameBuffer, *gDefaultFrameBuffer);
}

void UpdateIntermediateBuffer()
{
	if (!gReshaping)
	{
		if (gWinWidth != gIntermediateFrameBuffer->GetWidth()
			|| gWinHeight != gIntermediateFrameBuffer->GetHeight())
		{
			gIntermediateFrameBuffer->Reshape(gWinWidth, gWinHeight);
			gScenario->Reshape(gWinWidth, gWinHeight);
		}
	}
}

void Display(void)
{
	UpdateIntermediateBuffer();

	glMatrixMode(GL_PROJECTION);
	SetupCamProjection();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gIntermediateFrameBuffer->BindBuffer();
	ClearFrame();
	DrawScene();
	DrawInfo();
	gIntermediateFrameBuffer->UnbindBuffer();

	CopyFrame();

	glutSwapBuffers();

	gReshaping = false;
}

void Reshape(int w, int h)
{
	gReshaping = true;

	gWinWidth = w;
	gWinHeight = h;

	glViewport(0, 0, gWinWidth, gWinHeight);
	gDefaultFrameBuffer->Reshape(w, h);

	UpdateScenario(0);
	ResizeCamera();

	glMatrixMode(GL_PROJECTION);
	SetupCamProjection();

	glutPostRedisplay();
}

void StepAnim(double time_step)
{
	UpdateScenario(time_step);
	gAnimate = false;
	glutPostRedisplay();
}

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
}

int GetCurrTime()
{
	return glutGet(GLUT_ELAPSED_TIME);
}

void InitTime()
{
	gPrevTime = GetCurrTime();
	gUpdatesPerSec = 0.f;
}

void Reload()
{
	ParseArgs(gArgc, gArgv);
	SetupScenario();
	InitTime();
	gForceClear = true;
}

void Reset()
{
	if (gScenario != NULL)
	{
		gScenario->Reset();
	}
	gForceClear = true;
}

void Update(double time_elapsed)
{
	UpdateScenario(time_elapsed);
}

int GetNumTimeSteps()
{
	int num_steps = static_cast<int>(gPlaybackSpeed);
	if (num_steps == 0)
	{
		num_steps = 1;
	}
	num_steps = std::abs(num_steps);
	return num_steps;
}

int CalcDisplayAnimTime(int num_timesteps)
{
	int anim_time = static_cast<int>(gDisplayAnimTime * num_timesteps / gPlaybackSpeed);
	anim_time = std::abs(anim_time);
	return anim_time;
}

void Shutdown()
{
	if (gScenario != nullptr)
	{
		gScenario->Shutdown();
	}
	exit(0);
}

void Animate(int callback_val)
{
	if (gAnimate)
	{
		int num_steps = GetNumTimeSteps();
		
		int current_time = GetCurrTime();
		int elapsedTime = current_time - gPrevTime;
		gPrevTime = current_time;

		double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
		for (int i = 0; i < num_steps; ++i)
		{
			Update(timestep);
		}
		
		int update_dur = GetCurrTime() - current_time;

		glutPostRedisplay();
		gUpdatesPerSec = num_steps / (elapsedTime * 0.001);
		int timer_step = CalcDisplayAnimTime(num_steps);
		timer_step -= update_dur;
		timer_step = std::max(timer_step, 0);

		glutTimerFunc(timer_step, Animate, 0);
	}

	if (gScenario != nullptr)
	{
		if (gScenario->IsDone())
		{
			Shutdown();
		}
	}
}

void ToggleAnimate()
{
	gAnimate = !gAnimate;
	if (gAnimate)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void ChangePlaybackSpeed(double delta)
{
	double prev_playback = gPlaybackSpeed;
	gPlaybackSpeed += delta;

	if (std::abs(prev_playback) < 0.0001 && std::abs(gPlaybackSpeed) > 0.0001)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void ToggleDrawInfo()
{
	gDrawInfo = !gDrawInfo;
	if (gDrawInfo)
	{
		printf("Draw info enabled\n");
	}
	else
	{
		printf("Draw info disabled\n");
	}
}

void ToggleFilmStrip()
{
	gRenderFilmStrip = !gRenderFilmStrip;
	gForceClear = true;

	if (gScenario != NULL)
	{
		gScenario->EnableFilmstrip(gRenderFilmStrip);
	}

	if (gRenderFilmStrip)
	{
		printf("Filmstrip mode enabled\n");
	}
	else
	{
		printf("Filmstrip mode disabled\n");
	}
}

void Keyboard(unsigned char key, int x, int y) {

	if (gScenario != NULL)
	{
		gScenario->Keyboard(key, x, y);
	}

	bool update = false;
	switch (key) {
		// Quit.
#ifndef _LINUX_
	// case CTRL_CLOSE_EVENT:
	// case CTRL_C_EVENT:
#endif
	case 27: // escape
		Shutdown();
		break;
	case 13: // enter
		break;
	case ' ':
		ToggleAnimate();
		break;
	case '>':
		StepAnim(gAnimStep);
		break;
	case '<':
		StepAnim(-gAnimStep);
		break;
	case ',':
		ChangePlaybackSpeed(-gPlaybackDelta);
		break;
	case '.':
		ChangePlaybackSpeed(gPlaybackDelta);
		break;
	case '/':
		ChangePlaybackSpeed(-gPlaybackSpeed + 1);
		break;
	case 'l':
		Reload();
		break;
	case 'j':
		ToggleDrawInfo();
		break;
	case 'q':
		ToggleFilmStrip();
		break;
	case 'r':
		Reset();
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void MouseClick(int button, int state, int x, int y)
{
	double screen_x = static_cast<double>(x) / gWinWidth;
	double screen_y = static_cast<double>(y) / gWinHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseClick(button, state, screen_x, screen_y);
	}
	glutPostRedisplay();
}


void MouseMove(int x, int y)
{
	double screen_x = static_cast<double>(x) / gWinWidth;
	double screen_y = static_cast<double>(y) / gWinHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseMove(screen_x, screen_y);
	}
	glutPostRedisplay();
}

void InitOpenGl(void)
{
	glewInit();
	cDrawUtil::InitDrawUtil();

	gDefaultFrameBuffer = std::unique_ptr<cTextureDesc>(new cTextureDesc(0, 0, 0, gWinWidth, gWinHeight, 1, GL_RGBA, GL_RGBA));
	gIntermediateFrameBuffer = std::shared_ptr<cTextureDesc>(new cTextureDesc(gWinWidth, gWinHeight, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));
}

void HackDogMeasurements()
{
	double scale = 0.1136;
	tVector origin = tVector(12.73, 19.85, 0, 0);
	tMatrix m = cMathUtil::ScaleMat(scale) * cMathUtil::TranslateMat(-origin);
	m = cMathUtil::ScaleMat(tVector(tVector(-1, -1, 1, 0))) * m;

	tVector root = m * tVector(11.07, 9.15, 0, 1);
	tVector root_spine0 = m * tVector(11.68, 9.22, 0, 1);
	tVector spine0_spine1 = m * tVector(11.48, 8.76, 0, 1);
	tVector spine1_spine2 = m * tVector(11.08, 8.25, 0, 1);
	tVector spine2_spine3 = m * tVector(10.67, 7.71, 0, 1);
	tVector spine3_torso = m * tVector(10.05, 6.88, 0, 1);
	tVector torso_neck0 = m * tVector(8.94, 5.39, 0, 1);
	tVector neck0_neck1 = m * tVector(9.13, 4.6, 0, 1);
	tVector neck1_head = m * tVector(9.26, 3.85, 0, 1);
	tVector head_end = m * tVector(7.91, 3.4, 0, 1);

	tVector root_tail0 = m * tVector(72.89, 13.09, 0, 1);
	tVector tail0_tail1 = m * tVector(71.34, 15.15, 0, 1);
	tVector tail1_tail2 = m * tVector(69.79, 17.26, 0, 1);
	tVector tail2_tail3 = m * tVector(67.55, 18.99, 0, 1);
	tVector tail3_end = m * tVector(65.39, 19.42, 0, 1);

	tVector torso_upperarm = m * tVector(8.81, 6.17, 0, 1);
	tVector upper_arm_lower_arm = m * tVector(8.98, 8.14, 0, 1);
	tVector lower_arm_hand = m * tVector(8.04, 9.77, 0, 1);
	tVector lower_hand_finger = m * tVector(7.42, 10.5, 0, 1);
	tVector finger_end = m * tVector(6.86, 10.51, 0, 1);

	tVector root_thigh = m * tVector(11.54, 10.16, 0, 1);
	tVector thigh_leg = m * tVector(10.03, 8.91, 0, 1);
	tVector leg_foot = m * tVector(9.14, 10.77, 0, 1);
	tVector foot_toe = m * tVector(8, 10.57, 0, 1);
	tVector toe_end = m * tVector(7.43, 10.57, 0, 1);

	/*
	double scale = 0.6;
	tVector root = tVector(0, 1.61, 0, 1) * scale;
	tVector root_spine0 = tVector(0.09, 1.69, 0, 1) * scale;
	tVector spine0_spine1 = tVector(0.19, 1.71, 0, 1) * scale;
	tVector spine1_spine2 = tVector(0.31, 1.69, 0, 1) * scale;
	tVector spine2_spine3 = tVector(0.44, 1.69, 0, 1) * scale;
	tVector spine3_torso = tVector(0.62, 1.68, 0, 1) * scale;
	tVector torso_neck0 = tVector(0.98, 1.71, 0, 1) * scale;
	tVector neck0_neck1 = tVector(1.07, 1.82, 0, 1) * scale;
	tVector neck1_head = tVector(1.16, 1.93, 0, 1) * scale;
	tVector head_end = tVector(1.36, 1.43, 0, 1) * scale;

	tVector root_tail0 = tVector(-0.14, 1.6, 0, 1) * scale;
	tVector tail0_tail1 = tVector(-0.28, 1.6, 0, 1) * scale;
	tVector tail1_tail2 = tVector(-0.43, 1.6, 0, 1) * scale;
	tVector tail2_tail3 = tVector(-0.57, 1.6, 0, 1) * scale;
	tVector tail3_end = tVector(-0.71, 1.6, 0, 1) * scale;

	tVector torso_upperarm = tVector(0.88, 1.6, 0, 1) * scale;
	tVector upper_arm_lower_arm = tVector(0.69, 1.27, 0, 1) * scale;
	tVector lower_arm_hand = tVector(0.81, 0.94, 0, 1) * scale;
	tVector lower_hand_finger = tVector(0.89, 0.78, 0, 1) * scale;
	tVector finger_end = tVector(0.99, 0.78, 0, 1) * scale;

	tVector root_thigh = tVector(-0.05, 1.58, 0, 1) * scale;
	tVector thigh_leg = tVector(0.17, 1.27, 0, 1) * scale;
	tVector leg_foot = tVector(-0.08, 0.97, 0, 1) * scale;
	tVector foot_toe = tVector(0.05, 0.79, 0, 1) * scale;
	tVector toe_end = tVector(0.15, 0.79, 0, 1) * scale;
	*/


	tVector root_spine0_bone = (root_spine0 - root);
	tVector spine0 = (spine0_spine1 - root_spine0);
	tVector spine1 = (spine1_spine2 - spine0_spine1);
	tVector spine2 = (spine2_spine3 - spine1_spine2);
	tVector spine3 = (spine3_torso - spine2_spine3);
	tVector torso = (torso_neck0 - spine3_torso);
	tVector neck0 = (neck0_neck1 - torso_neck0);
	tVector neck1 = (neck1_head - neck0_neck1);
	tVector head = (head_end - neck1_head);

	tVector root_tail0_bone = (root_tail0 - root);
	tVector tail0 = (tail0_tail1 - root_tail0);
	tVector tail1 = (tail1_tail2 - tail0_tail1);
	tVector tail2 = (tail2_tail3 - tail1_tail2);
	tVector tail3 = (tail3_end - tail2_tail3);

	tVector torso_upperarm_bone = (torso_upperarm - spine3_torso);
	tVector forearm = (upper_arm_lower_arm - torso_upperarm);
	tVector arm = (lower_arm_hand - upper_arm_lower_arm);
	tVector hand = (lower_hand_finger - lower_arm_hand);
	tVector finger = (finger_end - lower_hand_finger);

	tVector root_thigh_bone = (root_thigh - root);
	tVector thigh = (thigh_leg - root_thigh);
	tVector leg = (leg_foot - thigh_leg);
	tVector foot = (foot_toe - leg_foot);
	tVector toe = (toe_end - foot_toe);


	double root_spine0_len = root_spine0_bone.segment(0, 3).norm();
	double spine0_len = spine0.segment(0, 3).norm();
	double spine1_len = spine1.segment(0, 3).norm();
	double spine2_len = spine2.segment(0, 3).norm();
	double spine3_len = spine3.segment(0, 3).norm();
	double torso_len = torso.segment(0, 3).norm();
	double neck0_len = neck0.segment(0, 3).norm();
	double neck1_len = neck1.segment(0, 3).norm();
	double head_len = head.segment(0, 3).norm();

	double root_tail0_len = root_tail0_bone.segment(0, 3).norm();
	double tail0_len = tail0.segment(0, 3).norm();
	double tail1_len = tail1.segment(0, 3).norm();
	double tail2_len = tail2.segment(0, 3).norm();
	double tail3_len = tail3.segment(0, 3).norm();

	double torso_upperarm_len = torso_upperarm_bone.segment(0, 3).norm();
	double forearm_len = forearm.segment(0, 3).norm();
	double arm_len = arm.segment(0, 3).norm();
	double hand_len = hand.segment(0, 3).norm();
	double finger_len = finger.segment(0, 3).norm();

	double root_thigh_len = root_thigh_bone.segment(0, 3).norm();
	double thigh_len = thigh.segment(0, 3).norm();
	double leg_len = leg.segment(0, 3).norm();
	double foot_len = foot.segment(0, 3).norm();
	double toe_len = toe.segment(0, 3).norm();


	double root_x = root[0];
	double root_y = root[1];
	double root_theta = std::atan2(root_spine0_bone[1], root_spine0_bone[0]) - 0.72664234068172606;

	double spine0_theta = std::atan2(spine0[1], spine0[0]);
	double spine1_theta = std::atan2(spine1[1], spine1[0]);
	double spine2_theta = std::atan2(spine2[1], spine2[0]);
	double spin3_theta = std::atan2(spine3[1], spine3[0]);
	double torso_theta = std::atan2(torso[1], torso[0]) - 0.083141231888441317;
	double neck0_theta = std::atan2(neck0[1], neck0[0]);
	double neck1_theta = std::atan2(neck1[1], neck1[0]);
	double head_theta = std::atan2(head[1], head[0]) + M_PI * 0.5;

	double tail0_theta = std::atan2(tail0[1], tail0[0]) - M_PI;
	double tail1_theta = std::atan2(tail1[1], tail1[0]) - M_PI;
	double tail2_theta = std::atan2(tail2[1], tail2[0]) - M_PI;
	double tail3_theta = std::atan2(tail3[1], tail3[0]) - M_PI;

	double forearm_theta = std::atan2(forearm[1], forearm[0]) + M_PI * 0.5;
	double arm_theta = std::atan2(arm[1], arm[0]) + M_PI * 0.5;
	double hand_theta = std::atan2(hand[1], hand[0]) + M_PI * 0.5;
	double finger_theta = std::atan2(finger[1], finger[0]);

	double thigh_theta = std::atan2(thigh[1], thigh[0]) + M_PI * 0.5;
	double leg_theta = std::atan2(leg[1], leg[0]) + M_PI * 0.5;
	double foot_theta = std::atan2(foot[1], foot[0]) + M_PI * 0.5;
	double toe_theta = std::atan2(toe[1], toe[0]);



	double spine0_theta_rel = spine0_theta - root_theta;
	double spine1_theta_rel = spine1_theta - spine0_theta;
	double spine2_theta_rel = spine2_theta - spine1_theta;
	double spin3_theta_rel = spin3_theta - spine2_theta;
	double torso_theta_rel = torso_theta - spin3_theta;
	double neck0_theta_rel = neck0_theta - torso_theta;
	double neck1_theta_rel = neck1_theta - neck0_theta;
	double head_theta_rel = head_theta - neck1_theta;

	double tail0_theta_rel = tail0_theta - root_theta;
	double tail1_theta_rel = tail1_theta - tail0_theta;
	double tail2_theta_rel = tail2_theta - tail1_theta;
	double tail3_theta_rel = tail3_theta - tail2_theta;

	double forearm_theta_rel = forearm_theta - torso_theta;
	double arm_theta_rel = arm_theta - forearm_theta;
	double hand_theta_rel = hand_theta - arm_theta;
	double finger_theta_rel = finger_theta - hand_theta;

	double thigh_theta_rel = thigh_theta - root_theta;
	double leg_theta_rel = leg_theta - thigh_theta;
	double foot_theta_rel = foot_theta - leg_theta;
	double toe_theta_rel = toe_theta - foot_theta;


	spine0_theta_rel = (spine0_theta_rel > M_PI) ? -(2 * M_PI - spine0_theta_rel) : spine0_theta_rel;
	spine1_theta_rel = (spine1_theta_rel > M_PI) ? -(2 * M_PI - spine1_theta_rel) : spine1_theta_rel;
	spine2_theta_rel = (spine2_theta_rel > M_PI) ? -(2 * M_PI - spine2_theta_rel) : spine2_theta_rel;
	spin3_theta_rel = (spin3_theta_rel > M_PI) ? -(2 * M_PI - spin3_theta_rel) : spin3_theta_rel;
	torso_theta_rel = (torso_theta_rel > M_PI) ? -(2 * M_PI - torso_theta_rel) : torso_theta_rel;
	neck0_theta_rel = (neck0_theta_rel > M_PI) ? -(2 * M_PI - neck0_theta_rel) : neck0_theta_rel;
	neck1_theta_rel = (neck1_theta_rel > M_PI) ? -(2 * M_PI - neck1_theta_rel) : neck1_theta_rel;
	head_theta_rel = (head_theta_rel > M_PI) ? -(2 * M_PI - head_theta_rel) : head_theta_rel;

	tail0_theta_rel = (tail0_theta_rel > M_PI) ? -(2 * M_PI - tail0_theta_rel) : tail0_theta_rel;
	tail1_theta_rel = (tail1_theta_rel > M_PI) ? -(2 * M_PI - tail1_theta_rel) : tail1_theta_rel;
	tail2_theta_rel = (tail2_theta_rel > M_PI) ? -(2 * M_PI - tail2_theta_rel) : tail2_theta_rel;
	tail3_theta_rel = (tail3_theta_rel > M_PI) ? -(2 * M_PI - tail3_theta_rel) : tail3_theta_rel;

	forearm_theta_rel = (forearm_theta_rel > M_PI) ? -(2 * M_PI - forearm_theta_rel) : forearm_theta_rel;
	arm_theta_rel = (arm_theta_rel > M_PI) ? -(2 * M_PI - arm_theta_rel) : arm_theta_rel;
	hand_theta_rel = (hand_theta_rel > M_PI) ? -(2 * M_PI - hand_theta_rel) : hand_theta_rel;
	finger_theta_rel = (finger_theta_rel > M_PI) ? -(2 * M_PI - finger_theta_rel) : finger_theta_rel;

	thigh_theta_rel = (thigh_theta_rel > M_PI) ? -(2 * M_PI - thigh_theta_rel) : thigh_theta_rel;
	leg_theta_rel = (leg_theta_rel > M_PI) ? -(2 * M_PI - leg_theta_rel) : leg_theta_rel;
	foot_theta_rel = (foot_theta_rel > M_PI) ? -(2 * M_PI - foot_theta_rel) : foot_theta_rel;
	toe_theta_rel = (toe_theta_rel > M_PI) ? -(2 * M_PI - toe_theta_rel) : toe_theta_rel;


	spine0_theta_rel = (spine0_theta_rel < -M_PI) ? (2 * M_PI + spine0_theta_rel) : spine0_theta_rel;
	spine1_theta_rel = (spine1_theta_rel < -M_PI) ? (2 * M_PI + spine1_theta_rel) : spine1_theta_rel;
	spine2_theta_rel = (spine2_theta_rel < -M_PI) ? (2 * M_PI + spine2_theta_rel) : spine2_theta_rel;
	spin3_theta_rel = (spin3_theta_rel < -M_PI) ? (2 * M_PI + spin3_theta_rel) : spin3_theta_rel;
	torso_theta_rel = (torso_theta_rel < -M_PI) ? (2 * M_PI + torso_theta_rel) : torso_theta_rel;
	neck0_theta_rel = (neck0_theta_rel < -M_PI) ? (2 * M_PI + neck0_theta_rel) : neck0_theta_rel;
	neck1_theta_rel = (neck1_theta_rel < -M_PI) ? (2 * M_PI + neck1_theta_rel) : neck1_theta_rel;
	head_theta_rel = (head_theta_rel < -M_PI) ? (2 * M_PI + head_theta_rel) : head_theta_rel;

	tail0_theta_rel = (tail0_theta_rel < -M_PI) ? (2 * M_PI + tail0_theta_rel) : tail0_theta_rel;
	tail1_theta_rel = (tail1_theta_rel < -M_PI) ? (2 * M_PI + tail1_theta_rel) : tail1_theta_rel;
	tail2_theta_rel = (tail2_theta_rel < -M_PI) ? (2 * M_PI + tail2_theta_rel) : tail2_theta_rel;
	tail3_theta_rel = (tail3_theta_rel < -M_PI) ? (2 * M_PI + tail3_theta_rel) : tail3_theta_rel;

	forearm_theta_rel = (forearm_theta_rel < -M_PI) ? (2 * M_PI + forearm_theta_rel) : forearm_theta_rel;
	arm_theta_rel = (arm_theta_rel < -M_PI) ? (2 * M_PI + arm_theta_rel) : arm_theta_rel;
	hand_theta_rel = (hand_theta_rel < -M_PI) ? (2 * M_PI + hand_theta_rel) : hand_theta_rel;
	finger_theta_rel = (finger_theta_rel < -M_PI) ? (2 * M_PI + finger_theta_rel) : finger_theta_rel;

	thigh_theta_rel = (thigh_theta_rel < -M_PI) ? (2 * M_PI + thigh_theta_rel) : thigh_theta_rel;
	leg_theta_rel = (leg_theta_rel < -M_PI) ? (2 * M_PI + leg_theta_rel) : leg_theta_rel;
	foot_theta_rel = (foot_theta_rel < -M_PI) ? (2 * M_PI + foot_theta_rel) : foot_theta_rel;
	toe_theta_rel = (toe_theta_rel < -M_PI) ? (2 * M_PI + toe_theta_rel) : toe_theta_rel;


	const int num_data = 23;
	std::vector<double> data(num_data);
	{
		int i = 0;
		data[i++] = root_x;
		data[i++] = root_y;
		data[i++] = root_theta;

		data[i++] = spine0_theta_rel;
		data[i++] = spine1_theta_rel;
		data[i++] = spine2_theta_rel;
		data[i++] = spin3_theta_rel;
		data[i++] = torso_theta_rel;
		data[i++] = neck0_theta_rel;
		data[i++] = neck1_theta_rel;
		data[i++] = head_theta_rel;

		data[i++] = tail0_theta_rel * 0;
		data[i++] = tail1_theta_rel * 0;
		data[i++] = tail2_theta_rel * 0;
		data[i++] = tail3_theta_rel * 0;

		data[i++] = forearm_theta_rel;
		data[i++] = arm_theta_rel;
		data[i++] = hand_theta_rel;
		data[i++] = finger_theta_rel;

		data[i++] = thigh_theta_rel;
		data[i++] = leg_theta_rel;
		data[i++] = foot_theta_rel;
		data[i++] = toe_theta_rel;
		assert(i == num_data);
	}

	FILE* f = cFileUtil::OpenFile("output/hack_pose_data.txt", "w");

	for (int i = 0; i < num_data; ++i)
	{
		if (i != 0)
		{
			fprintf(f, ",\t");
		}
		fprintf(f, "%.5f", data[i]);
	}

	cFileUtil::CloseFile(f);
}

void HackRaptorMeasurements()
{
	double scale = 0.07;
	tVector origin = tVector(15.5, 13.82, 0, 0);
	tMatrix m = cMathUtil::ScaleMat(scale) * cMathUtil::TranslateMat(-origin);
	m = cMathUtil::ScaleMat(tVector(tVector(1, -1, 1, 0))) * m;

	tVector root = m * tVector(34.61, 5.75, 0, 1);
	tVector spine0 = m * tVector(31.57, 4.83, 0, 1);
	tVector spine1 = m * tVector(34.04, 4.62, 0, 1);
	tVector spine2 = m * tVector(35.5, 4.45, 0, 1);
	tVector spine3 = m * tVector(36.8, 4.23, 0, 1);
	tVector head = m * tVector(38, 3.88, 0, 1);
	tVector head_end = m * tVector(39.44, 4.76, 0, 1);

	tVector tail0 = m * tVector(27.5, 4.76, 0, 1);
	tVector tail1 = m * tVector(25.44, 4.45, 0, 1);
	tVector tail2 = m * tVector(23.28, 4.27, 0, 1);
	tVector tail3 = m * tVector(21.3, 4.2, 0, 1);
	tVector tail4 = m * tVector(18.91, 4.3, 0, 1);
	tVector tail_end = m * tVector(16.8, 4.7, 0, 1);

	tVector right_thigh = root;
	tVector right_knee = m * tVector(27.34, 8.43, 0, 1);
	tVector right_ankle = m * tVector(24.41, 8.7, 0, 1);
	tVector right_toe = m * tVector(22.54, 10.76, 0, 1);
	tVector right_toe_end = m * tVector(23.21, 11.43, 0, 1);

	tVector left_thigh = root;
	tVector left_knee = m * tVector(32.77, 7.7, 0, 1);
	tVector left_ankle = m * tVector(32.18, 10.2, 0, 1);
	tVector left_toe = m * tVector(36.3, 11.75, 0, 1);
	tVector left_toe_end = m * tVector(36.8, 11.22, 0, 1);




	tVector spine0_root = spine0 - root;
	tVector spine1_spine0 = spine1 - spine0;
	tVector spine2_spine1 = spine2 - spine1;
	tVector spine3_spine2 = spine3 - spine2;
	tVector head_spine3 = head - spine3;
	tVector head_to_end = head_end - head;
	
	tVector tail0_root = tail0 - root;
	tVector tail1_tail0 = tail1 - tail0;
	tVector tail2_tail1 = tail2 - tail1;
	tVector tail3_tail2 = tail3 - tail2;
	tVector tail4_tail3 = tail4 - tail3;
	tVector tail4_to_end = tail_end - tail4;

	tVector right_thigh_root = right_thigh - root;
	tVector right_knee_thigh = right_knee - right_thigh;
	tVector right_ankle_knee = right_ankle - right_knee;
	tVector right_toe_ankle = right_toe - right_ankle;
	tVector right_toe_to_end = right_toe_end - right_toe;

	tVector left_thigh_root = left_thigh - root;
	tVector left_knee_thigh = left_knee - left_thigh;
	tVector left_ankle_knee = left_ankle - left_knee;
	tVector left_toe_ankle = left_toe - left_ankle;
	tVector left_toe_to_end = left_toe_end - left_toe;

	double root_x = root[0];
	double root_y = root[1];
	double root_theta = std::atan2(spine0_root[1], spine0_root[0]) - 0.519701053;

	double spine0_theta = std::atan2(spine1_spine0[1], spine1_spine0[0]);
	double spine1_theta = std::atan2(spine2_spine1[1], spine2_spine1[0]);
	double spine2_theta = std::atan2(spine3_spine2[1], spine3_spine2[0]);
	double spine3_theta = std::atan2(head_spine3[1], head_spine3[0]);
	double head_theta = std::atan2(head_to_end[1], head_to_end[0]) + M_PI * 0.5;

	double tail0_theta = std::atan2(tail1_tail0[1], tail1_tail0[0]) - M_PI;
	double tail1_theta = std::atan2(tail2_tail1[1], tail2_tail1[0]) - M_PI;
	double tail2_theta = std::atan2(tail3_tail2[1], tail3_tail2[0]) - M_PI;
	double tail3_theta = std::atan2(tail4_tail3[1], tail4_tail3[0]) - M_PI;
	double tail4_theta = std::atan2(tail4_to_end[1], tail4_to_end[0]) - M_PI;

	double right_thigh_theta = std::atan2(right_knee_thigh[1], right_knee_thigh[0]) + M_PI * 0.5;
	double right_knee_theta = std::atan2(right_ankle_knee[1], right_ankle_knee[0]) + M_PI * 0.5;
	double right_ankle_theta = std::atan2(right_toe_ankle[1], right_toe_ankle[0]) + M_PI * 0.5;
	double right_toe_theta = std::atan2(right_toe_to_end[1], right_toe_to_end[0]);

	double left_thigh_theta = std::atan2(left_knee_thigh[1], left_knee_thigh[0]) + M_PI * 0.5;
	double left_knee_theta = std::atan2(left_ankle_knee[1], left_ankle_knee[0]) + M_PI * 0.5;
	double left_ankle_theta = std::atan2(left_toe_ankle[1], left_toe_ankle[0]) + M_PI * 0.5;
	double left_toe_theta = std::atan2(left_toe_to_end[1], left_toe_to_end[0]);
	


	double spine0_theta_rel = spine0_theta - root_theta;
	double spine1_theta_rel = spine1_theta - spine0_theta;
	double spine2_theta_rel = spine2_theta - spine1_theta;
	double spine3_theta_rel = spine3_theta - spine2_theta;
	double head_theta_rel = head_theta - spine3_theta;

	double tail0_theta_rel = tail0_theta - root_theta;
	double tail1_theta_rel = tail1_theta - tail0_theta;
	double tail2_theta_rel = tail2_theta - tail1_theta;
	double tail3_theta_rel = tail3_theta - tail2_theta;
	double tail4_theta_rel = tail4_theta - tail3_theta;

	double right_thigh_theta_rel = right_thigh_theta - root_theta;
	double right_knee_theta_rel = right_knee_theta - right_thigh_theta;
	double right_ankle_theta_rel = right_ankle_theta - right_knee_theta;
	double right_toe_theta_rel = right_toe_theta - right_ankle_theta;

	double left_thigh_theta_rel = left_thigh_theta - root_theta;
	double left_knee_theta_rel = left_knee_theta - left_thigh_theta;
	double left_ankle_theta_rel = left_ankle_theta - left_knee_theta;
	double left_toe_theta_rel = left_toe_theta - left_ankle_theta;



	spine0_theta_rel = (spine0_theta_rel > M_PI) ? -(2 * M_PI - spine0_theta_rel) : spine0_theta_rel;
	spine1_theta_rel = (spine1_theta_rel > M_PI) ? -(2 * M_PI - spine1_theta_rel) : spine1_theta_rel;
	spine2_theta_rel = (spine2_theta_rel > M_PI) ? -(2 * M_PI - spine2_theta_rel) : spine2_theta_rel;
	spine3_theta_rel = (spine3_theta_rel > M_PI) ? -(2 * M_PI - spine3_theta_rel) : spine3_theta_rel;
	head_theta_rel = (head_theta_rel > M_PI) ? -(2 * M_PI - head_theta_rel) : head_theta_rel;

	tail0_theta_rel = (tail0_theta_rel > M_PI) ? -(2 * M_PI - tail0_theta_rel) : tail0_theta_rel;
	tail1_theta_rel = (tail1_theta_rel > M_PI) ? -(2 * M_PI - tail1_theta_rel) : tail1_theta_rel;
	tail2_theta_rel = (tail2_theta_rel > M_PI) ? -(2 * M_PI - tail2_theta_rel) : tail2_theta_rel;
	tail3_theta_rel = (tail3_theta_rel > M_PI) ? -(2 * M_PI - tail3_theta_rel) : tail3_theta_rel;
	tail4_theta_rel = (tail4_theta_rel > M_PI) ? -(2 * M_PI - tail4_theta_rel) : tail4_theta_rel;

	right_thigh_theta_rel = (right_thigh_theta_rel > M_PI) ? -(2 * M_PI - right_thigh_theta_rel) : right_thigh_theta_rel;
	right_knee_theta_rel = (right_knee_theta_rel > M_PI) ? -(2 * M_PI - right_knee_theta_rel) : right_knee_theta_rel;
	right_ankle_theta_rel = (right_ankle_theta_rel > M_PI) ? -(2 * M_PI - right_ankle_theta_rel) : right_ankle_theta_rel;
	right_toe_theta_rel = (right_toe_theta_rel > M_PI) ? -(2 * M_PI - right_toe_theta_rel) : right_toe_theta_rel;

	left_thigh_theta_rel = (left_thigh_theta_rel > M_PI) ? -(2 * M_PI - left_thigh_theta_rel) : left_thigh_theta_rel;
	left_knee_theta_rel = (left_knee_theta_rel > M_PI) ? -(2 * M_PI - left_knee_theta_rel) : left_knee_theta_rel;
	left_ankle_theta_rel = (left_ankle_theta_rel > M_PI) ? -(2 * M_PI - left_ankle_theta_rel) : left_ankle_theta_rel;
	left_toe_theta_rel = (left_toe_theta_rel > M_PI) ? -(2 * M_PI - left_toe_theta_rel) : left_toe_theta_rel;



	spine0_theta_rel = (spine0_theta_rel < -M_PI) ? (2 * M_PI + spine0_theta_rel) : spine0_theta_rel;
	spine1_theta_rel = (spine1_theta_rel < -M_PI) ? (2 * M_PI + spine1_theta_rel) : spine1_theta_rel;
	spine2_theta_rel = (spine2_theta_rel < -M_PI) ? (2 * M_PI + spine2_theta_rel) : spine2_theta_rel;
	spine3_theta_rel = (spine3_theta_rel < -M_PI) ? (2 * M_PI + spine3_theta_rel) : spine3_theta_rel;
	head_theta_rel = (head_theta_rel < -M_PI) ? (2 * M_PI + head_theta_rel) : head_theta_rel;

	tail0_theta_rel = (tail0_theta_rel < -M_PI) ? (2 * M_PI + tail0_theta_rel) : tail0_theta_rel;
	tail1_theta_rel = (tail1_theta_rel < -M_PI) ? (2 * M_PI + tail1_theta_rel) : tail1_theta_rel;
	tail2_theta_rel = (tail2_theta_rel < -M_PI) ? (2 * M_PI + tail2_theta_rel) : tail2_theta_rel;
	tail3_theta_rel = (tail3_theta_rel < -M_PI) ? (2 * M_PI + tail3_theta_rel) : tail3_theta_rel;
	tail4_theta_rel = (tail4_theta_rel < -M_PI) ? (2 * M_PI + tail4_theta_rel) : tail4_theta_rel;

	right_thigh_theta_rel = (right_thigh_theta_rel < -M_PI) ? (2 * M_PI + right_thigh_theta_rel) : right_thigh_theta_rel;
	right_knee_theta_rel = (right_knee_theta_rel < -M_PI) ? (2 * M_PI + right_knee_theta_rel) : right_knee_theta_rel;
	right_ankle_theta_rel = (right_ankle_theta_rel < -M_PI) ? (2 * M_PI + right_ankle_theta_rel) : right_ankle_theta_rel;
	right_toe_theta_rel = (right_toe_theta_rel < -M_PI) ? (2 * M_PI + right_toe_theta_rel) : right_toe_theta_rel;

	left_thigh_theta_rel = (left_thigh_theta_rel < -M_PI) ? (2 * M_PI + left_thigh_theta_rel) : left_thigh_theta_rel;
	left_knee_theta_rel = (left_knee_theta_rel < -M_PI) ? (2 * M_PI + left_knee_theta_rel) : left_knee_theta_rel;
	left_ankle_theta_rel = (left_ankle_theta_rel < -M_PI) ? (2 * M_PI + left_ankle_theta_rel) : left_ankle_theta_rel;
	left_toe_theta_rel = (left_toe_theta_rel < -M_PI) ? (2 * M_PI + left_toe_theta_rel) : left_toe_theta_rel;



	const int num_data = 21;
	std::vector<double> data(num_data);
	{
		int i = 0;
		data[i++] = root_x;
		data[i++] = root_y;
		data[i++] = root_theta;

		data[i++] = spine0_theta_rel;
		data[i++] = spine1_theta_rel;
		data[i++] = spine2_theta_rel;
		data[i++] = spine3_theta_rel;
		data[i++] = head_theta_rel;

		data[i++] = tail0_theta_rel;
		data[i++] = tail1_theta_rel;
		data[i++] = tail2_theta_rel;
		data[i++] = tail3_theta_rel;
		data[i++] = tail4_theta_rel;

		data[i++] = right_thigh_theta_rel;
		data[i++] = right_knee_theta_rel;
		data[i++] = right_ankle_theta_rel;
		data[i++] = right_toe_theta_rel;

		data[i++] = left_thigh_theta_rel;
		data[i++] = left_knee_theta_rel;
		data[i++] = left_ankle_theta_rel;
		data[i++] = left_toe_theta_rel;

		assert(i == num_data);
	}

	FILE* f = cFileUtil::OpenFile("output/hack_pose_data.txt", "w");

	for (int i = 0; i < num_data; ++i)
	{
		if (i != 0)
		{
			fprintf(f, ",\t");
		}
		fprintf(f, "%.5f", data[i]);
	}

	cFileUtil::CloseFile(f);
}

void HackBipedMeasurements()
{
	double scale = 0.75 / 18.26;
	tVector origin = tVector(19.26, 59.17, 0, 0);
	tMatrix m = cMathUtil::ScaleMat(scale) * cMathUtil::TranslateMat(-origin);
	m = cMathUtil::ScaleMat(tVector(tVector(1, -1, 1, 0))) * m;

	tVector root = m * tVector(18.57, 41.05, 0, 1);
	tVector head = m * tVector(23.57, 31.87, 0, 1);
	tVector right_knee = m * tVector(21.78, 49.07, 0, 1);
	tVector right_ankle = m * tVector(18.34, 58.02, 0, 1);
	tVector right_toe = m * tVector(19.34, 58.02, 0, 1);
	tVector left_knee = m * tVector(21.78, 49.07, 0, 1);
	tVector left_ankle = m * tVector(18.34, 58.02, 0, 1);
	tVector left_toe = m * tVector(19.34, 58.02, 0, 1);
	
	tVector head_root = head - root;
	tVector right_knee_root = right_knee - root;
	tVector right_ankle_knee = right_ankle - right_knee;
	tVector right_toe_ankle = right_toe - right_ankle;
	tVector left_knee_root = left_knee - root;
	tVector left_ankle_knee = left_ankle - left_knee;
	tVector left_toe_ankle = left_toe - left_ankle;

	double root_x = root[0];
	double root_y = root[1];
	double root_theta = std::atan2(-head_root[0], head_root[1]);
	double right_thigh_theta = std::atan2(right_knee_root[1], right_knee_root[0]) + M_PI / 2;
	double right_knee_theta = std::atan2(right_ankle_knee[1], right_ankle_knee[0]) + M_PI / 2;
	double right_ankle_theta = std::atan2(right_toe_ankle[1], right_toe_ankle[0]);
	double left_thigh_theta = std::atan2(left_knee_root[1], left_knee_root[0]) + M_PI / 2;
	double left_knee_theta = std::atan2(left_ankle_knee[1], left_ankle_knee[0]) + M_PI / 2;
	double left_ankle_theta = std::atan2(left_toe_ankle[1], left_toe_ankle[0]);


	double right_thigh_theta_rel = right_thigh_theta - root_theta;
	double right_knee_theta_rel = right_knee_theta - right_thigh_theta;
	double right_ankle_theta_rel = right_ankle_theta - right_knee_theta;
	double left_thigh_theta_rel = left_thigh_theta - root_theta;
	double left_knee_theta_rel = left_knee_theta - left_thigh_theta;
	double left_ankle_theta_rel = left_ankle_theta - left_knee_theta;

	right_thigh_theta_rel = (right_thigh_theta_rel > M_PI) ? -(2 * M_PI - right_thigh_theta_rel) : right_thigh_theta_rel;
	right_knee_theta_rel = (right_knee_theta_rel > M_PI) ? -(2 * M_PI - right_knee_theta_rel) : right_knee_theta_rel;
	right_ankle_theta_rel = (right_ankle_theta_rel > M_PI) ? -(2 * M_PI - right_ankle_theta_rel) : right_ankle_theta_rel;
	left_thigh_theta_rel = (left_thigh_theta_rel > M_PI) ? -(2 * M_PI - left_thigh_theta_rel) : left_thigh_theta_rel;
	left_knee_theta_rel = (left_knee_theta_rel > M_PI) ? -(2 * M_PI - left_knee_theta_rel) : left_knee_theta_rel;
	left_ankle_theta_rel = (left_ankle_theta_rel > M_PI) ? -(2 * M_PI - left_ankle_theta_rel) : left_ankle_theta_rel;
	

	const int num_data = 9;
	std::vector<double> data(num_data);
	{
		int i = 0;
		data[i++] = root_x;
		data[i++] = root_y;
		data[i++] = root_theta;

		data[i++] = right_thigh_theta_rel;
		data[i++] = right_knee_theta_rel;
		data[i++] = right_ankle_theta_rel;
		
		data[i++] = left_thigh_theta_rel;
		data[i++] = left_knee_theta_rel;
		data[i++] = left_ankle_theta_rel;
		
		assert(i == num_data);
	}

	FILE* f = cFileUtil::OpenFile("output/hack_pose_data.txt", "w");

	for (int i = 0; i < num_data; ++i)
	{
		if (i != 0)
		{
			fprintf(f, ",\t");
		}
		fprintf(f, "%.5f", data[i]);
	}

	cFileUtil::CloseFile(f);
}

void InitCaffe()
{
	FLAGS_alsologtostderr = 1;
	int caffe_argc = 1; // hack
	caffe::GlobalInit(&caffe_argc, &gArgv);
}

/*
void HackTest1()
{
	const int x_size = 1;
	const int y_size = 1;

	cNeuralNet net0;
	net0.LoadNet("data/policies/test/nets/dog_actor_deploy.prototxt");
	net0.LoadSolver("data/policies/test/nets/dog_actor_solver.prototxt");
	net0.LoadModel("data/policies/test/models/test_dog_actor_model.prototxt");
	//net0.LoadNet("data/policies/test/nets/weighted_test_deploy.prototxt");
	//net0.LoadSolver("data/policies/test/nets/weighted_test_solver.prototxt");
	//net0.OutputModel("output/test_model.prototxt");

	cNeuralNet net1;
	net1.LoadNet("data/policies/test/nets/dog_actor_deploy.prototxt");
	net1.LoadSolver("data/policies/test/nets/dog_actor_solver.prototxt");
	//net1.LoadNet("data/policies/test/nets/weighted_test_deploy.prototxt");
	//net1.LoadSolver("data/policies/test/nets/weighted_test_solver.prototxt");
	net1.CopyModel(net0);
	
	int batch_size = net0.GetBatchSize();
	int input_size = net0.GetInputSize();
	int output_size = net0.GetOutputSize();

	Eigen::MatrixXd xs = Eigen::MatrixXd::Ones(batch_size, input_size);
	//Eigen::MatrixXd ys = Eigen::MatrixXd::Ones(batch_size, output_size);
	//Eigen::MatrixXd weights = 0.25 * Eigen::MatrixXd::Ones(batch_size, output_size);
	Eigen::MatrixXd ys = Eigen::MatrixXd::Random(batch_size, output_size);
	Eigen::MatrixXd weights = Eigen::MatrixXd::Random(batch_size, output_size).cwiseAbs();

	for (int i = 0; i < weights.size(); ++i)
	{
		//weights.data()[i] = (cMathUtil::FlipCoin()) ? 0 : weights.data()[i];
	}

	Eigen::MatrixXd curr_ys;
	net0.EvalBatch(xs, curr_ys);

	Eigen::MatrixXd d_ys = ys - curr_ys;
	d_ys = d_ys.cwiseProduct(weights);
	Eigen::MatrixXd ys1 = curr_ys + d_ys;

	cNeuralNet::tProblem prob0;
	prob0.mX = xs;
	prob0.mY = ys;
	prob0.mW = weights;
	prob0.mPassesPerStep = 1;
	net0.Train(prob0);

	cNeuralNet::tProblem prob1 = prob0;
	prob1.mY = ys1;
	prob1.mW.setOnes();
	net1.Train(prob1);

	bool net_diff = net0.CompareGrad(net1);
	int xx = 0;
	++xx;
}
*/
/*
void HackTest2()
{
	const int x_size = 1;
	const int y_size = 1;

	cNeuralNet net0;
	net0.LoadNet("data/policies/test/nets/dog_actor_deploy.prototxt");
	net0.LoadSolver("data/policies/test/nets/dog_actor_solver.prototxt");
	
	int batch_size = net0.GetBatchSize();
	int input_size = net0.GetInputSize();
	int output_size = net0.GetOutputSize();

	Eigen::MatrixXd xs = Eigen::MatrixXd::Ones(batch_size, input_size);
	Eigen::MatrixXd weights = 1 * Eigen::MatrixXd::Ones(batch_size, output_size);
	Eigen::MatrixXd ys = Eigen::MatrixXd::Random(batch_size, output_size);
	
	weights.setZero();
	double tar_i = 22;
	double tar_j = 14;
	weights(tar_i, tar_j) = 1;
	double target_val = ys(tar_i, tar_j);

	cNeuralNet::tProblem prob0;
	prob0.mX = xs;
	prob0.mY = ys;
	prob0.mW = weights;
	prob0.mPassesPerStep = 1;

	for (int i = 0; i < 500; ++i)
	{
		net0.Train(prob0);

		Eigen::VectorXd x = Eigen::VectorXd::Ones(input_size);
		Eigen::VectorXd y;
		net0.Eval(x, y);

		int xx = 0;
		++xx;
	}

	Eigen::VectorXd x = Eigen::VectorXd::Ones(input_size);
	Eigen::VectorXd y;
	net0.Eval(x, y);

	double target_test_val = y(tar_j);
	int xx = 0;
	++xx;
}
*/
/*
void testBVHReader()
{
	int mBVHMotionFile;
	cBVHReader bvhReader;
	cMotionDB motionDB;
	cMocapKinController kinModel;

	std::string fileLocation = "D:\\Research\\TerrainRL\\mocapData\\0005_Walking001.bvh";

	bvhReader.parseBVH(fileLocation);
	bvhReader.printData();
	motionDB.processMotionClips(bvhReader);
	kinModel.setMotionDataBase(motionDB);
	std::cout << "Glen put this here and he better remember to remove it..." << std::endl;
	exit(1);
}
*/

int main(int argc, char** argv)
{
	//HackTest();
	//HackTest1();
	//HackTest2();
	//HackDogMeasurements();
	//HackRaptorMeasurements();
	//HackBipedMeasurements();
	// testBVHReader();
	
 	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	InitCaffe();

	glutInit(&gArgc, gArgv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(gWinWidth, gWinHeight);
	glutCreateWindow("Terrain RL");

	InitOpenGl();
	SetupScenario();

	Reshape(gWinWidth, gWinHeight);
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove);
	glutTimerFunc(gDisplayAnimTime, Animate, 0);

	InitTime();
	glutMainLoop();

	return EXIT_SUCCESS;
}

