#include "DrawScenarioSimChar.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"
#include "render/DrawWorld.h"
#include "render/DrawCharacter.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawPerturb.h"
#include "render/DrawGround.h"
#include "render/DrawMesh.h"

const int gTracerBufferSize = 2000;
const double gTracerSamplePeriod = 1 / 30.0;
const size_t  gInitGroundUpdateCount = std::numeric_limits<size_t>::max();
const tVector gFillTint = tVector(1, 1, 1, 1);

const std::string gOutputCharFile = "output/char_state.txt";

cDrawScenarioSimChar::cDrawScenarioSimChar(cCamera& cam)
					: cDrawScenarioSimInteractive(cam)
{
	mDrawInfo = false;
	mDrawPoliInfo = false;
	mEnableTrace = false;
	mDrawFeatures = false;
	mDrawPolicyPlots = false;
	mEnableCharDrawShapes = false;
	mPauseSim = false;
	mPrevGroundUpdateCount = gInitGroundUpdateCount;
}

cDrawScenarioSimChar::~cDrawScenarioSimChar()
{
}

void cDrawScenarioSimChar::Init()
{
	BuildScene(mScene);
	SetupScene(mScene);

	cDrawScenarioSimInteractive::Init();
	
	InitTracer();
	mPrevGroundUpdateCount = gInitGroundUpdateCount;
	BuildGroundDrawMesh();
}

void cDrawScenarioSimChar::Reset()
{
	mScene->Reset();
	cDrawScenarioSimInteractive::Reset();
	mTracer.Reset();
}

void cDrawScenarioSimChar::Clear()
{
	cDrawScenarioSimInteractive::Clear();
	mScene->Clear();
	mTracer.Clear();
	mPrevGroundUpdateCount = gInitGroundUpdateCount;
}

void cDrawScenarioSimChar::Update(double time_elapsed)
{
	if (!mPauseSim)
	{
		cDrawScenarioSimInteractive::Update(time_elapsed);

		if (mEnableTrace)
		{
			UpdateTracer(time_elapsed);
		}

		UpdateGroundDrawMesh();
	}
}

void cDrawScenarioSimChar::UpdateTracer(double time_elapsed)
{
	auto ctrl = mScene->GetCharacter()->GetController();
	mTracer.Update(time_elapsed);
}

void cDrawScenarioSimChar::UpdateGroundDrawMesh()
{
	const auto& ground = mScene->GetGround();
	size_t update_count = ground->GetUpdateCount();
	if (update_count != mPrevGroundUpdateCount)
	{
		const auto& ground = mScene->GetGround();
		cDrawGround::BuildMesh(ground.get(), mGroundDrawMesh.get());
		mPrevGroundUpdateCount = ground->GetUpdateCount();
	}
}

void cDrawScenarioSimChar::ResetCallback()
{
	cDrawScenarioSimInteractive::ResetCallback();
	mTracer.Reset();
	BuildGroundDrawMesh();
}

void cDrawScenarioSimChar::PauseSim(bool pause)
{
	mPauseSim = pause;
}

void cDrawScenarioSimChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimInteractive::Keyboard(key, x, y);

	switch (key)
	{
	case 'f':
		mDrawInfo = !mDrawInfo;
		break;
	case 'g':
		mDrawFeatures = !mDrawFeatures;
		break;
	case 'h':
		mDrawPolicyPlots = !mDrawPolicyPlots;
		break;
	case 'p':
		mDrawPoliInfo = !mDrawPoliInfo;
		break;
	case 'x':
		SpawnProjectile();
		break;
	case 'y':
		ToggleTrace();
		break;
	case 's':
		OutputCharState(GetOutputCharFile());
		break;
	case 'z':
		SpawnBigProjectile();
		break;
	case 'v':
		EnableCharDrawShapes(!mEnableCharDrawShapes);
		break;
	default:
		break;
	}
}

void cDrawScenarioSimChar::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioSimChar>(new cScenarioSimChar());
}

void cDrawScenarioSimChar::SetupScene(std::shared_ptr<cScenarioSimChar>& out_scene)
{
	out_scene->ParseArgs(mArgParser);
	out_scene->Init();
	tCallbackFunc func = std::bind(&cDrawScenarioSimChar::ResetCallback, this);
	out_scene->SetResetCallback(func);
}

void cDrawScenarioSimChar::UpdateScene(double time_elapsed)
{
	cDrawScenarioSimInteractive::UpdateScene(time_elapsed);
	mScene->Update(time_elapsed);
}

tVector cDrawScenarioSimChar::GetCamTrackPos() const
{
	const auto& character = mScene->GetCharacter();
	return character->CalcCOM();
}

tVector cDrawScenarioSimChar::GetCamStillPos() const
{
	const auto& character = mScene->GetCharacter();
	tVector char_pos = character->CalcCOM();
	
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();
	const auto& ground = mScene->GetGround();

	const int num_samples = 16;
	double ground_samples[num_samples] = { 0 };
	const double pad = std::min(0.5, 0.5 * cam_w);

	double avg_h = 0;

	double min_x = char_pos[0];
	double max_x = char_pos[0] + cam_w;

	int num_valid_samples = 0;
	for (int i = 0; i < num_samples; ++i)
	{
		tVector pos = char_pos;
		pos[0] = static_cast<double>(i) / (num_samples - 1) * (max_x - min_x) + min_x;

		bool valid_sample = true;
		double ground_h = ground->SampleHeight(pos, valid_sample);
		if (valid_sample)
		{
			ground_samples[i] = ground_h;
			avg_h += ground_h;
			++num_valid_samples;
		}
	}
	avg_h /= num_valid_samples;

	std::sort(ground_samples, &(ground_samples[num_samples - 1]));
	double med_h = ground_samples[num_samples / 2];
	double min_h = ground_samples[0];

	tVector track_pos = char_pos;
	double target_h = avg_h;
	//double target_h = min_h;

	//double y_pad = -0.2;
	double y_pad = -0.4;
	track_pos[1] = target_h + y_pad + 0.5 * cam_h;

	return track_pos;
}

void cDrawScenarioSimChar::InitTracer()
{
	mTraceHandles.clear();
	mTracer.Init(gTracerBufferSize, gTracerSamplePeriod);

	tVectorArr tracer_cols;
	tracer_cols.push_back(tVector(0, 0, 1, 0.5));
	tracer_cols.push_back(tVector(1, 0, 0, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0, 0.5));
	tracer_cols.push_back(tVector(0.75, 0, 0.75, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0.5, 0.5));
	tracer_cols.push_back(tVector(0, 0, 0, 0.5));
	int handle = AddCharTrace(mScene->GetCharacter(), tracer_cols);
	mTraceHandles.push_back(handle);
}

int cDrawScenarioSimChar::AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
										const tVectorArr& cols)
{
	cCharTracer::tParams params;
	params.mChar = character;
	params.mColors = cols;
	params.mType = cCharTracer::eTraceCOM;

	for (int i = 0; i < character->GetNumBodyParts(); ++i)
	{
		if (character->IsValidBodyPart(i)
			&& character->IsEndEffector(i))
		{
			params.mContactList.push_back(i);
		}
	}

	int handle = mTracer.AddTrace(params);
	return handle;
}

void cDrawScenarioSimChar::ToggleTrace()
{
	mTracer.Reset();
	mEnableTrace = !mEnableTrace;
}

cDrawScenarioSimChar::eDrawMode cDrawScenarioSimChar::GetDrawMode() const
{
	cWorld::eSimMode sim_mode = mScene->GetSimMode();
	eDrawMode draw_mode = eDrawMode3D;
	switch (sim_mode)
	{
	case cWorld::eSimMode2D:
		draw_mode = eDrawMode2D;
		break;
	case cWorld::eSimMode3D:
		draw_mode = eDrawMode3D;
		break;
	default:
		break;
	}
	return draw_mode;
}

void cDrawScenarioSimChar::DrawGround() const
{
	const auto& ground = mScene->GetGround();
	
	tVector focus = mCam.GetFocus();
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();

	tVector ground_col = GetGroundColor();
	cDrawUtil::SetColor(ground_col);
	mGroundDrawMesh->Draw();

	if (ground->GetGroundClass() == cGround::eClassDynamicObstacles3D
		|| ground->GetGroundClass() == cGround::eClassConveyor3D)
	{
		DrawGroundDynamicObstacles3D(ground);
	}

	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		tVector bound_min = focus - tVector(cam_w, cam_h, 0, 0) * 0.5;
		tVector bound_max = focus + tVector(cam_w, cam_h, 0, 0) * 0.5;
		cDrawGround::DrawRuler2D(ground.get(), bound_min, bound_max);
	}
}

void cDrawScenarioSimChar::DrawCharacters() const
{
	const auto& character = mScene->GetCharacter();
	DrawCharacter(character);
}

void cDrawScenarioSimChar::DrawCharacter(const std::shared_ptr<cSimCharacter>& character) const
{
	eDrawMode draw_mode = GetDrawMode();
	bool enable_draw_shape = mEnableCharDrawShapes;
	cDrawSimCharacter::Draw(*(character.get()), gFillTint, GetLineColor(), enable_draw_shape);
}

void cDrawScenarioSimChar::DrawTrace() const
{
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(GetVisOffset());
	mTracer.Draw();
	cDrawUtil::PopMatrix();
}

void cDrawScenarioSimChar::DrawObjs() const
{
	const auto& obj_entries = mScene->GetObjs();
	for (size_t i = 0; i < obj_entries.size(); ++i)
	{
		const cScenarioSimChar::tObjEntry& entry = obj_entries[i];
		const auto& obj = entry.mObj;
		cDrawUtil::SetColor(entry.mColor);
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

		tVector line_col = GetLineColor();
		if (line_col[3] > 0)
		{
			cDrawUtil::SetColor(line_col);
			cDrawObj::Draw(obj.get(), cDrawUtil::eDrawWire);
		}
	}
}

void cDrawScenarioSimChar::DrawMisc() const
{
	DrawPerturbs();
}

void cDrawScenarioSimChar::DrawInfo() const
{
	if (mDrawInfo)
	{
		DrawCoM();
		DrawTorque();
		DrawHeading();
		DrawCtrlInfo();
	}

	if (mDrawPoliInfo)
	{
		DrawPoliInfo();
	}

	if (mEnableTrace)
	{
		DrawTrace();
	}

	if (mDrawFeatures)
	{
		DrawFeatures();
	}

	if (mDrawPolicyPlots)
	{
		DrawPolicyPlots();
	}
}

void cDrawScenarioSimChar::EnableDraw3D(bool enable)
{
	cDrawScenarioSimInteractive::EnableDraw3D(enable);

	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		EnableCharDrawShapes(false);
	}
	else
	{
		EnableCharDrawShapes(false);
	}
}

void cDrawScenarioSimChar::DrawCoM() const
{
	const tVector col = tVector(0, 1, 0, 0.5);
	const double marker_size = 0.1;
	const double vel_scale = 0.1;
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawCoM(*(character.get()), marker_size, vel_scale, col, GetVisOffset());
}

void cDrawScenarioSimChar::DrawTorque() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawTorque(*(character.get()), GetVisOffset());
}

void cDrawScenarioSimChar::DrawHeading() const
{
	const auto& character = mScene->GetCharacter();
	double arrow_size = 0.2;
	tVector arrow_col = tVector(0, 0.8, 0, 0.5);
	cDrawCharacter::DrawHeading(*(character.get()), arrow_size, arrow_col, GetVisOffset());
}

void cDrawScenarioSimChar::DrawCtrlInfo() const
{
	const auto& character = mScene->GetCharacter();
	const auto& ground = mScene->GetGround();
	eDrawMode draw_mode = GetDrawMode();
	bool draw_3d = (draw_mode == eDrawMode3D);
	cDrawSimCharacter::DrawCtrlInfo(character->GetController().get(), ground.get(), GetVisOffset(), draw_3d);
}

void cDrawScenarioSimChar::DrawPoliInfo() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawPoliInfo(character->GetController().get(), mCam);
}

void cDrawScenarioSimChar::DrawFeatures() const
{
	const double marker_size = 0.05;
	const double vel_scale = 0.025;
	const tVector pos_col = tVector(1, 0, 0, 0.5);
	const tVector vel_col = tVector(0, 0.75, 0, 0.5);
	const tVector terrain_col = tVector(0, 0, 1, 0.5);
	const auto& character = mScene->GetCharacter();
	const auto& ground = mScene->GetGround();

	cDrawSimCharacter::DrawCharFeatures(*(character.get()), *ground.get(),
		marker_size, vel_scale, pos_col, vel_col, GetVisOffset());
	cDrawSimCharacter::DrawTerainFeatures(*(character.get()), marker_size, terrain_col, GetVisOffset());
}

void cDrawScenarioSimChar::DrawPolicyPlots() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawPolicyPlots(character->GetController().get(), mCam);
}

void cDrawScenarioSimChar::DrawGroundDynamicObstacles3D(const std::shared_ptr<cGround>& ground) const
{
	const tVector fill_col = tVector(0.75, 0.75, 0.75, 1.0);
	const tVector line_col = GetLineColor();

	auto obstacles3d = std::dynamic_pointer_cast<cGroundDynamicObstacles3D>(ground);
	int num_obstacles = obstacles3d->GetNumObstacles();
	for (int i = 0; i < num_obstacles; ++i)
	{
		const auto& obj = obstacles3d->GetObj(i);

		cDrawUtil::SetColor(fill_col);

		auto box = dynamic_cast<const cSimBox*>(&obj);
		if (box != nullptr)
		{
			tVector box_size = box->GetSize();
			tVector tex_coord_min = -box_size;
			tVector tex_coord_max = box_size;
			tex_coord_min[1] = 0;
			tex_coord_max[1] = 0;
			cDrawObj::DrawBox(box, tex_coord_min, tex_coord_max);
		}
		else
		{
			cDrawObj::Draw(&obj, cDrawUtil::eDrawSolid);
		}

		if (line_col[3] > 0)
		{
			cDrawUtil::SetColor(line_col);
			cDrawObj::Draw(&obj, cDrawUtil::eDrawWire);
		}
	}
}

void cDrawScenarioSimChar::DrawPerturbs() const
{
	const auto& world = mScene->GetWorld();
	cDrawWorld::DrawPerturbs(*world.get());
}

std::string cDrawScenarioSimChar::BuildTextInfoStr() const
{
	const auto& character = mScene->GetCharacter();
	double time = mScene->GetTime();
	tVector com = character->CalcCOM();
	tVector com_vel = character->CalcCOMVel();
	const auto& ctrl = character->GetController();

	char buffer[256];
#ifdef _LINUX_
	sprintf(buffer, "Time: %.2fs\nPosition: (%.2f, %.2f, %.2f)\nVelocity: (%.2f, %.2f, %.2f)\n",
				time, com[0], com[1], com[2], com_vel[0], com_vel[1], com_vel[2]);
#else
	sprintf_s(buffer, "Time: %.2fs\nPosition: (%.2f, %.2f, %.2f)\nVelocity: (%.2f, %.2f, %.2f)\n",
					time, com[0], com[1], com[2], com_vel[0], com_vel[1], com_vel[2]);
#endif
	
	std::string str(buffer);
	if (ctrl != nullptr)
	{
		std::string ctrl_str = ctrl->BuildTextInfoStr();
		str += ctrl_str;
	}
	
	return str;
}

void cDrawScenarioSimChar::EnableCharDrawShapes(bool enable)
{
	mEnableCharDrawShapes = enable;
	if (mEnableCharDrawShapes)
	{
		printf("Char Draw Shapes Enabled\n");
	}
	else
	{
		printf("Char Draw Shapes Disabled\n");
	}
}

void cDrawScenarioSimChar::Shutdown()
{
	mScene->Shutdown();
}

std::string cDrawScenarioSimChar::GetName() const
{
	return mScene->GetName();
}

const std::shared_ptr<cScenarioSimChar>& cDrawScenarioSimChar::GetScene() const
{
	return mScene;
}

std::string cDrawScenarioSimChar::GetOutputCharFile() const
{
	return gOutputCharFile;
}

void cDrawScenarioSimChar::OutputCharState(const std::string& out_file) const
{
	mScene->OutputCharState(out_file);
}

void cDrawScenarioSimChar::SpawnProjectile()
{
	mScene->SpawnProjectile();
}

void cDrawScenarioSimChar::SpawnBigProjectile()
{
	mScene->SpawnBigProjectile();
}

void cDrawScenarioSimChar::BuildGroundDrawMesh()
{
	mGroundDrawMesh = std::unique_ptr<cDrawMesh>(new cDrawMesh());
	mGroundDrawMesh->Init(1);

	const auto& ground = mScene->GetGround();
	cDrawGround::BuildMesh(ground.get(), mGroundDrawMesh.get());
	mPrevGroundUpdateCount = ground->GetUpdateCount();
}