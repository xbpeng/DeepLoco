#include "DrawScenarioTerrainRL.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"
#include "render/DrawWorld.h"
#include "render/DrawCharacter.h"
#include "render/DrawPerturb.h"
#include "render/DrawGround.h"
#include "render/Shader.h"
#include "render/TextureDesc.h"
#include "render/ShadowMap.h"
#include "render/GBuffer.h"

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gVisOffset = tVector(0, 0, 0.5, 0); // offset for visualization elements
const tVector gSunDir = tVector(0.6, 0.67, 0.45, 0);

cDrawScenarioTerrainRL::cDrawScenarioTerrainRL(cCamera& cam)
						: cDrawScenario(cam)
{
	mClearFilmStrip = false;
	mCamDelta = cam.GetPosition() - cam.GetFocus();
}

cDrawScenarioTerrainRL::~cDrawScenarioTerrainRL()
{
}

void cDrawScenarioTerrainRL::Init()
{
	cDrawScenario::Init();
	InitRenderResources();
	mClearFilmStrip = false;
	EnableDraw3D(GetDrawMode());
}

void cDrawScenarioTerrainRL::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioTerrainRL::Reset()
{
	cDrawScenario::Reset();
	mClearFilmStrip = false;
}

void cDrawScenarioTerrainRL::Clear()
{
	cDrawScenario::Clear();
}

void cDrawScenarioTerrainRL::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	UpdateScene(time_elapsed);

	tVector prev_cam_focus = mCam.GetFocus();
	UpdateCamera();

	if (mEnableFilmStrip)
	{
		if (!(prev_cam_focus - mCam.GetFocus()).isMuchSmallerThan(0.001))
		{
			mClearFilmStrip = true;
		}
	}
}

tVector cDrawScenarioTerrainRL::GetCamTrackPos() const
{
	return mCam.GetFocus();
}

tVector cDrawScenarioTerrainRL::GetCamStillPos() const
{
	return mCam.GetFocus();
}

void cDrawScenarioTerrainRL::ResetCallback()
{
	ResetCamera();
}

void cDrawScenarioTerrainRL::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case 'c':
		ToggleCamTrackMode(eCamTrackModeStill);
		break;
	default:
		break;
	}
}

void cDrawScenarioTerrainRL::MouseClick(int button, int state, double x, double y)
{
	cDrawScenario::MouseClick(button, state, x, y);
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		mCam.MouseClick(button, state, x, y);
	}
}

void cDrawScenarioTerrainRL::MouseMove(double x, double y)
{
	cDrawScenario::MouseMove(x, y);
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		mCam.MouseMove(x, y);
	}
}

void cDrawScenarioTerrainRL::Reshape(int w, int h)
{
	mIntBufferTex->Reshape(w, h);
}

void cDrawScenarioTerrainRL::SetOutputTex(const std::shared_ptr<cTextureDesc>& tex)
{
	mOutputTex = tex;
}

cDrawScenarioTerrainRL::eDrawMode cDrawScenarioTerrainRL::GetDrawMode() const
{
	return eDrawMode3D;
}

void cDrawScenarioTerrainRL::DrawScene()
{
	mOutputTex->BindBuffer();
	mCam.SetupGLProj();

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		DrawGrid();
	}
	else if (draw_mode == eDrawMode3D)
	{
		DoShadowPass();
		mOutputTex->BindBuffer();
		mLambertShaderMesh->Bind();
		SetupMeshShader3D();

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
	}

	DrawGroundMainScene();
	DrawCharacterMainScene();
	DrawObjsMainScene();
	DrawMiscMainScene();

	if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->Unbind();
	}

	DrawInfo();

	mOutputTex->UnbindBuffer();
}

void cDrawScenarioTerrainRL::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	tVector origin = mCam.GetFocus();
	origin += tVector(0, 0, -1, 0);
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioTerrainRL::DrawGroundMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.5;
	const double enable_tex = 1;
	if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
		mGridTex1->BindTex(GL_TEXTURE0);
	}

	DrawGround();

	if (draw_mode == eDrawMode3D)
	{
		mGridTex1->UnbindTex(GL_TEXTURE0);
	}
}

void cDrawScenarioTerrainRL::DrawCharacterMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawCharacters();
}

void cDrawScenarioTerrainRL::DrawObjsMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawObjs();
}

void cDrawScenarioTerrainRL::DrawMiscMainScene()
{
	eDrawMode draw_mode = GetDrawMode();
	const double roughness = 0.4;
	const double enable_tex = 0;
	if (draw_mode == eDrawMode3D)
	{
		mLambertShaderMesh->SetUniform4(mLambertMaterialDataHandle, tVector(roughness, enable_tex, 0, 0));
	}
	DrawMisc();
}

void cDrawScenarioTerrainRL::DrawGround() const
{
}

void cDrawScenarioTerrainRL::DrawCharacters() const
{
}

void cDrawScenarioTerrainRL::DrawObjs() const
{
}

void cDrawScenarioTerrainRL::DrawMisc() const
{
}

void cDrawScenarioTerrainRL::DrawInfo() const
{
}

void cDrawScenarioTerrainRL::EnableDraw3D(bool enable)
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		mCam.SetProj(cCamera::eProjPerspective);
	}
	else
	{
		mCam.SetProj(cCamera::eProjOrtho);
		mCam.SetPosition(mCam.GetFocus() + mCamDelta);
		mCam.SetUp(tVector(0, 1, 0, 0));
	}
	//ToggleCamTrackMode(eCamTrackModeXYZ);
}

tVector cDrawScenarioTerrainRL::GetVisOffset() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		return tVector::Zero();
	}
	return gVisOffset;
}

tVector cDrawScenarioTerrainRL::GetLineColor() const
{
	eDrawMode draw_mode = GetDrawMode();
	return gLineColor;
}

tVector cDrawScenarioTerrainRL::GetGroundColor() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode3D)
	{
		return tVector::Ones();
	}
	else
	{
		return tVector(151 / 255.0, 151 / 255.0, 151 / 255.0, 1.0);
	}
}

std::string cDrawScenarioTerrainRL::BuildTextInfoStr() const
{
	return "";
}

void cDrawScenarioTerrainRL::Shutdown()
{
}

void cDrawScenarioTerrainRL::InitRenderResources()
{
	bool succ = true;

	{
		mShaderDepth = std::unique_ptr<cShader>(new cShader());
		succ &= mShaderDepth->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/Depth_PS.glsl");
	}

	{
		mLambertShaderMesh = std::unique_ptr<cShader>(new cShader());
		succ &= mLambertShaderMesh->BuildShader("render/shaders/Mesh_VS.glsl", "render/shaders/Lighting_Lambert_PS.glsl");

		mLambertShaderMesh->Bind();
		mLambertShaderMesh->BindShaderUniform(mLambertLightDirHandle, "gLightDir");
		mLambertShaderMesh->BindShaderUniform(mLambertLightColourHandle, "gLightColour");
		mLambertShaderMesh->BindShaderUniform(mLambertAmbientColourHandle, "gAmbientColour");
		mLambertShaderMesh->BindShaderUniform(mLambertShadowProjHandle, "gShadowProj");
		mLambertShaderMesh->BindShaderUniform(mLambertMaterialDataHandle, "gMaterialData");
		mLambertShaderMesh->BindShaderUniform(mLambertFogColorHandle, "gFogColor");
		mLambertShaderMesh->BindShaderUniform(mLambertFogDataHandle, "gFogData");

		GLint albedo_tex = glGetUniformLocation(mLambertShaderMesh->GetProg(), "gTexture");
		glUniform1i(albedo_tex, 0);
		GLint shadow_tex = glGetUniformLocation(mLambertShaderMesh->GetProg(), "gShadowTex");
		glUniform1i(shadow_tex, 1);
		mLambertShaderMesh->Unbind();
	}

	int w = mOutputTex->GetWidth();
	int h = mOutputTex->GetHeight();
	mIntBufferTex = std::unique_ptr<cTextureDesc>(new cTextureDesc(w, h, GL_RGBA16F, GL_RGBA, GL_FLOAT, false));
	
	float shadow_size = 40.f;
	float shadow_near_z = 1.f;
	float shadow_far_z = 60.f;
	int shadow_res = 2048;
	mShadowCam = cCamera(tVector(0, 0, 1, 0), tVector::Zero(), tVector(0, 1, 0, 0),
						shadow_size, shadow_size, shadow_near_z, shadow_far_z);
	mShadowCam.SetProj(cCamera::eProjOrtho);
	mShadowMap = std::unique_ptr<cShadowMap>(new cShadowMap());
	mShadowMap->Init(shadow_res, shadow_res);

	succ &= LoadTextures();
	
	if (!succ)
	{
		printf("Failed to setup render resources\n");
	}
}

bool cDrawScenarioTerrainRL::LoadTextures()
{
	bool succ = true;
	mGridTex0 = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/grid0.png", true));
	succ &= mGridTex0->IsValid();

	mGridTex1 = std::unique_ptr<cTextureDesc>(new cTextureDesc("data/textures/grid1.png", true));
	succ &= mGridTex1->IsValid();

	return succ;
}

void cDrawScenarioTerrainRL::SetupMeshShader3D()
{
	tMatrix view_mat = mCam.BuildWorldViewMatrix();

	tVector cam_focus = mCam.GetFocus();
	tVector cam_pos = mCam.GetPosition();
	double near_dist = mCam.GetNearZ();

	const double fog_cutoff = (cam_focus - cam_pos).norm() - near_dist;
	const double fog_decay = 0.001;
	const tVector ambient_col = tVector(0.6, 0.6, 0.6, 0);
	const tVector light_col = tVector(0.5, 0.5, 0.5, 0);
	const tVector fog_col = tVector(0.97, 0.97, 1, 1);
	const tVector fog_data = tVector(fog_cutoff, fog_decay, 0, 0);

	tVector light_dir = gSunDir;
	light_dir = view_mat * light_dir;

	double w = static_cast<double>(mIntBufferTex->GetWidth());
	double h = static_cast<double>(mIntBufferTex->GetHeight());
	tVector tex_res = tVector(w, h, 0, 0);

	mLambertShaderMesh->SetUniform3(mLambertLightDirHandle, light_dir);
	mLambertShaderMesh->SetUniform3(mLambertLightColourHandle, light_col);
	mLambertShaderMesh->SetUniform3(mLambertAmbientColourHandle, ambient_col);
	mLambertShaderMesh->SetUniform4(mLambertFogColorHandle, fog_col);
	mLambertShaderMesh->SetUniform4(mLambertFogDataHandle, fog_data);

	tMatrix view_world = mCam.BuildViewWorldMatrix();
	tMatrix shadow_view = mShadowCam.BuildWorldViewMatrix();
	tMatrix shadow_proj = mShadowCam.BuildProjMatrix();
	tMatrix shadow_mat = shadow_proj * shadow_view * view_world;

	float shadow_mat_data[] = { (float)shadow_mat(0, 0), (float)shadow_mat(1, 0), (float)shadow_mat(2, 0), (float)shadow_mat(3, 0),
		(float)shadow_mat(0, 1), (float)shadow_mat(1, 1), (float)shadow_mat(2, 1), (float)shadow_mat(3, 1),
		(float)shadow_mat(0, 2), (float)shadow_mat(1, 2), (float)shadow_mat(2, 2), (float)shadow_mat(3, 2),
		(float)shadow_mat(0, 3), (float)shadow_mat(1, 3), (float)shadow_mat(2, 3), (float)shadow_mat(3, 3) };
	glProgramUniformMatrix4fv(mLambertShaderMesh->GetProg(), mLambertShadowProjHandle, 1, false, shadow_mat_data);

	mShadowMap->BindTex(GL_TEXTURE1);
	//mAOTex->BindTex(GL_TEXTURE2);
}

void cDrawScenarioTerrainRL::DoShadowPass()
{
	// front face culling to prevent shelf occlusion
	glCullFace(GL_FRONT);

	//float dist = 10.f;
	float dist = 30.f;
	const tVector& sun_dir = gSunDir;
	tVector delta = dist * sun_dir;
	tVector focus = mCam.GetFocus();
	mShadowCam.SetPosition(focus + delta);
	mShadowCam.SetFocus(focus);

	mShadowMap->BindBuffer();
	mShaderDepth->Bind();
	cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
	cDrawUtil::ClearDepth(1);

	// shadow pass
	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PushMatrix();
	mShadowCam.SetupGLProj();

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PushMatrix();
	mShadowCam.SetupGLView();

	DrawCharacters();
	DrawObjs();
	DrawMisc();
	glCullFace(GL_BACK);
	DrawGround();

	mShaderDepth->Unbind();
	mShadowMap->UnbindBuffer();

	glMatrixMode(GL_PROJECTION);
	cDrawUtil::PopMatrix();

	glMatrixMode(GL_MODELVIEW);
	cDrawUtil::PopMatrix();
}
