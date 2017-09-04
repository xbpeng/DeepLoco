#include "DrawScenarioImitateEval.h"
#include "scenarios/ScenarioImitateEval.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6, 0.65, 0.675, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioImitateEval::cDrawScenarioImitateEval(cCamera& cam)
	: cDrawScenarioPoliEval(cam)
{
	mDrawKinChar = false;
}

cDrawScenarioImitateEval::~cDrawScenarioImitateEval()
{
}

void cDrawScenarioImitateEval::Init()
{
	cDrawScenarioPoliEval::Init();
	mDrawKinChar = false;
}

void cDrawScenarioImitateEval::Reset()
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	if (eval_scene != nullptr)
	{
		eval_scene->EndEpisodeRecord();
	}
	cDrawScenarioPoliEval::Reset();
}

void cDrawScenarioImitateEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioPoliEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'k':
		ToggleDrawKinChar();
		break;
	default:
		break;
	}
}

void cDrawScenarioImitateEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateEval>(new cScenarioImitateEval());
}

void cDrawScenarioImitateEval::DrawCharacters() const
{
	cDrawScenarioPoliEval::DrawCharacters();

	if (mDrawKinChar)
	{
		DrawKinChar();
	}
}

void cDrawScenarioImitateEval::DrawKinChar() const
{
	const auto& kin_char = GetKinChar();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(GetDrawKinCharOffset());
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, GetLineColor());
	cDrawUtil::PopMatrix();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitateEval::GetKinChar() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateEval>(mScene);
	return scene->GetKinChar();
}

void cDrawScenarioImitateEval::ToggleDrawKinChar()
{
	mDrawKinChar = !mDrawKinChar;
	if (mDrawKinChar)
	{
		printf("Enable draw kin character\n");
	}
	else
	{
		printf("Disable draw kin character\n");
	}
}

tVector cDrawScenarioImitateEval::GetDrawKinCharOffset() const
{
	/*
	const auto& kin_char = GetKinChar();
	const auto& sim_char = mScene->GetCharacter();
	tVector kin_pos = kin_char->GetRootPos();
	tVector sim_pos = sim_char->GetRootPos();
	tVector target_pos = sim_pos += tVector(-1, 0, 1, 0);
	target_pos[1] = kin_pos[1];
	return target_pos - kin_pos;
	*/
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		return gKinCharOffset;
	}
	return tVector::Zero();
}

tVector cDrawScenarioImitateEval::GetCamTrackPos() const
{
	return cDrawScenarioPoliEval::GetCamTrackPos();
	//return GetKinChar()->GetRootPos();
}