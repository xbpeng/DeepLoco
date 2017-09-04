#include "DrawScenarioImitate.h"
#include "scenarios/ScenarioImitate.h"
#include "scenarios/ScenarioExpImitate.h"
#include "anim/KinCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6, 0.65, 0.675, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioImitate::cDrawScenarioImitate(cCamera& cam)
	: cDrawScenarioTrainCacla(cam)
{
}

cDrawScenarioImitate::~cDrawScenarioImitate()
{
}

void cDrawScenarioImitate::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
}

void cDrawScenarioImitate::DrawCharacters() const
{
	cDrawScenarioTrainCacla::DrawCharacters();
	DrawKinChar();
}

void cDrawScenarioImitate::DrawKinChar() const
{
	const auto& kin_char = GetKinChar();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(GetDrawKinCharOffset());
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, gLineColor);
	cDrawUtil::PopMatrix();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitate::GetKinChar() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioExpImitate>(mScene);
	return scene->GetKinChar();
}

tVector cDrawScenarioImitate::GetDrawKinCharOffset() const
{
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		return gKinCharOffset;
	}
	return tVector::Zero();
}