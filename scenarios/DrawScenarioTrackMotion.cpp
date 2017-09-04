#include "DrawScenarioTrackMotion.h"
#include "ScenarioTrackMotion.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);

cDrawScenarioTrackMotion::cDrawScenarioTrackMotion(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
	mDrawKinChar = false;
}

cDrawScenarioTrackMotion::~cDrawScenarioTrackMotion()
{
}

void cDrawScenarioTrackMotion::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimChar::Keyboard(key, x, y);

	if (key >= '1' && key <= '9')
	{
		CommandAction(key - '1');
	}
	else
	{
		switch (key)
		{
		case 'd':
			mDrawKinChar = !mDrawKinChar;
			break;
		case 'b':
			ApplyRandForce();
			break;
		default:
			break;
		}
	}
}

void cDrawScenarioTrackMotion::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::unique_ptr<cScenarioSimChar>(new cScenarioTrackMotion());
}

void cDrawScenarioTrackMotion::DrawCharacters() const
{
	cDrawScenarioSimChar::DrawCharacters();
	if (mDrawKinChar)
	{
		std::shared_ptr<cScenarioTrackMotion> track_scene = std::static_pointer_cast<cScenarioTrackMotion>(mScene);
		const cKinCharacter& character = track_scene->GetKinCharacter();
		cDrawCharacter::Draw(character, gLinkWidth, gFilLColor, gLineColor);
	}
}

void cDrawScenarioTrackMotion::ApplyRandForce()
{
	std::shared_ptr<cScenarioTrackMotion> track_scene = std::static_pointer_cast<cScenarioTrackMotion>(mScene);
	track_scene->ApplyRandForce();
}

void cDrawScenarioTrackMotion::CommandAction(int a)
{
	std::shared_ptr<cScenarioSimChar> sim_scene = std::static_pointer_cast<cScenarioSimChar>(mScene);
	const auto& character = sim_scene->GetCharacter();
	const std::shared_ptr<cCharController>& ctrl = character->GetController();
	ctrl->CommandAction(a);
}
