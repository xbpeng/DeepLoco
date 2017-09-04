#pragma once
#include <memory>

#include "DrawScenarioSimChar.h"

class cDrawScenarioTrackMotion : public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrackMotion(cCamera& cam);
	virtual ~cDrawScenarioTrackMotion();
	virtual void Keyboard(unsigned char key, int x, int y);

protected:
	bool mDrawKinChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void ApplyRandForce();

	virtual void CommandAction(int a);
};