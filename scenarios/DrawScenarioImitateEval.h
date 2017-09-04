#pragma once
#include "DrawScenarioPoliEval.h"

class cKinCharacter;
class cDrawScenarioImitateEval : public cDrawScenarioPoliEval
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitateEval(cCamera& cam);
	virtual ~cDrawScenarioImitateEval();

	virtual void Init();
	virtual void Reset();
	virtual void Keyboard(unsigned char key, int x, int y);

protected:

	bool mDrawKinChar;

	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawKinChar() const;
	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual void ToggleDrawKinChar();
	virtual tVector GetDrawKinCharOffset() const;

	virtual tVector GetCamTrackPos() const;
};