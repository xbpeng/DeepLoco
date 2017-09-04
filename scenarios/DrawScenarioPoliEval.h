#pragma once
#include <memory>

#include "DrawScenarioSimChar.h"

class cDrawScenarioPoliEval : public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioPoliEval(cCamera& cam);
	virtual ~cDrawScenarioPoliEval();

	virtual void Keyboard(unsigned char key, int x, int y);

	virtual std::string BuildTextInfoStr() const;

protected:
	
	virtual void BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const;

	virtual void CommandAction(int a);
	virtual void ToggleRecordActions();
	virtual void ToggleRecordCtrlForce();
};