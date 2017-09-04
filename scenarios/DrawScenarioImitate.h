#pragma once
#include "DrawScenarioTrainCacla.h"

class cKinCharacter;

class cDrawScenarioImitate: public cDrawScenarioTrainCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioImitate(cCamera& cam);
	virtual ~cDrawScenarioImitate();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawKinChar() const;
	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual tVector GetDrawKinCharOffset() const;
};