#pragma once
#include <memory>

#include "DrawScenarioSimChar.h"
#include "scenarios/ScenarioTrain.h"

class cDrawScenarioTrain: public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrain(cCamera& cam);
	virtual ~cDrawScenarioTrain();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Shutdown();

	virtual std::string GetName() const;

protected:
	std::shared_ptr<cScenarioTrain> mTrain;

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
	virtual void SetupTrainScene(std::shared_ptr<cScenarioTrain>& out_train_scene, std::shared_ptr<cScenarioSimChar>& out_scene);
	virtual void ToggleTraining();

	virtual void UpdateScene(double time_elapsed);
};