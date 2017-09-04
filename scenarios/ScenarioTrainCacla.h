#pragma once

#include "scenarios/ScenarioTrain.h"

class cStochPGTrainer;

class cScenarioTrainCacla : public cScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainCacla();
	virtual ~cScenarioTrainCacla();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void ResetCriticWeights();

	virtual std::string GetName() const;

protected:
	enum eCaclaTrainerType
	{
		eCaclaTrainerTypeVal,
		eCaclaTrainerTypeMax
	};

	eCaclaTrainerType mTrainerType;

	virtual void ParseTrainerType(const std::string& str, eCaclaTrainerType& out_mode) const;
	virtual void InitTrainer();
	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainer(const std::shared_ptr<cTrainerInterface>& out_trainer);

	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;

	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerCriticOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerActorOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);

	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void SetupTrainerActionCovar();
	virtual void SetupTrainerActionBounds();
	virtual void BuildActionCovar(Eigen::VectorXd& out_covar) const;
};