#pragma once
#include "scenarios/ScenarioImitateTarget.h"
#include "scenarios/ScenarioExpHike.h"

class cScenarioTrainHike : public cScenarioImitateTarget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainHike();
	virtual ~cScenarioTrainHike();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual std::string GetName() const;

protected:

	bool mTrainHLC;
	bool mTrainLLC;

	cCharController::tExpParams mLLCExpParams;
	cCharController::tExpParams mLLCInitExpParams;

	std::string mLLCOutputFile;
	cNeuralNetTrainer::tParams mLLCTrainerParams;
	int mLLCItersPerOutput;
	std::vector<std::shared_ptr<cNeuralNetLearner>> mLLCLearners;
	std::shared_ptr<cTrainerInterface> mLLCTrainer;

	int mHLCEpochIters;
	int mLLCEpochIters;

	virtual void BuildScenePool();
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const;

	virtual void InitTrainer();
	virtual void SetExpMode(cScenarioExpHike::eExpMode mode, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual cScenarioExpHike::eExpMode CalcExpMode(int hlc_iter, int llc_iter) const;

	virtual void BuildLLCTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupLLCTrainerParams(cNeuralNetTrainer::tParams& out_params) const;
	virtual void InitLLCTrainer();
	virtual void InitLLCLearners();
	virtual void SetupLLCLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	
	virtual void SetupLLCTrainerOffsetScale();
	virtual void SetupLLCTrainerCriticOffsetScale();
	virtual void SetupLLCTrainerActorOffsetScale();

	virtual void BuildLLCActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void SetupLLCTrainerActionCovar();
	virtual void SetupLLCTrainerActionBounds();
	virtual void BuildLLCActionCovar(Eigen::VectorXd& out_covar) const;

	virtual void UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done);
	virtual void UpdateExpMode(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void UpdateLLCTrainer(const std::vector<tExpTuple>& tuples, int exp_id);
	virtual void UpdateExpSceneLLCRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual bool IsLLCLearnerDone(int learner_id) const;
	virtual cCharController::tExpParams BuildLLCExpParams(int iter) const;

	virtual std::shared_ptr<cTerrainRLCharController> GetDefaultLLC() const;

	virtual void PrintLearnerInfo(int exp_id) const;
	virtual void PrintLLCLearnerInfo(int exp_id) const;

	virtual void IntOutputCallback() const;
};