#pragma once

#include "scenarios/Scenario.h"
#include "scenarios/ScenarioExp.h"
#include "learning/NeuralNetTrainer.h"
#include <mutex>

class cScenarioTrain : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrain();
	virtual ~cScenarioTrain();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void Run();

	virtual void Update(double time_elapsed);

	virtual void SetExpPoolSize(int size);
	virtual int GetPoolSize() const;
	virtual const std::shared_ptr<cScenarioExp>& GetExpScene(int i) const;
	virtual bool LoadControlParams(const std::string& param_file);

	virtual bool TrainingComplete() const;
	virtual void EnableTraining(bool enable);
	virtual void ToggleTraining();
	virtual bool TrainingEnabled() const;
	virtual int GetIter() const;

	virtual const std::string& GetOutputFile() const;
	virtual void SetOutputFile(const std::string& file);
	virtual cNeuralNetTrainer::tParams& GetTrainerParams();
	virtual const cNeuralNetTrainer::tParams& GetTrainerParams() const;

	virtual void SetTimeStep(double time_step);
	virtual bool IsDone() const;
	virtual void Shutdown();

	virtual void OutputModel() const;
	virtual void OutputModel(const std::string& out_file) const;
	virtual std::vector<std::string>& GetNetFiles();
	virtual const std::vector<std::string>& GetNetFiles() const;

	virtual std::string GetName() const;

protected:

	std::vector<std::string> mNetFiles;
	cNeuralNetTrainer::tParams mTrainerParams;
	int mMaxIter;
	int mExpPoolSize;
	bool mEnableTraining;
	
	int mNumAnnealIters;
	int mNumBaseAnnealIters;

	int mNumCurriculumIters;
	int mNumCurriculumStageIters;
	double mNumCurriculumInitExpScale;

	double mTimeStep; // used for Run()

	cCharController::tExpParams mExpParams;
	cCharController::tExpParams mInitExpParams;

	std::vector<std::shared_ptr<cScenarioExp>> mExpPool;
	std::vector<std::shared_ptr<cNeuralNetLearner>> mLearners;
	std::shared_ptr<cTrainerInterface> mTrainer;
	std::shared_ptr<cArgParser> mArgParser;

	std::string mOutputFile;
	int mItersPerOutput;

	virtual void BuildScenePool();
	virtual void ClearScenePool();
	virtual void ResetScenePool();
	
	virtual void BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void SetupExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual const std::shared_ptr<cCharController>& GetDefaultController() const;

	virtual void SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const;
	virtual void InitTrainer();
	virtual void InitLearners();
	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	
	virtual const std::shared_ptr<cCharController>& GetRefController() const;

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainer(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual bool IsLearnerDone(int learner_id) const;

	virtual void UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id);
	virtual void UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp);
	virtual void UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done);
	virtual void UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const;
	virtual void UpdateSceneCurriculum(double phase, std::shared_ptr<cScenarioExp>& out_exp) const;
	
	virtual cCharController::tExpParams BuildExpParams(int iter) const;
	virtual double CalcCurriculumPhase(int iter) const;

	virtual void PrintLearnerInfo(int exp_id) const;

	virtual bool EnableCurriculum() const;

	virtual void ExpHelper(std::shared_ptr<cScenarioExp> exp, int exp_id);
};