#pragma once

#include "learning/NeuralNetTrainer.h"

class cACTrainer : public cNeuralNetTrainer
{
public:
	static std::string GetCriticFilename(const std::string& actor_filename);

	virtual ~cACTrainer();

	virtual void Init(const tParams& params);
	virtual void Clear();
	virtual void Reset();

	virtual int GetIter() const;
	virtual int GetCriticIter() const;
	virtual int GetActorIter() const;

	virtual void SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetCriticInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetActorInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& out_offset, const Eigen::VectorXd& out_scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& out_offset, const Eigen::VectorXd& out_scale);

	virtual void GetInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetOutputOffsetScale(Eigen::VectorXd& offset, Eigen::VectorXd& out_scale) const;
	virtual void GetCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetCriticOutputOffsetScale(Eigen::VectorXd& offset, Eigen::VectorXd& out_scale) const;
	virtual void GetActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetActorOutputOffsetScale(Eigen::VectorXd& offset, Eigen::VectorXd& out_scale) const;

	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual const std::string& GetNetFile() const;
	virtual const std::string& GetSolverFile() const;
	virtual const std::string& GetActorNetFile() const;
	virtual const std::string& GetActorSolverFile() const;
	virtual const std::string& GetCriticNetFile() const;
	virtual const std::string& GetCriticSolverFile() const;

	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;

	virtual const Eigen::VectorXd& GetCriticInputOffset() const;
	virtual const Eigen::VectorXd& GetCriticInputScale() const;
	virtual const Eigen::VectorXd& GetActorInputOffset() const;
	virtual const Eigen::VectorXd& GetActorInputScale() const;

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void OutputModel(const std::string& filename) const;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;
	virtual const std::unique_ptr<cNeuralNet>& GetCritic() const;
	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;

	virtual bool HasInitModel() const;
	virtual bool HasActorInitModel() const;
	virtual bool HasCriticInitModel() const;

	virtual void EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void EvalActor(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void EvalCritic(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void ResetCriticWeights();

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:

	int mActorIter;
	cNeuralNet::tProblem mActorProb;
	std::unique_ptr<cNeuralNet> mActorNet;
	std::vector<int> mActorBatchBuffer;
	
	std::vector<cNeuralNet::eOffsetScaleType> mActorInputOffsetScaleTypes;
	tDataRecord mDataRecordActorX;

	cACTrainer();

	virtual void InitInputOffsetScaleTypes();
	virtual void InitCriticInputOffsetScaleTypes();
	virtual void InitActorInputOffsetScaleTypes();

	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitStage();
	virtual void InitAvgReward();
	virtual void InitDataRecord();
	virtual void SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const;

	virtual void BuildNets();
	virtual void BuildActor(const std::string& net_file, const std::string& solver_file);
	virtual void LoadModels();

	virtual void BuildTupleActorX(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void BuildActorProblemX(const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorTupleXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	
	virtual void FetchActorMinibatch(int batch_size, std::vector<int>& out_batch);

	virtual bool Step();
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y) = 0;
	virtual void BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void ApplySteps(int num_steps);
	
	virtual void UpdateMisc(const std::vector<int>& tuple_ids);
	virtual void UpdateAvgReward(const std::vector<int>& tuple_ids);
	virtual void UpdateDataRecord(const tExpTuple& tuple);
	
	virtual void IncActorIter();
	virtual void UpdateActorNet(const cNeuralNet::tProblem& prob);

	virtual void UpdateCurrActiveNetID();
	virtual void UpdateOffsetScale();
	virtual void UpdateCriticOffsetScale();
	virtual void UpdateActorOffsetScale();

	virtual int GetActorBatchSize() const;

	virtual void UpdateActorBatchBuffer();
	virtual void UpdateCritic();
	virtual void UpdateActor();
	virtual void StepActor();
	virtual void BuildActorProblem(cNeuralNet::tProblem& out_prob);

	virtual void UpdateActorBatchBufferPostStep(int batch_size);

	virtual int GetServerActorID() const;
	virtual void ResetSolvers();
	virtual void ResetActorSolver();

	virtual void OutputIntermediateModel(const std::string& filename) const;
};