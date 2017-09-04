#pragma once

#include "learning/ACTrainer.h"

class cCaclaTrainer : public cACTrainer
{
public:
	
	cCaclaTrainer();
	virtual ~cCaclaTrainer();

	virtual void Init(const tParams& params);
	virtual void Clear();
	virtual void Reset();
	virtual int AddTuple(const tExpTuple& tuple, int prev_id, int learner_id);

	virtual void SetActionBounds(const Eigen::VectorXd& action_min, const Eigen::VectorXd& action_max);
	virtual void SetActionCovar(const Eigen::VectorXd& action_covar);
	virtual void ResetCriticWeights();

protected:
	
	std::vector<int> mOffPolicyBuffer;

	Eigen::MatrixXd mBatchXBuffer;
	Eigen::MatrixXd mBatchYBuffer;
	Eigen::VectorXd mBatchValBuffer0;
	Eigen::VectorXd mBatchValBuffer1;

	std::vector<double> mActorBatchTDBuffer;

	bool mHasActionBounds;
	Eigen::VectorXd mActionMin;
	Eigen::VectorXd mActionMax;
	Eigen::VectorXd mActionCovar;

	virtual void InitBatchBuffers();
	virtual void InitProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitActionBounds();

	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual void FetchActorMinibatch(int batch_size, std::vector<int>& out_batch);

	virtual void Pretrain();

	virtual bool Step();
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual int GetTargetNetID(int net_id) const;
	virtual const std::unique_ptr<cNeuralNet>& GetCriticTarget() const;

	virtual double CalcCurrCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net);
	virtual double CalcNewCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net);
	virtual void CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	virtual void CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemYCacla(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemYTD(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	
	virtual int GetPoolSize() const;
	virtual void UpdateActorBatchBuffer();
	virtual void UpdateActorBatchBufferPostStep(int batch_size);

	virtual bool EnableTargetNet() const;
	virtual bool CheckUpdateTarget(int iter) const;
	virtual void UpdateTargetNet();
	virtual void SyncTargetNets();

	virtual void UpdateBuffers(int t);
	virtual bool IsOffPolicy(int t) const;

	virtual ePGMode GetPGMode() const;
	virtual bool CheckActorTD(double td) const;
	virtual void ProcessPoliGrad(const Eigen::VectorXd& action, double scale, Eigen::VectorXd& out_diff) const;
	virtual double CalcAdvantage(double td) const;
	virtual double CalcLogp(const Eigen::VectorXd& mean_action, const Eigen::VectorXd& sample_action) const;
};