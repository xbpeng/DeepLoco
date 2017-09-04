#pragma once
#include <memory>
#include <mutex>
#include "learning/TrainerInterface.h"
#include "learning/ExpTuple.h"
#include "learning/NeuralNet.h"
#include "learning/NeuralNetLearner.h"
#include "learning/ExpBuffer.h"

//#define DISABLE_EXP_REPLAY

class cNeuralNetTrainer : public cTrainerInterface, 
						public std::enable_shared_from_this<cNeuralNetTrainer>
{
public:
	enum eStage
	{
		eStageInit,
		eStageTrain,
		eStageMax
	};

	static double CalcDiscountNorm(double discount);
	
	cNeuralNetTrainer();
	virtual ~cNeuralNetTrainer();

	virtual void Init(const tParams& params);
	virtual void Clear();
	virtual void LoadModel(const std::string& model_file);
	virtual void LoadScale(const std::string& scale_file);
	virtual void Reset();
	virtual void EndTraining();

	virtual int AddTuple(const tExpTuple& tuple, int prev_id, int learner_id);
	virtual int AddTuples(const std::vector<tExpTuple>& tuples, int prev_id, int learner_id);
	virtual int IncBufferHead(int head) const;

	virtual void Train();

	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;
	virtual double GetDiscount() const;
	virtual double GetAvgReward() const;
	virtual int GetIter() const;
	virtual double NormalizeReward(double r) const;

	virtual void SetNumInitSamples(int num);
	virtual void SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void GetInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void GetOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual int GetNumInitSamples() const;
	virtual const std::string& GetNetFile() const;
	virtual const std::string& GetSolverFile() const;

	virtual eStage GetStage() const;
	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual int GetInputSize() const;
	virtual int GetOutputSize() const;
	virtual int GetBatchSize() const;
	virtual int GetNumTuplesPerBatch() const;
	virtual void ResetExpBuffer();

	virtual int GetNumTuples() const;
	virtual void OutputModel(const std::string& filename) const;

	virtual bool HasInitModel() const;
	virtual void EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);
	virtual int RegisterLearner(cNeuralNetLearner* learner);
	virtual void UnregisterLearner(cNeuralNetLearner* learner);
	virtual void SetIntOutputCallback(tCallbackFunc func);
	virtual void OutputIntermediate();
	virtual void OutputIntermediateModel(const std::string& filename) const;

	virtual void Lock();
	virtual void Unlock();

	virtual bool IsDone() const;

protected:
	struct tDataRecord
	{
		Eigen::VectorXd mMin;
		Eigen::VectorXd mMax;
		Eigen::VectorXd mMean;
		Eigen::VectorXd mMeanSquares;
		int mCount;

		void Init(int size);
		void Update(const Eigen::VectorXd data);
		void CalcVar(Eigen::VectorXd& out_var) const;
		void CalcOffsetScale(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types, double max_scale,
							Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
		int GetSize() const;
	};

	eStage mStage;
	tParams mParams;
	int mIter;
	bool mDone;
	std::unique_ptr<cExpBuffer> mExpBuffer;

	std::vector<cNeuralNet::eOffsetScaleType> mInputOffsetScaleTypes;
	tDataRecord mDataRecordX;

	cNeuralNet::tProblem mProb;
	std::vector<std::unique_ptr<cNeuralNet>> mNetPool;
	int mCurrActiveNet;
	std::vector<int> mBatchBuffer;
	double mAvgReward;

	std::mutex mLock;
	std::vector<cNeuralNetLearner*> mLearners;

	tCallbackFunc mIntOutputCallback;

	const std::unique_ptr<cNeuralNet>& GetCurrNet() const;

	virtual void BuildExpBuffer(std::unique_ptr<cExpBuffer>& out_exp_buffer) const;
	virtual void InitExpBuffer(int buffer_size);
	virtual void SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const;

	virtual void InitInputOffsetScaleTypes();
	virtual void InitBatchBuffer();
	virtual void InitProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitDataRecord();
	virtual int GetPlaybackMemSize() const;
	virtual void ResetParams();

	virtual int GetProblemXSize() const;
	virtual int GetProblemYSize() const;

	virtual void Pretrain();
	virtual bool Step();
	virtual bool BuildProblem(int net_id, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	// virtual void BuildProblemStateAndAction(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	// virtual void BuildProblemNextState(int net_id, const std::vector<int>& tuple_ids,
		// const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void UpdateMisc(const std::vector<int>& tuple_ids);
	virtual void UpdateDataRecord(const tExpTuple& tuple);

	virtual void BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void BuildTupleNextState(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void FetchMinibatch(int size, std::vector<int>& out_batch);

	virtual int GetTargetNetID(int net_id) const;
	virtual void UpdateCurrActiveNetID();
	virtual const std::unique_ptr<cNeuralNet>& GetTargetNet(int net_id) const;
	virtual void UpdateNet(int net_id, const cNeuralNet::tProblem& prob);

	virtual tExpTuple GetTuple(int t) const;

	virtual const Eigen::VectorXd& GetInputOffset() const;
	virtual const Eigen::VectorXd& GetInputScale() const;
	virtual void UpdateOffsetScale();
	virtual void UpdateStage();
	virtual void InitStage();
	virtual void ApplySteps(int num_steps);
	virtual void IncIter();

	virtual int GetNetPoolSize() const;
	virtual void BuildNets();
	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual int GetPoolSize() const;
	virtual void LoadModels();

	virtual bool EnableIntOutput() const;

	virtual int GetNumLearners() const;
	virtual void ResetLearners();
	virtual void ResetSolvers();

	virtual void OutputTuple(const tExpTuple& tuple, const std::string& out_file) const;
};