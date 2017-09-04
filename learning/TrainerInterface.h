#pragma once
#include <string>
#include "NeuralNet.h"
#include "NeuralNetLearner.h"

class cTrainerInterface
{
public:

	typedef std::function<void()> tCallbackFunc;

	enum eRewardMode
	{
		eRewardModeStart,
		eRewardModeAvg,
		eRewardModeMax
	};

	// policy gradient mode
	enum ePGMode
	{
		ePGModeCacla,
		ePGModeTD,
		ePGModePTD,
		ePGModeMax
	};

	struct tParams
	{
		std::string mNetFile;
		std::string mSolverFile;
		std::string mModelFile;
		std::string mCriticNetFile;
		std::string mCriticSolverFile;
		std::string mCriticModelFile;
		std::string mForwardDynamicsNetFile;
		std::string mForwardDynamicsSolverFile;
		std::string mForwardDynamicsModelFile;

		int mNumThreads;
		bool mEnableExpReplay;
		int mPlaybackMemSize;
		int mPoolSize;
		int mNumInitSamples;
		int mNumStepsPerIter;
		int mFreezeTargetIters; // for deep q learning
		int mPretrainIters;
		double mDiscount;

		bool mInitInputOffsetScale;
		double mInputScaleMax;

		eRewardMode mRewardMode;
		double mAvgRewardStep;

		int mIntOutputIters;
		std::string mIntOutputFile;
		
		// policy grad params
		ePGMode mPGMode;
		bool mPGEnableOnPolicy;
		bool mPGEnableImportanceSampling;
		double mPGAdvScale;
		double mPGIWClip;
		double mPGAdvClip;

		// td lambda params
		bool mEnableTDLambda;
		int mNumRewardSteps;
		double mTDLambda;

		// entropy params
		int mNumEntropySamples;
		double mEntropyWeight;
		double mEntropyKernelWidth;

		tParams();
	};
	
	static void ParsePGMode(const std::string& mode_str, ePGMode& out_mode);

	virtual ~cTrainerInterface();
	virtual void Init(const tParams& params) = 0;
	virtual void Clear() = 0;
	virtual void Reset() = 0;
	virtual void Train() = 0;

	virtual int GetIter() const = 0;
	virtual int GetCriticIter() const;
	virtual int GetActorIter() const;
	virtual void ResetExpBuffer();

	virtual void EndTraining() = 0;
	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner) = 0;

	virtual void LoadModel(const std::string& model_file) = 0;
	virtual void LoadScale(const std::string& model_file) = 0;
	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual bool HasInitModel() const = 0;
	virtual bool HasActorInitModel() const;
	virtual bool HasCriticInitModel() const;
	virtual bool HasForwardDynamicsInitModel() const;

	virtual int GetStateSize() const = 0;
	virtual int GetActionSize() const = 0;
	virtual int GetInputSize() const = 0;
	virtual int GetOutputSize() const = 0;

	virtual void SetIntOutputCallback(tCallbackFunc func) = 0;
	virtual void OutputIntermediate() = 0;
	virtual void OutputIntermediateModel(const std::string& filename) const = 0;

	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetForwardDynamicsInputSize() const;
	virtual int GetForwardDynamicsOutputSize() const;

	virtual void SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types) = 0;
	virtual void SetCriticInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetActorInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);
	virtual void SetForwardDynamicsInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types);

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) = 0;
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) = 0;
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void OutputModel(const std::string& filename) const = 0;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual bool IsDone() const = 0;

protected:

	cTrainerInterface();
};