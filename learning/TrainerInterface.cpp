#include "TrainerInterface.h"


cTrainerInterface::tParams::tParams()
{
	mNetFile = "";
	mSolverFile = "";
	mModelFile = "";
	mCriticNetFile = "";
	mCriticSolverFile = "";
	mCriticModelFile = "";

	mForwardDynamicsNetFile = "";
	mForwardDynamicsSolverFile = "";
	mForwardDynamicsModelFile = "";

	mNumThreads = 1;
	mEnableExpReplay = true;
	mPlaybackMemSize = 1024;
	mPoolSize = 1;
	mNumInitSamples = 1024;
	mNumStepsPerIter = 1;
	mFreezeTargetIters = 0;
	mPretrainIters = 0;
	mDiscount = 0.9;

	mInitInputOffsetScale = true;
	mInputScaleMax = std::numeric_limits<double>::infinity();

	mRewardMode = eRewardModeStart;
	mAvgRewardStep = 0.01;

	mIntOutputIters = 0;
	mIntOutputFile = "";

	mPGMode = ePGModeCacla;
	mPGEnableOnPolicy = false;
	mPGEnableImportanceSampling = false;
	mPGAdvScale = 1;
	mPGIWClip = 2;
	mPGAdvClip = 3;

	mEnableTDLambda = false;
	mNumRewardSteps = 1;
	mTDLambda = 0.9;

	mNumEntropySamples = 32;
	mEntropyWeight = 0.05;
	mEntropyKernelWidth = 0.1;
}

void cTrainerInterface::ParsePGMode(const std::string& mode_str, ePGMode& out_mode)
{
	if (mode_str == "" || mode_str == "cacla")
	{
		out_mode = ePGModeCacla;
	}
	else if (mode_str == "td")
	{
		out_mode = ePGModeTD;
	}
	else if (mode_str == "ptd")
	{
		out_mode = ePGModePTD;
	}
	else
	{
		printf("Unsupported policy gradient mode %s\n", mode_str.c_str());
		assert(false); // unsupported pg mode
	}
}

cTrainerInterface::cTrainerInterface()
{
}

cTrainerInterface::~cTrainerInterface()
{
}

int cTrainerInterface::GetCriticIter() const
{
	return GetIter();
}

int cTrainerInterface::GetActorIter() const
{
	return GetIter();
}

void cTrainerInterface::ResetExpBuffer()
{
}

void cTrainerInterface::LoadCriticModel(const std::string& model_file)
{
	LoadModel(model_file);
}

void cTrainerInterface::LoadCriticScale(const std::string& scale_file)
{
	LoadScale(scale_file);
}

void cTrainerInterface::LoadActorModel(const std::string& model_file)
{
	LoadModel(model_file);
}

void cTrainerInterface::LoadActorScale(const std::string& scale_file)
{
	LoadScale(scale_file);
}

bool cTrainerInterface::HasActorInitModel() const
{
	return HasInitModel();
}

bool cTrainerInterface::HasCriticInitModel() const
{
	return HasInitModel();
}

bool cTrainerInterface::HasForwardDynamicsInitModel() const
{
	return HasInitModel();
}

int cTrainerInterface::GetCriticInputSize() const
{
	return GetInputSize();
}

int cTrainerInterface::GetCriticOutputSize() const
{
	return GetOutputSize();
}

int cTrainerInterface::GetActorInputSize() const
{
	return GetInputSize();
}

int cTrainerInterface::GetActorOutputSize() const
{
	return GetOutputSize();
}

int cTrainerInterface::GetForwardDynamicsInputSize() const
{
	return GetOutputSize();
}
int cTrainerInterface::GetForwardDynamicsOutputSize() const
{
	return GetOutputSize();
}

void cTrainerInterface::SetCriticInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	SetInputOffsetScaleType(scale_types);
}

void cTrainerInterface::SetActorInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	SetInputOffsetScaleType(scale_types);
}

void cTrainerInterface::SetForwardDynamicsInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	SetInputOffsetScaleType(scale_types);
}

void cTrainerInterface::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetInputOffsetScale(offset, scale);
}

void cTrainerInterface::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetInputOffsetScale(offset, scale);
}

void cTrainerInterface::SetForwardDynamicsInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetInputOffsetScale(offset, scale);
}

void cTrainerInterface::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetOutputOffsetScale(offset, scale);
}

void cTrainerInterface::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetOutputOffsetScale(offset, scale);
}

void cTrainerInterface::SetForwardDynamicsOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetOutputOffsetScale(offset, scale);
}


void cTrainerInterface::OutputCritic(const std::string& filename) const
{
	OutputModel(filename);
}

void cTrainerInterface::OutputActor(const std::string& filename) const
{
	OutputModel(filename);
}