#include "NeuralNetLearner.h"
#include "NeuralNetTrainer.h"

cNeuralNetLearner::cNeuralNetLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
{
	assert(trainer != nullptr);
	mTrainer = trainer;
	mNet = nullptr;

	mIter = mTrainer->GetIter();
	mNumTuples = mTrainer->GetNumTuples();
	mID = mTrainer->RegisterLearner(this);
	mPrevTupleID = gInvalidIdx;
	mStage = mTrainer->GetStage();
}

cNeuralNetLearner::~cNeuralNetLearner()
{
	mTrainer->UnregisterLearner(this);
}

void cNeuralNetLearner::Reset()
{
	mIter = mTrainer->GetIter();
	mNumTuples = mTrainer->GetNumTuples();
	mStage = mTrainer->GetStage();
	mPrevTupleID = gInvalidIdx;
	SyncNet();
}

void cNeuralNetLearner::Init()
{
	mPrevTupleID = gInvalidIdx;
	LoadNet(mTrainer->GetNetFile());
	SyncNet();
}

void cNeuralNetLearner::Train(const std::vector<tExpTuple>& tuples)
{
	mTrainer->Lock();

	UpdateTrainer();
	AddTuples(tuples);
	mTrainer->Train();
	SyncNet();

	mIter = mTrainer->GetIter();
	mNumTuples = mTrainer->GetNumTuples();
	mStage = mTrainer->GetStage();

	mTrainer->Unlock();
}

int cNeuralNetLearner::GetIter() const
{
	return mIter;
}

int cNeuralNetLearner::GetNumTuples() const
{
	return mNumTuples;
}

int cNeuralNetLearner::GetStage() const
{
	return mStage;
}

void cNeuralNetLearner::SetNet(cNeuralNet* net)
{
	assert(net != nullptr);
	mNet = net;
}

const cNeuralNet* cNeuralNetLearner::GetNet() const
{
	return mNet;
}

void cNeuralNetLearner::LoadNet(const std::string& net_file)
{
	mNet->LoadNet(net_file);
}

void cNeuralNetLearner::LoadSolver(const std::string& solver_file)
{
	mNet->LoadSolver(solver_file);
}

void cNeuralNetLearner::OutputModel(const std::string& filename) const
{
	if (HasNet())
	{
		mNet->OutputModel(filename);
	}
	else
	{
		mTrainer->Lock();
		mTrainer->OutputModel(filename);
		mTrainer->Unlock();
	}
	printf("Model saved to %s\n", filename.c_str());
}

void cNeuralNetLearner::SyncNet()
{
	if (HasNet())
	{
		auto& trainer_net = mTrainer->GetNet();
		mNet->CopyModel(*trainer_net);
	}
}

bool cNeuralNetLearner::HasNet() const
{
	return mNet != nullptr;
}

bool cNeuralNetLearner::IsDone() const
{
	return mTrainer->IsDone();
}

void cNeuralNetLearner::ResetExpBuffer()
{
	mTrainer->ResetExpBuffer();
}

void cNeuralNetLearner::AddTuples(const std::vector<tExpTuple>& tuples)
{
	mPrevTupleID = mTrainer->AddTuples(tuples, mPrevTupleID, mID);
}

void cNeuralNetLearner::UpdateTrainer()
{
}