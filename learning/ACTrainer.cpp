#include "ACTrainer.h"
#include "util/FileUtil.h"
#include "util/Util.h"
#include "ACLearner.h"

std::string cACTrainer::GetCriticFilename(const std::string& actor_filename)
{
	std::string file_no_ext = cFileUtil::RemoveExtension(actor_filename);
	std::string ext = cFileUtil::GetExtension(actor_filename);
	std::string critic_file = file_no_ext + "_critic." + ext;
	return critic_file;
}

cACTrainer::cACTrainer()
{
	mActorIter = 0;
}

cACTrainer::~cACTrainer()
{
}

void cACTrainer::Init(const tParams& params)
{
	mParams = params;
	mActorIter = 0;
	mActorBatchBuffer.clear();

	cNeuralNetTrainer::Init(params);
	InitActorProblem(mActorProb);
}

void cACTrainer::Clear()
{
	cNeuralNetTrainer::Clear();

	mActorIter = 0;
	mActorProb.Clear();
	mActorNet.reset();
	mActorBatchBuffer.clear();
}

void cACTrainer::Reset()
{
	mActorIter = 0;
	mActorBatchBuffer.clear();
	cNeuralNetTrainer::Reset();
}

int cACTrainer::GetIter() const
{
	return GetCriticIter();
}

int cACTrainer::GetCriticIter() const
{
	return mIter;
}

int cACTrainer::GetActorIter() const
{
	return mActorIter;
}

void cACTrainer::SetInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	SetCriticInputOffsetScaleType(scale_types);
	SetActorInputOffsetScaleType(scale_types);
}

void cACTrainer::SetCriticInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	cNeuralNetTrainer::SetInputOffsetScaleType(scale_types);
}

void cACTrainer::SetActorInputOffsetScaleType(const std::vector<cNeuralNet::eOffsetScaleType>& scale_types)
{
	assert(scale_types.size() == GetActorInputSize());
	mActorInputOffsetScaleTypes = scale_types;
}

void cACTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetCriticInputOffsetScale(offset, scale);
	SetActorInputOffsetScale(offset, scale);
}

void cACTrainer::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cNeuralNetTrainer::SetInputOffsetScale(offset, scale);
}

void cACTrainer::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetInputOffsetScale(offset, scale);
}

void cACTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetActorOutputOffsetScale(offset, scale);
}

void cACTrainer::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cNeuralNetTrainer::SetOutputOffsetScale(offset, scale);
}

void cACTrainer::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetOutputOffsetScale(offset, scale);
}

void cACTrainer::GetInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetActorInputOffsetScale(out_offset, out_scale);
}

void cACTrainer::GetOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetActorOutputOffsetScale(out_offset, out_scale);
}

void cACTrainer::GetCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetCritic();
	out_offset = net->GetInputOffset();
	out_scale = net->GetInputScale();
}

void cACTrainer::GetCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetCritic();
	out_offset = net->GetOutputOffset();
	out_scale = net->GetOutputScale();
}

void cACTrainer::GetActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetActor();
	out_offset = net->GetInputOffset();
	out_scale = net->GetInputScale();
}

void cACTrainer::GetActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const auto& net = GetActor();
	out_offset = net->GetOutputOffset();
	out_scale = net->GetOutputScale();
}

void cACTrainer::LoadCriticModel(const std::string& model_file)
{
	cNeuralNetTrainer::LoadModel(model_file);
}

void cACTrainer::LoadCriticScale(const std::string& scale_file)
{
	cNeuralNetTrainer::LoadScale(scale_file);
}

void cACTrainer::LoadActorModel(const std::string& model_file)
{
	const auto& actor = GetActor();
	actor->LoadModel(model_file);
}

void cACTrainer::LoadActorScale(const std::string& scale_file)
{
	const auto& actor = GetActor();
	actor->LoadScale(scale_file);
}

const std::string& cACTrainer::GetNetFile() const
{
	return GetActorNetFile();
}

const std::string& cACTrainer::GetSolverFile() const
{
	return GetActorSolverFile();
}

const std::string& cACTrainer::GetActorNetFile() const
{
	return mParams.mNetFile;
}

const std::string& cACTrainer::GetActorSolverFile() const
{
	return mParams.mSolverFile;
}

const std::string& cACTrainer::GetCriticNetFile() const
{
	return mParams.mCriticNetFile;
}

const std::string& cACTrainer::GetCriticSolverFile() const
{
	return mParams.mCriticSolverFile;
}

void cACTrainer::OutputModel(const std::string& filename) const
{
	std::string critic_filename = GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}

void cACTrainer::OutputCritic(const std::string& filename) const
{
	const auto& critic = GetCritic();
	critic->OutputModel(filename);
	printf("Critic model saved to %s\n", filename.c_str());
}

void cACTrainer::OutputActor(const std::string& filename) const
{
	const auto& actor = GetActor();
	actor->OutputModel(filename);
	printf("Actor model saved to %s\n", filename.c_str());
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetNet() const
{
	return GetActor();
}

int cACTrainer::GetCriticInputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetInputSize();
}

int cACTrainer::GetCriticOutputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetOutputSize();
}

int cACTrainer::GetActorInputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetInputSize();
}

int cACTrainer::GetActorOutputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetOutputSize();
}

const Eigen::VectorXd& cACTrainer::GetCriticInputOffset() const
{
	const auto& net = GetCritic();
	return net->GetInputOffset();
}

const Eigen::VectorXd& cACTrainer::GetCriticInputScale() const
{
	const auto& net = GetCritic();
	return net->GetInputScale();
}

const Eigen::VectorXd& cACTrainer::GetActorInputOffset() const
{
	const auto& net = GetActor();
	return net->GetInputOffset();
}

const Eigen::VectorXd& cACTrainer::GetActorInputScale() const
{
	const auto& net = GetCritic();
	return net->GetInputScale();
}

void cACTrainer::InitInputOffsetScaleTypes()
{
	InitCriticInputOffsetScaleTypes();
	InitActorInputOffsetScaleTypes();
}

void cACTrainer::InitCriticInputOffsetScaleTypes()
{
	int input_size = GetCriticInputSize();
	mInputOffsetScaleTypes.resize(input_size);
	for (int i = 0; i < input_size; ++i)
	{
		mInputOffsetScaleTypes[i] = cNeuralNet::eOffsetScaleTypeNone;
	}
}

void cACTrainer::InitActorInputOffsetScaleTypes()
{
	int input_size = GetActorInputSize();
	mActorInputOffsetScaleTypes.resize(input_size);
	for (int i = 0; i < input_size; ++i)
	{
		mActorInputOffsetScaleTypes[i] = cNeuralNet::eOffsetScaleTypeNone;
	}
}

void cACTrainer::InitActorProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetActor();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cACTrainer::InitStage()
{
	if (mParams.mRewardMode == eRewardModeAvg)
	{
		InitAvgReward();
	}
	
	cNeuralNetTrainer::InitStage();
}

void cACTrainer::InitAvgReward()
{
	double avg_reward = 0;
	int num_tuples = mExpBuffer->GetNumTuples();
	for (int i = 0; i < num_tuples; ++i)
	{
		tExpTuple tuple = GetTuple(i);
		avg_reward += tuple.mReward / num_tuples;
	}
	mAvgReward = avg_reward;
}

void cACTrainer::InitDataRecord()
{
	int critic_size = GetCriticInputSize();
	int actor_size = GetActorInputSize();
	mDataRecordX.Init(critic_size);
	mDataRecordActorX.Init(actor_size);
}

void cACTrainer::SetupExpBufferParams(int buffer_size, cExpBuffer::tParams& out_params) const
{
	cNeuralNetTrainer::SetupExpBufferParams(buffer_size, out_params);
	out_params.mStateEndSize = GetStateSize();
}

void cACTrainer::BuildNets()
{
	int pool_size = GetPoolSize();
	BuildNetPool(GetCriticNetFile(), GetCriticSolverFile(), pool_size);
	BuildActor(GetActorNetFile(), GetActorSolverFile());
	LoadModels();
}

void cACTrainer::BuildActor(const std::string& net_file, const std::string& solver_file)
{
	mActorNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	mActorNet->LoadNet(net_file);
	mActorNet->LoadSolver(solver_file);
}

void cACTrainer::LoadModels()
{
	if (mParams.mModelFile != "")
	{
		LoadActorModel(mParams.mModelFile);
	}

	if (mParams.mCriticModelFile != "")
	{
		LoadCriticModel(mParams.mCriticModelFile);
	}
}

void cACTrainer::BuildTupleActorX(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateBeg;
}

void cACTrainer::BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = tuple.mAction;
}

void cACTrainer::BuildActorProblemX(const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int batch_size = GetActorBatchSize();
	batch_size = std::min(batch_size, static_cast<int>(tuple_ids.size()));
	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x;
		BuildTupleActorX(tuple, x);
		out_prob.mX.row(i) = x;
	}
}

void cACTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int batch_size = GetActorBatchSize();
	batch_size = std::min(batch_size, static_cast<int>(tuple_ids.size()));
	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd y;
		BuildTupleActorY(tuple, y);
		out_prob.mY.row(i) = y;
	}
}

void cACTrainer::BuildActorTupleXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateEnd;
}

void cACTrainer::FetchActorMinibatch(int batch_size, std::vector<int>& out_batch)
{
	cNeuralNetTrainer::FetchMinibatch(batch_size, out_batch);
}

void cACTrainer::BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateEnd;
}

void cACTrainer::ApplySteps(int num_steps)
{
	printf("Actor Iter %i\n", mActorIter);
	cNeuralNetTrainer::ApplySteps(num_steps);
}

void cACTrainer::UpdateMisc(const std::vector<int>& tuple_ids)
{
	if (mParams.mRewardMode == eRewardModeAvg)
	{
		UpdateAvgReward(tuple_ids);
	}
}

void cACTrainer::UpdateAvgReward(const std::vector<int>& tuple_ids)
{
	double avg_reward = 0;
	int num_data = static_cast<int>(tuple_ids.size());
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		double r = tuple.mReward;

		avg_reward += r;
	}
	avg_reward /= num_data;
	
	mAvgReward += mParams.mAvgRewardStep * (avg_reward - mAvgReward);
}

void cACTrainer::UpdateDataRecord(const tExpTuple& tuple)
{
	Eigen::VectorXd critic_x;
	BuildTupleX(tuple, critic_x);
	mDataRecordX.Update(critic_x);

	Eigen::VectorXd actor_x;
	BuildTupleActorX(tuple, actor_x);
	mDataRecordActorX.Update(actor_x);
}

void cACTrainer::UpdateCurrActiveNetID()
{
	mCurrActiveNet = (mCurrActiveNet + 1) % GetNetPoolSize();
}

void cACTrainer::UpdateOffsetScale()
{
	UpdateCriticOffsetScale();
	UpdateActorOffsetScale();
}

void cACTrainer::UpdateCriticOffsetScale()
{
	Eigen::VectorXd offset = GetCriticInputOffset();
	Eigen::VectorXd scale = GetCriticInputScale();
	mDataRecordX.CalcOffsetScale(mInputOffsetScaleTypes, mParams.mInputScaleMax, offset, scale);
	SetCriticInputOffsetScale(offset, scale);
}

void cACTrainer::UpdateActorOffsetScale()
{
	Eigen::VectorXd offset = GetActorInputOffset();
	Eigen::VectorXd scale = GetActorInputScale();
	mDataRecordActorX.CalcOffsetScale(mActorInputOffsetScaleTypes, mParams.mInputScaleMax, offset, scale);
	SetActorInputOffsetScale(offset, scale);
}

bool cACTrainer::Step()
{
	UpdateCritic();
	UpdateActor();
	return true;
}

int cACTrainer::GetStateSize() const
{
	int size = 0;
	const auto& curr_net = GetActor();
	if (curr_net->HasNet())
	{
		size = curr_net->GetInputSize();
	}
	return size;
}

int cACTrainer::GetActionSize() const
{
	int size = 0;
	const auto& curr_net = GetActor();
	if (curr_net->HasNet())
	{
		size = curr_net->GetOutputSize();
	}
	return size;
}

int cACTrainer::GetActorBatchSize() const
{
	const std::unique_ptr<cNeuralNet>& net = GetActor();
	return net->GetBatchSize();
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetCritic() const
{
	return cNeuralNetTrainer::GetCurrNet();
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetActor() const
{
	return mActorNet;
}

bool cACTrainer::HasInitModel() const
{
	return HasActorInitModel() && HasCriticInitModel();
}

bool cACTrainer::HasActorInitModel() const
{
	bool has_init_model = false;
	const auto& actor = GetActor();
	if (actor != nullptr)
	{
		has_init_model = actor->HasValidModel();
	}
	return has_init_model;
}

bool cACTrainer::HasCriticInitModel() const
{
	bool has_init_model = false;
	const auto& critic = GetCritic();
	if (critic != nullptr)
	{
		has_init_model = critic->HasValidModel();
	}
	return has_init_model;
}

void cACTrainer::EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	EvalActor(tuple, out_y);
}

void cACTrainer::EvalActor(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	Eigen::VectorXd x;
	BuildTupleActorX(tuple, x);
	const auto& net = GetActor();
	net->Eval(x, out_y);
}

void cACTrainer::EvalCritic(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	/*
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);
	const auto& net = GetCritic();
	net->Eval(x, out_y);
	*/

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildTupleX(tuple, x);
	out_y = Eigen::VectorXd::Zero(GetCriticOutputSize());

	int num_critics = GetNetPoolSize();
	for (int i = 0; i < num_critics; ++i)
	{
		const auto& net = mNetPool[i];
		net->Eval(x, y);
		out_y += y;
	}
	out_y /= num_critics;
}

void cACTrainer::ResetCriticWeights()
{
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		const auto& critic_net = mNetPool[i];
		critic_net->ResetWeights();
	}
}

void cACTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cACLearner>(new cACLearner(shared_from_this()));
}

void cACTrainer::UpdateActorBatchBuffer()
{
	int batch_size = GetActorBatchSize();
	FetchActorMinibatch(batch_size, mActorBatchBuffer);
}

void cACTrainer::UpdateCritic()
{
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		bool succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}
}

void cACTrainer::UpdateActor()
{
	if (mStage != eStageInit)
	{
		UpdateActorBatchBuffer();
	}

	int batch_size = GetActorBatchSize();
	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	int num_batches = buffer_size / batch_size;

	for (int b = 0; b < num_batches; ++b)
	{
		StepActor();
		UpdateActorBatchBufferPostStep(batch_size);
	}
}

void cACTrainer::StepActor()
{
	BuildActorProblem(mActorProb); // problem here, returning fd problem
	UpdateActorNet(mActorProb);
	IncActorIter();
}

void cACTrainer::IncActorIter()
{
	++mActorIter;
}

void cACTrainer::UpdateActorNet(const cNeuralNet::tProblem& prob)
{
	auto& curr_net = mActorNet;
	curr_net->Train(prob);
}

void cACTrainer::BuildActorProblem(cNeuralNet::tProblem& out_prob)
{
	int num_data = GetActorBatchSize();
	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	assert(buffer_size >= num_data);
	
	BuildActorProblemX(mActorBatchBuffer, out_prob);
	BuildActorProblemY(mActorBatchBuffer, out_prob.mX, out_prob);
}

void cACTrainer::UpdateActorBatchBufferPostStep(int batch_size)
{
	mActorBatchBuffer.erase(mActorBatchBuffer.begin(), mActorBatchBuffer.begin() + batch_size);
}


int cACTrainer::GetServerActorID() const
{
	return mParams.mPoolSize;
}

void cACTrainer::ResetSolvers()
{
	cNeuralNetTrainer::ResetSolvers();
	ResetActorSolver();
}

void cACTrainer::ResetActorSolver()
{
	mActorNet->ResetSolver();
}

void cACTrainer::OutputIntermediateModel(const std::string& filename) const
{
	std::string critic_filename = GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}