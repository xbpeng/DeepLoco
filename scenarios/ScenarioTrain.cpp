#include "ScenarioTrain.h"
#include <thread>
#include "learning/CaclaTrainer.h"
#include "sim/BaseControllerCacla.h"

const double gInitCurriculumPhase = 1;

cScenarioTrain::cScenarioTrain()
{
	mTrainerParams.mPoolSize = 2; // double Q learning
	mTrainerParams.mPlaybackMemSize = 500000;
	mTrainerParams.mNumInitSamples = 200;
	mTrainerParams.mFreezeTargetIters = 0;

	mExpPoolSize = 1;
	mMaxIter = 100000;

	mExpParams.mRate = 0.1;
	mExpParams.mTemp = 0.1;
	mExpParams.mBaseActionRate = 0.1;
	mInitExpParams.mRate = 1;
	mInitExpParams.mTemp = 1;
	mInitExpParams.mBaseActionRate = 1;

	mNumAnnealIters = 1;
	mNumBaseAnnealIters = 1;

	mNumCurriculumIters = 0;
	mNumCurriculumStageIters = 0;
	mNumCurriculumInitExpScale = 1;

	mItersPerOutput = 20;
	mTimeStep = 1 / 30.0;
	EnableTraining(true);

	cTerrainRLCtrlFactory::InitNetFileArray(mNetFiles);
}

cScenarioTrain::~cScenarioTrain()
{
}

void cScenarioTrain::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenario::ParseArgs(parser);
	
	parser->ParseString("output_path", mOutputFile);
	parser->ParseInt("trainer_max_iter", mMaxIter);

	parser->ParseInt("exp_base_anneal_iters", mNumBaseAnnealIters);
	parser->ParseDouble("init_exp_rate", mInitExpParams.mRate);
	parser->ParseDouble("init_exp_temp", mInitExpParams.mTemp);
	parser->ParseDouble("init_exp_base_rate", mInitExpParams.mBaseActionRate);
	parser->ParseDouble("init_exp_noise", mInitExpParams.mNoise);
	parser->ParseDouble("init_exp_noise_internal", mInitExpParams.mNoiseInternal);

	parser->ParseInt("trainer_net_pool_size", mTrainerParams.mPoolSize);
	parser->ParseBool("trainer_enable_exp_replay", mTrainerParams.mEnableExpReplay);
	parser->ParseInt("trainer_replay_mem_size", mTrainerParams.mPlaybackMemSize);
	parser->ParseBool("trainer_init_input_offset_scale", mTrainerParams.mInitInputOffsetScale);
	parser->ParseInt("trainer_num_init_samples", mTrainerParams.mNumInitSamples);
	parser->ParseInt("trainer_num_steps_per_iters", mTrainerParams.mNumStepsPerIter);
	parser->ParseInt("trainer_freeze_target_iters", mTrainerParams.mFreezeTargetIters);
	parser->ParseInt("trainer_pretrain_iters", mTrainerParams.mPretrainIters);

	parser->ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser->ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);
	parser->ParseInt("trainer_num_anneal_iters", mNumAnnealIters);
	parser->ParseInt("trainer_curriculum_iters", mNumCurriculumIters);
	parser->ParseInt("trainer_curriculum_stage_iters", mNumCurriculumStageIters);
	parser->ParseDouble("trainer_curriculum_init_exp_scale", mNumCurriculumInitExpScale);
	parser->ParseInt("trainer_iters_per_output", mItersPerOutput);
	parser->ParseDouble("trainer_input_scale_max", mTrainerParams.mInputScaleMax);
	parser->ParseDouble("trainer_discount", mTrainerParams.mDiscount);

	std::string pg_mode_str = "";
	parser->ParseString("trainer_pg_mode", pg_mode_str);
	cTrainerInterface::ParsePGMode(pg_mode_str, mTrainerParams.mPGMode);

	parser->ParseBool("trainer_pg_enable_on_policy", mTrainerParams.mPGEnableOnPolicy);
	parser->ParseBool("trainer_pg_enable_importance_sampling", mTrainerParams.mPGEnableImportanceSampling);
	parser->ParseDouble("trainer_pg_adv_scale", mTrainerParams.mPGAdvScale);
	parser->ParseDouble("trainer_pg_iw_clip", mTrainerParams.mPGIWClip);
	parser->ParseDouble("trainer_pg_adv_clip", mTrainerParams.mPGAdvClip);

	parser->ParseBool("trainer_enable_td_lambda", mTrainerParams.mEnableTDLambda);
	parser->ParseInt("trainer_num_reward_steps", mTrainerParams.mNumRewardSteps);
	parser->ParseDouble("trainer_td_lambda", mTrainerParams.mTDLambda);

	parser->ParseInt("trainer_num_entropy_samples", mTrainerParams.mNumEntropySamples);
	parser->ParseDouble("trainer_entropy_weight", mTrainerParams.mEntropyWeight);
	parser->ParseDouble("trainer_entropy_kernel_width", mTrainerParams.mEntropyKernelWidth);

	parser->ParseString("policy_net", mNetFiles[cTerrainRLCtrlFactory::eNetFileActor]);
	parser->ParseString("critic_net", mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic]);
	parser->ParseString("policy_net1", mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1]);
	parser->ParseString("critic_net1", mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1]);
	parser->ParseString("policy_solver", mNetFiles[cTerrainRLCtrlFactory::eNetFileActorSolver]);
	parser->ParseString("critic_solver", mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticSolver]);
	parser->ParseString("policy_solver1", mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1Solver]);
	parser->ParseString("critic_solver1", mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1Solver]);
	parser->ParseString("policy_model", mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel]);
	parser->ParseString("critic_model", mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel]);
	parser->ParseString("policy_model1", mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1Model]);
	parser->ParseString("critic_model1", mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1Model]);
	parser->ParseString("policy_model_terrain", mNetFiles[cTerrainRLCtrlFactory::eNetFileActorTerrainModel]);
	parser->ParseString("policy_model_action", mNetFiles[cTerrainRLCtrlFactory::eNetFileActorActionModel]);
	parser->ParseString("policy_model_terrain", mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticTerrainModel]);
	parser->ParseString("policy_model_value", mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticValueModel]);

	parser->ParseString("forward_dynamics_net", mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamics]);
	parser->ParseString("forward_dynamics_solver", mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsSolver]);
	parser->ParseString("forward_dynamics_model", mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsModel]);



	mArgParser = parser;
}

void cScenarioTrain::Init()
{
	cScenario::Init();
	BuildScenePool();
	InitTrainer();
	InitLearners();
	EnableTraining(true);
}

void cScenarioTrain::Reset()
{
	cScenario::Reset();
	ResetScenePool();
}

void cScenarioTrain::Clear()
{
	cScenario::Clear();
	if (mTrainer != nullptr)
	{
		mTrainer->Clear();
	}
	mLearners.clear();
}

void cScenarioTrain::Run()
{
	int num_threads = GetPoolSize();
	std::vector<std::thread> threads(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		std::thread& curr_thread = threads[i];
		curr_thread = std::thread(&cScenarioTrain::ExpHelper, this, mExpPool[i], i);
	}
	
	for (int i = 0; i < num_threads; ++i)
	{
		threads[i].join();
	}
}

void cScenarioTrain::Update(double time_elapsed)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		UpdateExpScene(time_elapsed, i, mExpPool[i]);
	}
}

void cScenarioTrain::SetExpPoolSize(int size)
{
	mExpPoolSize = size;
	mExpPoolSize = std::max(1, mExpPoolSize);
	mTrainerParams.mNumThreads = mExpPoolSize;
}

int cScenarioTrain::GetPoolSize() const
{
	return static_cast<int>(mExpPool.size());
}

const std::shared_ptr<cScenarioExp>& cScenarioTrain::GetExpScene(int i) const
{
	return mExpPool[i];
}

bool cScenarioTrain::LoadControlParams(const std::string& param_file)
{
	bool succ = true;
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		succ &= mExpPool[i]->LoadControlParams(param_file);
	}
	return succ;
}

bool cScenarioTrain::TrainingComplete() const
{
	return GetIter() >= mMaxIter;
}

void cScenarioTrain::EnableTraining(bool enable)
{
	mEnableTraining = enable;

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->EnableExplore(enable);
	}
}

void cScenarioTrain::ToggleTraining()
{
	EnableTraining(!mEnableTraining);
}

bool cScenarioTrain::TrainingEnabled() const
{
	return mEnableTraining;
}

int cScenarioTrain::GetIter() const
{
	int iter = mTrainer->GetIter();
	return iter;
}


const std::string& cScenarioTrain::GetOutputFile() const
{
	return mOutputFile;
}

void cScenarioTrain::SetOutputFile(const std::string& file)
{
	mOutputFile = file;
}

cNeuralNetTrainer::tParams& cScenarioTrain::GetTrainerParams()
{
	return mTrainerParams;
}

const cNeuralNetTrainer::tParams& cScenarioTrain::GetTrainerParams() const
{
	return mTrainerParams;
}

void cScenarioTrain::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

bool cScenarioTrain::IsDone() const
{
	bool done = mTrainer->IsDone();
	int iter = GetIter();
	done |= iter >= mMaxIter;
	return done;
}

void cScenarioTrain::Shutdown()
{
	cScenario::Shutdown();
	mTrainer->EndTraining();
	mTrainer->OutputModel(mOutputFile);
}

void cScenarioTrain::OutputModel() const
{
	OutputModel(mOutputFile);
}

void cScenarioTrain::OutputModel(const std::string& out_file) const
{
	mTrainer->OutputModel(out_file);
}

std::vector<std::string>& cScenarioTrain::GetNetFiles()
{
	return mNetFiles;
}

const std::vector<std::string>& cScenarioTrain::GetNetFiles() const
{
	return mNetFiles;
}

std::string cScenarioTrain::GetName() const
{
	return "Train";
}

void cScenarioTrain::BuildScenePool()
{
	ClearScenePool();

	mExpPool.resize(mExpPoolSize);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto& curr_exp = mExpPool[i];
		BuildExpScene(i, curr_exp);
		SetupExpScene(i, curr_exp);
		curr_exp->Init();

		if (i == 0)
		{
			mExpParams = curr_exp->GetExpParams();
		}

		curr_exp->SetExpParams(mInitExpParams);
		UpdateSceneCurriculum(gInitCurriculumPhase, curr_exp);
		curr_exp->Reset(); // rebuild ground
	}

	ResetScenePool();
}

void cScenarioTrain::ClearScenePool()
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->Clear();
	}
}

void cScenarioTrain::ResetScenePool()
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->Reset();
	}
}

void cScenarioTrain::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExp());
}

void cScenarioTrain::SetupExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp->ParseArgs(mArgParser);
	std::vector<std::string>& net_files = out_exp->GetNetFiles();
	net_files = mNetFiles;
}

const std::shared_ptr<cCharController>& cScenarioTrain::GetDefaultController() const
{
	const auto& exp_scene = GetExpScene(0);
	const auto& character = exp_scene->GetCharacter();
	return character->GetController();
}

void cScenarioTrain::SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const
{
	out_params.mNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActor];
	out_params.mSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActorSolver];
	out_params.mModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel];
	out_params.mCriticNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic];
	out_params.mCriticSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticSolver];
	out_params.mCriticModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel];
}

void cScenarioTrain::InitTrainer()
{
	SetupTrainerParams(mTrainerParams);
	BuildTrainer(mTrainer);
	mTrainer->Init(mTrainerParams);
	SetupTrainer(mTrainer);
}

void cScenarioTrain::InitLearners()
{
	int pool_size = GetPoolSize();
	mLearners.resize(pool_size);
	for (int i = 0; i < pool_size; ++i)
	{
		auto& curr_learner = mLearners[i];
		mTrainer->RequestLearner(curr_learner);
		auto& exp_scene = GetExpScene(i);

		const auto& character = exp_scene->GetCharacter();
		SetupLearner(character, curr_learner);
		curr_learner->Init();
	}
}

void cScenarioTrain::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(character->GetController());
	auto& net = ctrl->GetNet();
	out_learner->SetNet(net.get());
}

bool cScenarioTrain::IsLearnerDone(int learner_id) const
{
	const auto& learner = mLearners[learner_id];
	bool done = learner->IsDone();
	int iter = learner->GetIter();
	done |= iter >= mMaxIter;
	return done;
}

const std::shared_ptr<cCharController>& cScenarioTrain::GetRefController() const
{
	const std::shared_ptr<cScenarioExp> exp = GetExpScene(0);
	const std::shared_ptr<cSimCharacter> character = exp->GetCharacter();
	return character->GetController();
}

void cScenarioTrain::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	out_trainer = trainer;
}

void cScenarioTrain::SetupTrainer(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	SetupTrainerOffsetScale(out_trainer);
}

void cScenarioTrain::SetupTrainerOffsetScale(const std::shared_ptr<cTrainerInterface>& out_trainer)
{
	bool valid_init_model = out_trainer->HasInitModel();
	if (!valid_init_model)
	{
		std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(GetDefaultController());
		
		Eigen::VectorXd input_offset;
		Eigen::VectorXd input_scale;
		ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
		out_trainer->SetInputOffsetScale(input_offset, input_scale);

		std::vector<cNeuralNet::eOffsetScaleType> scale_types;
		ctrl->BuildNNInputOffsetScaleTypes(scale_types);
		out_trainer->SetInputOffsetScaleType(scale_types);

		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
		out_trainer->SetOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioTrain::UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id)
{
	auto& learner = mLearners[exp_id];
	learner->Train(tuples);

	int iters = learner->GetIter();
	PrintLearnerInfo(exp_id);
	
	if ((iters % mItersPerOutput == 0 && iters > 0) || iters == 1)
	{
		learner->OutputModel(mOutputFile);
	}
}

void cScenarioTrain::PrintLearnerInfo(int exp_id) const
{
	const auto& learner = mLearners[exp_id];
	const auto& scene = mExpPool[exp_id];
	const auto& exp_params = scene->GetExpParams();

	int iters = learner->GetIter();
	int num_tuples = learner->GetNumTuples();
	double curriculum_phase = CalcCurriculumPhase(iters);

	printf("\nIter %i\n", iters);
	printf("Num Tuples: %i\n", num_tuples);
	printf("Curriculum Phase: %.5f\n", curriculum_phase);
	printf("Exp Rate: %.5f\n", exp_params.mRate);
	printf("Exp Temp: %.5f\n", exp_params.mTemp);
	printf("Exp Noise: %.5f\n", exp_params.mNoise);
	printf("Exp Base Rate: %.5f\n", exp_params.mBaseActionRate);
}

void cScenarioTrain::UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp)
{
	bool dummy_flag;
	UpdateExpScene(time_step, exp_id, out_exp, dummy_flag);
}

void cScenarioTrain::UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done)
{
	out_done = false;
	out_exp->Update(time_step);
	
	if (time_step > 0)
	{
		bool is_full = out_exp->IsTupleBufferFull();
		if (is_full)
		{
			if (mEnableTraining)
			{
				const std::vector<tExpTuple>& tuples = out_exp->GetTuples();
				UpdateTrainer(tuples, exp_id);
				UpdateExpSceneRates(exp_id, out_exp);
				out_done = IsLearnerDone(exp_id);
			}
			out_exp->ResetTupleBuffer();
		}
	}
}

void cScenarioTrain::UpdateExpSceneRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();

	cCharController::tExpParams exp_params = BuildExpParams(iters);
	out_exp->SetExpParams(exp_params);

	double curriculum_phase = CalcCurriculumPhase(iters);
	UpdateSceneCurriculum(curriculum_phase, out_exp);
}

void cScenarioTrain::UpdateSceneCurriculum(double phase, std::shared_ptr<cScenarioExp>& out_exp) const
{
	double terrain_lerp = phase;
	out_exp->SetCurriculumPhase(phase);
}

cCharController::tExpParams cScenarioTrain::BuildExpParams(int iter) const
{
	double lerp = static_cast<double>(iter) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);

	double base_rate_lerp = static_cast<double>(iter) / mNumBaseAnnealIters;
	base_rate_lerp = cMathUtil::Clamp(base_rate_lerp, 0.0, 1.0);

	cCharController::tExpParams exp_params = mExpParams;
	exp_params.mRate = (1 - lerp) * mInitExpParams.mRate + lerp * mExpParams.mRate;
	exp_params.mTemp = (1 - lerp) * mInitExpParams.mTemp + lerp * mExpParams.mTemp;
	exp_params.mNoise = (1 - lerp) * mInitExpParams.mNoise + lerp * mExpParams.mNoise;
	exp_params.mNoiseInternal = (1 - lerp) * mInitExpParams.mNoiseInternal + lerp * mExpParams.mNoiseInternal;
	exp_params.mBaseActionRate = (1 - base_rate_lerp) * mInitExpParams.mBaseActionRate + base_rate_lerp * mExpParams.mBaseActionRate;

	return exp_params;
}

double cScenarioTrain::CalcCurriculumPhase(int iters) const
{
	bool enable_curiculum = EnableCurriculum();
	double lerp = (enable_curiculum) ? (static_cast<double>(iters) / mNumCurriculumIters) : 1;
	double phase = lerp;

	if (iters == 0)
	{
		phase = gInitCurriculumPhase;
	}

	return phase;
}

bool cScenarioTrain::EnableCurriculum() const
{
	bool enable = mNumCurriculumIters >= 1;
	return enable;
}

void cScenarioTrain::ExpHelper(std::shared_ptr<cScenarioExp> exp, int exp_id)
{
	bool done = false;
	while (!done)
	{
		UpdateExpScene(mTimeStep, exp_id, exp, done);
	}
	exp->Shutdown();
}
