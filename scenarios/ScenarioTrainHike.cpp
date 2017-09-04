#include "scenarios/ScenarioTrainHike.h"
#include "sim/WaypointController.h"
#include "learning/CaclaTrainer.h"
#include "learning/ACLearner.h"
#include "util/FileUtil.h"

cScenarioTrainHike::cScenarioTrainHike()
{
	mTrainHLC = true;
	mTrainLLC = true;
	mLLCItersPerOutput = 20;

	mLLCExpParams.mRate = 0.1;
	mLLCExpParams.mTemp = 0.1;
	mLLCExpParams.mBaseActionRate = 0.1;
	mLLCInitExpParams.mRate = 1;
	mLLCInitExpParams.mTemp = 1;
	mLLCInitExpParams.mBaseActionRate = 1;

	mHLCEpochIters = 1000;
	mLLCEpochIters = 1000;
}

cScenarioTrainHike::~cScenarioTrainHike()
{
}

void cScenarioTrainHike::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioImitateTarget::ParseArgs(parser);

	parser->ParseInt("hlc_epoch_iters", mHLCEpochIters);
	parser->ParseInt("llc_epoch_iters", mLLCEpochIters);

	parser->ParseString("llc_output_path", mLLCOutputFile);
	parser->ParseBool("train_hlc", mTrainHLC);
	parser->ParseBool("train_llc", mTrainLLC);

	parser->ParseDouble("llc_init_exp_rate", mLLCInitExpParams.mRate);
	parser->ParseDouble("llc_init_exp_temp", mLLCInitExpParams.mTemp);
	parser->ParseDouble("llc_init_exp_base_rate", mLLCInitExpParams.mBaseActionRate);
	parser->ParseDouble("llc_init_exp_noise", mLLCInitExpParams.mNoise);
	parser->ParseDouble("llc_init_exp_noise_internal", mLLCInitExpParams.mNoiseInternal);

	parser->ParseInt("trainer_iters_per_output", mLLCItersPerOutput);

	parser->ParseInt("llc_trainer_net_pool_size", mLLCTrainerParams.mPoolSize);
	parser->ParseInt("llc_trainer_replay_mem_size", mLLCTrainerParams.mPlaybackMemSize);
	parser->ParseBool("llc_trainer_init_input_offset_scale", mLLCTrainerParams.mInitInputOffsetScale);
	parser->ParseInt("llc_trainer_num_init_samples", mLLCTrainerParams.mNumInitSamples);
	parser->ParseInt("llc_trainer_num_steps_per_iters", mLLCTrainerParams.mNumStepsPerIter);
	parser->ParseInt("llc_trainer_freeze_target_iters", mLLCTrainerParams.mFreezeTargetIters);
	parser->ParseInt("llc_trainer_pretrain_iters", mLLCTrainerParams.mPretrainIters);

	parser->ParseInt("llc_trainer_int_iter", mLLCTrainerParams.mIntOutputIters);
	parser->ParseString("llc_trainer_int_output", mLLCTrainerParams.mIntOutputFile);
	parser->ParseDouble("llc_trainer_input_scale_max", mLLCTrainerParams.mInputScaleMax);
	parser->ParseDouble("llc_trainer_discount", mLLCTrainerParams.mDiscount);

	std::string pg_mode_str = "";
	parser->ParseString("llc_trainer_pg_mode", pg_mode_str);
	cTrainerInterface::ParsePGMode(pg_mode_str, mLLCTrainerParams.mPGMode);

	parser->ParseBool("llc_trainer_pg_enable_on_policy", mLLCTrainerParams.mPGEnableOnPolicy);
	parser->ParseBool("llc_trainer_pg_enable_importance_sampling", mLLCTrainerParams.mPGEnableImportanceSampling);
	parser->ParseDouble("llc_trainer_pg_adv_scale", mLLCTrainerParams.mPGAdvScale);
	parser->ParseDouble("llc_trainer_pg_iw_clip", mLLCTrainerParams.mPGIWClip);
	parser->ParseDouble("llc_trainer_pg_adv_clip", mLLCTrainerParams.mPGAdvClip);
}

void cScenarioTrainHike::Init()
{
	cScenarioImitateTarget::Init();
	InitLLCTrainer();
	InitLLCLearners();
}

void cScenarioTrainHike::Reset()
{
	cScenarioImitateTarget::Reset();
	mLLCTrainer->Reset();
}

void cScenarioTrainHike::Clear()
{
	cScenarioImitateTarget::Clear();
	if (mLLCTrainer != nullptr)
	{
		mLLCTrainer->Clear();
	}
	mLLCLearners.clear();
}

std::string cScenarioTrainHike::GetName() const
{
	return "Train Hike";
}

void cScenarioTrainHike::BuildScenePool()
{
	cScenarioImitateTarget::BuildScenePool();

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto& curr_exp = mExpPool[i];
		auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(curr_exp);

		if (i == 0)
		{
			mLLCExpParams = exp_hike->GetLLCExpParams();
		}

		exp_hike->SetLLCExpParams(mLLCInitExpParams);
	}
}

void cScenarioTrainHike::BuildExpScene(int id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExpHike>(new cScenarioExpHike());
}

void cScenarioTrainHike::SetupTrainerParams(cNeuralNetTrainer::tParams& out_params) const
{
	cScenarioImitateTarget::SetupTrainerParams(out_params);

#if !defined(HACK_SOCCER_LLC)
	out_params.mNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1];
	out_params.mSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1Solver];
	out_params.mModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1Model];
	out_params.mCriticNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1];
	out_params.mCriticSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1Solver];
	out_params.mCriticModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1Model];
#endif
}

void cScenarioTrainHike::InitTrainer()
{
	cScenarioImitateTarget::InitTrainer();

	cTrainerInterface::tCallbackFunc func = std::bind(&cScenarioTrainHike::IntOutputCallback, this);
	mTrainer->SetIntOutputCallback(func);
}

void cScenarioTrainHike::SetExpMode(cScenarioExpHike::eExpMode mode, std::shared_ptr<cScenarioExp>& out_exp) const
{
	auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(out_exp);
	exp_hike->SetExpMode(mode);
}

cScenarioExpHike::eExpMode cScenarioTrainHike::CalcExpMode(int hlc_iter, int llc_iter) const
{
	int hlc_epoc = hlc_iter / mHLCEpochIters;
	int llc_epoc = llc_iter / mLLCEpochIters;

	cScenarioExpHike::eExpMode mode = (mTrainHLC) ? cScenarioExpHike::eExpModeHLC
										: cScenarioExpHike::eExpModeLLC;
	if (mTrainLLC && hlc_epoc > llc_epoc)
	{
		mode = cScenarioExpHike::eExpModeLLC;
	}

	return mode;
}

void cScenarioTrainHike::BuildLLCTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	cScenarioImitateTarget::BuildTrainer(out_trainer);
}

void cScenarioTrainHike::SetupLLCTrainerParams(cNeuralNetTrainer::tParams& out_params) const
{
	out_params.mNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActor];
	out_params.mSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActorSolver];
	out_params.mModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel];
	out_params.mCriticNetFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic];
	out_params.mCriticSolverFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticSolver];
	out_params.mCriticModelFile = mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel];

	// disable intermediate output for LLCal policy
	// intermediate policies will be outputted in-sync with the meta policy
	out_params.mIntOutputIters = 0;
}

void cScenarioTrainHike::InitLLCTrainer()
{
	SetupLLCTrainerParams(mLLCTrainerParams);
	BuildLLCTrainer(mLLCTrainer);
	mLLCTrainer->Init(mLLCTrainerParams);
	SetupLLCTrainerOffsetScale();

	SetupLLCTrainerActionCovar();
	SetupLLCTrainerActionBounds();
}

void cScenarioTrainHike::InitLLCLearners()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	int pool_size = GetPoolSize();
	mLLCLearners.resize(pool_size);
	for (int i = 0; i < pool_size; ++i)
	{
		auto& curr_learner = mLLCLearners[i];
		mLLCTrainer->RequestLearner(curr_learner);
		auto& exp_scene = GetExpScene(i);

		const auto& character = exp_scene->GetCharacter();
		SetupLLCLearner(character, curr_learner);
		curr_learner->Init();
	}
}

void cScenarioTrainHike::SetupLLCLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	auto llc = GetDefaultLLC();
	auto& net = llc->GetNet();
	out_learner->SetNet(net.get());

	auto ac_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(llc);
	if (ac_ctrl != nullptr)
	{
		auto ac_learner = std::static_pointer_cast<cACLearner>(out_learner);
		auto& critic = ac_ctrl->GetCritic();
		ac_learner->SetCriticNet(critic.get());
	}
	else
	{
		assert(false); // controller does not support cacla
	}
}

void cScenarioTrainHike::SetupLLCTrainerOffsetScale()
{
	SetupLLCTrainerCriticOffsetScale();
	SetupLLCTrainerActorOffsetScale();
}

void cScenarioTrainHike::SetupLLCTrainerCriticOffsetScale()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	bool valid_init_model = mLLCTrainer->HasCriticInitModel();
	if (!valid_init_model)
	{
		int critic_input_size = mLLCTrainer->GetCriticInputSize();
		int critic_output_size = mLLCTrainer->GetCriticOutputSize();

		auto LLC_ctrl = GetDefaultLLC();
		auto cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(LLC_ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd critic_input_offset;
			Eigen::VectorXd critic_input_scale;
			cacla_ctrl->BuildCriticInputOffsetScale(critic_input_offset, critic_input_scale);

			Eigen::VectorXd critic_output_offset;
			Eigen::VectorXd critic_output_scale;
			cacla_ctrl->BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

			assert(critic_input_offset.size() == critic_input_size);
			assert(critic_input_scale.size() == critic_input_size);
			assert(critic_output_offset.size() == critic_output_size);
			assert(critic_output_scale.size() == critic_output_size);
			mLLCTrainer->SetCriticInputOffsetScale(critic_input_offset, critic_input_scale);
			mLLCTrainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
		}
		else
		{
			assert(false); // controller does not implement actor-critic interface
		}
	}
}

void cScenarioTrainHike::SetupLLCTrainerActorOffsetScale()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	bool valid_init_model = mLLCTrainer->HasActorInitModel();
	if (!valid_init_model)
	{
		int actor_input_size = mLLCTrainer->GetActorInputSize();
		int actor_output_size = mLLCTrainer->GetActorOutputSize();

		auto LLC_ctrl = GetDefaultLLC();
		auto cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(LLC_ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd actor_input_offset;
			Eigen::VectorXd actor_input_scale;
			cacla_ctrl->BuildActorInputOffsetScale(actor_input_offset, actor_input_scale);

			Eigen::VectorXd actor_output_offset;
			Eigen::VectorXd actor_output_scale;
			cacla_ctrl->BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);

			assert(actor_input_offset.size() == actor_input_size);
			assert(actor_input_scale.size() == actor_input_size);
			assert(actor_output_offset.size() == actor_output_size);
			assert(actor_output_scale.size() == actor_output_size);
			mLLCTrainer->SetActorInputOffsetScale(actor_input_offset, actor_input_scale);
			mLLCTrainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
		}
		else
		{
			assert(false); // controller does not support CACLA
		}
	}
}

void cScenarioTrainHike::BuildLLCActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	auto LLC_ctrl = GetDefaultLLC();
	LLC_ctrl->GetPoliActionBounds(out_min, out_max);
}

void cScenarioTrainHike::SetupLLCTrainerActionCovar()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	Eigen::VectorXd action_covar;
	BuildLLCActionCovar(action_covar);

	auto cacla_trainer = std::dynamic_pointer_cast<cCaclaTrainer>(mLLCTrainer);
	if (cacla_trainer != nullptr)
	{
		cacla_trainer->SetActionCovar(action_covar);
	}
}

void cScenarioTrainHike::SetupLLCTrainerActionBounds()
{
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	Eigen::VectorXd action_min;
	Eigen::VectorXd action_max;
	BuildLLCActionBounds(action_min, action_max);

	auto cacla_trainer = std::dynamic_pointer_cast<cCaclaTrainer>(mLLCTrainer);
	if (cacla_trainer != nullptr)
	{
		cacla_trainer->SetActionBounds(action_min, action_max);
	}
}

void cScenarioTrainHike::BuildLLCActionCovar(Eigen::VectorXd& out_covar) const
{
	auto LLC_ctrl = GetDefaultLLC();
	LLC_ctrl->BuildActionExpCovar(out_covar);
}


void cScenarioTrainHike::UpdateExpScene(double time_step, int exp_id, std::shared_ptr<cScenarioExp>& out_exp, bool& out_done)
{
	out_done = false;
	out_exp->Update(time_step);

	if (time_step > 0)
	{
		bool update_exp_mode = false;
		auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(out_exp);
		cScenarioExpHike::eExpMode exp_mode = exp_hike->GetExpMode();
		
		if (exp_mode == cScenarioExpHike::eExpModeHLC)
		{
			bool is_full = exp_hike->IsTupleBufferFull();
			if (is_full)
			{
				if (mEnableTraining)
				{
					const std::vector<tExpTuple>& tuples = exp_hike->GetTuples();
					UpdateTrainer(tuples, exp_id);
					UpdateExpSceneRates(exp_id, out_exp);
					out_done = IsLearnerDone(exp_id);
				}
				exp_hike->ResetTupleBuffer();
				update_exp_mode = true;
			}
		}

		if (exp_mode == cScenarioExpHike::eExpModeLLC)
		{
			bool is_full = exp_hike->IsLLCTupleBufferFull();
			if (is_full)
			{
				if (mEnableTraining)
				{
					const std::vector<tExpTuple>& tuples = exp_hike->GetLLCTuples();
					UpdateLLCTrainer(tuples, exp_id);

					UpdateExpSceneLLCRates(exp_id, out_exp);
					out_done &= IsLLCLearnerDone(exp_id);
				}
				exp_hike->ResetLLCTupleBuffer();
				update_exp_mode = true;
			}
		}

		if (update_exp_mode)
		{
			UpdateExpMode(exp_id, out_exp);
		}
	}
}

void cScenarioTrainHike::UpdateExpMode(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(out_exp);
	cScenarioExpHike::eExpMode exp_mode = exp_hike->GetExpMode();
#if defined(HACK_SOCCER_LLC)
	return;
#endif
	const auto& hlc_learner = mLearners[exp_id];
	const auto& llc_learner = mLLCLearners[exp_id];
	int hlc_iter = hlc_learner->GetIter();
	int llc_iter = llc_learner->GetIter();

	cScenarioExpHike::eExpMode new_exp_mode = CalcExpMode(hlc_iter, llc_iter);
	if (exp_mode != new_exp_mode)
	{
		SetExpMode(new_exp_mode, out_exp);

		if (new_exp_mode == cScenarioExpHike::eExpModeHLC)
		{
			hlc_learner->ResetExpBuffer();
		}
		else if (new_exp_mode == cScenarioExpHike::eExpModeLLC)
		{
			llc_learner->ResetExpBuffer();
		}
	}
}

void cScenarioTrainHike::UpdateLLCTrainer(const std::vector<tExpTuple>& tuples, int exp_id)
{
	auto& learner = mLLCLearners[exp_id];
	learner->Train(tuples);

	int iters = learner->GetIter();
	PrintLLCLearnerInfo(exp_id);

	if ((iters % mLLCItersPerOutput == 0 && iters > 0) || iters == 1)
	{
		learner->OutputModel(mLLCOutputFile);
	}
}

void cScenarioTrainHike::UpdateExpSceneLLCRates(int exp_id, std::shared_ptr<cScenarioExp>& out_exp) const
{
	// this is intentional, when annealing exp rates use the hlc's iteration count
	auto& learner = mLearners[exp_id];
	int iters = learner->GetIter();

	auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(out_exp);
	cCharController::tExpParams exp_params = BuildLLCExpParams(iters);
	exp_hike->SetLLCExpParams(exp_params);
}

bool cScenarioTrainHike::IsLLCLearnerDone(int learner_id) const
{
	const auto& learner = mLLCLearners[learner_id];
	bool done = learner->IsDone();
	int iter = learner->GetIter();
	done |= iter >= mMaxIter;
	return done;
}

cCharController::tExpParams cScenarioTrainHike::BuildLLCExpParams(int iter) const
{
	double lerp = static_cast<double>(iter) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);

	cCharController::tExpParams exp_params;
	exp_params.mRate = (1 - lerp) * mLLCInitExpParams.mRate + lerp * mLLCExpParams.mRate;
	exp_params.mTemp = (1 - lerp) * mLLCInitExpParams.mTemp + lerp * mLLCExpParams.mTemp;
	exp_params.mNoise = (1 - lerp) * mLLCInitExpParams.mNoise + lerp * mLLCExpParams.mNoise;
	exp_params.mBaseActionRate = (1 - lerp) * mLLCInitExpParams.mBaseActionRate + lerp * mLLCExpParams.mBaseActionRate;
	return exp_params;
}

std::shared_ptr<cTerrainRLCharController> cScenarioTrainHike::GetDefaultLLC() const
{
	auto waypoint_ctrl = std::dynamic_pointer_cast<cWaypointController>(GetDefaultController());
	auto LLC_ctrl = std::dynamic_pointer_cast<cTerrainRLCharController>(waypoint_ctrl->GetLLC());
	return LLC_ctrl;
}

void cScenarioTrainHike::PrintLearnerInfo(int exp_id) const
{
	printf("\nHLC Trainer");
	cScenarioImitateTarget::PrintLearnerInfo(exp_id);
}

void cScenarioTrainHike::PrintLLCLearnerInfo(int exp_id) const
{
	const auto& learner = mLLCLearners[exp_id];
	const auto& scene = mExpPool[exp_id];

	auto exp_hike = std::dynamic_pointer_cast<cScenarioExpHike>(scene);
	const auto& exp_params = exp_hike->GetLLCExpParams();

	int iters = learner->GetIter();
	int num_tuples = learner->GetNumTuples();

	printf("\nLLCal Trainer\n");
	printf("Iter %i\n", iters);
	printf("Num Tuples: %i\n", num_tuples);
	printf("Exp Rate: %.5f\n", exp_params.mRate);
	printf("Exp Temp: %.5f\n", exp_params.mTemp);
	printf("Exp Noise: %.5f\n", exp_params.mNoise);
	printf("Exp Base Rate: %.5f\n", exp_params.mBaseActionRate);
}

void cScenarioTrainHike::IntOutputCallback() const
{
	if (mLLCTrainerParams.mIntOutputFile != "")
	{
		int LLC_iter = mLLCTrainer->GetIter();
		int HLC_iter = mTrainer->GetIter();
		std::string ext = cFileUtil::GetExtension(mLLCTrainerParams.mIntOutputFile);
		std::string filename_noext = cFileUtil::RemoveExtension(mLLCTrainerParams.mIntOutputFile);

		char str_buffer[128];
		sprintf(str_buffer, "%010d_%010d", HLC_iter, LLC_iter);
		std::string filename = filename_noext + "_" + str_buffer + "." + ext;
		mLLCTrainer->OutputIntermediateModel(filename);
	}
}