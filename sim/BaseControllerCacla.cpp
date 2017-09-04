#include "BaseControllerCacla.h"

cBaseControllerCacla::cBaseControllerCacla() : cTerrainRLCharController()
{
	mExpParams.mBaseActionRate = 0.2;
}

cBaseControllerCacla::~cBaseControllerCacla()
{
}

void cBaseControllerCacla::Init(cSimCharacter* character)
{
	cTerrainRLCharController::Init(character);
	BuildCriticNet(mCriticNet);
}

int cBaseControllerCacla::GetPoliActionSize() const
{
	return GetNumOptParams();
}

void cBaseControllerCacla::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	BuildOptParams(out_action);
}

bool cBaseControllerCacla::ValidCritic() const
{
	return mCriticNet->HasNet();
}

void cBaseControllerCacla::BuildCriticNet(std::unique_ptr<cNeuralNet>& out_net) const
{
	out_net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
}

bool cBaseControllerCacla::LoadCriticNet(const std::string& net_file)
{
	bool succ = true;
	mCriticNet->Clear();
	mCriticNet->LoadNet(net_file);

	int input_size = mCriticNet->GetInputSize();
	int output_size = mCriticNet->GetOutputSize();
	int critic_input_size = GetCriticInputSize();
	int critic_output_size = GetCriticOutputSize();

	if (input_size != critic_input_size)
	{
		printf("Network input dimension does not match expected input size (%i (network) vs %i (controller) ).\n", input_size, critic_input_size);
		succ = false;
	}

	if (output_size != critic_output_size)
	{
		printf("Network output dimension does not match expected output size (%i (network) vs %i (controller) ).\n", output_size, critic_output_size);
		succ = false;
	}
	
	if (succ)
	{
		Eigen::VectorXd input_offset;
		Eigen::VectorXd input_scale;
		BuildCriticInputOffsetScale(input_offset, input_scale);
		SetCriticInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		BuildCriticOutputOffsetScale(output_offset, output_scale);
		SetCriticOutputOffsetScale(output_offset, output_scale);
	}
	else
	{
		mCriticNet->Clear();
		assert(false);
	}

	return succ;
}

void cBaseControllerCacla::LoadCriticModel(const std::string& model_file)
{
	mCriticNet->LoadModel(model_file);
}

void cBaseControllerCacla::CopyNet(const cNeuralNet& net)
{
	CopyActorNet(net);
}

void cBaseControllerCacla::CopyActorNet(const cNeuralNet& net)
{
	mNet->CopyModel(net);
}

void cBaseControllerCacla::CopyCriticNet(const cNeuralNet& net)
{
	mCriticNet->CopyModel(net);
}

void cBaseControllerCacla::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cTerrainRLCharController::BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerCacla::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cBaseControllerCacla::SetNNInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	SetActorInputOffsetScale(offset, scale);
}

void cBaseControllerCacla::SetNNOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	SetActorOutputOffsetScale(offset, scale);
}


void cBaseControllerCacla::BuildActorInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerCacla::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int actor_output_size = GetNumOptParams();

	int num_actions = GetNumActions();
	if (num_actions > 0)
	{
		int default_action_id = GetDefaultAction();
		if (default_action_id == gInvalidIdx)
		{
			default_action_id = 0;
		}

		out_offset = Eigen::VectorXd::Zero(actor_output_size);
		out_scale = Eigen::VectorXd::Ones(actor_output_size);

		BuildActionOptParams(default_action_id, out_offset);
		out_offset *= -1;

		if (num_actions > 1)
		{
			out_scale.setZero();
			Eigen::VectorXd param_buffer;
			for (int a = 0; a < num_actions; ++a)
			{
				if (a != default_action_id)
				{
					BuildActionOptParams(a, param_buffer);
					param_buffer += out_offset;
					param_buffer = param_buffer.cwiseAbs();
					out_scale = out_scale.cwiseMax(param_buffer);
				}
			}

			out_scale = out_scale.cwiseInverse();
		}
	}
}

void cBaseControllerCacla::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	cTerrainRLCharController::SetNNInputOffsetScale(offset, scale);
}

void cBaseControllerCacla::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	cTerrainRLCharController::SetNNOutputOffsetScale(offset, scale);
}

void cBaseControllerCacla::BuildCriticInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildNNInputOffsetScale(out_offset, out_scale);
}

void cBaseControllerCacla::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = 1;
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

void cBaseControllerCacla::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mCriticNet->SetInputOffsetScale(offset, scale);
}

void cBaseControllerCacla::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) const
{
	mCriticNet->SetOutputOffsetScale(offset, scale);
}

bool cBaseControllerCacla::ShouldExplore() const
{
	bool explore = false;
	if (EnabledExplore())
	{
		double rand = cMathUtil::RandDouble(0, 1);
		double exp_rate = mExpParams.mRate;
		explore = rand < exp_rate;
	}
	return explore;
}

void cBaseControllerCacla::DecideAction(tAction& out_action)
{
	bool explore = ShouldExplore();

	if (explore)
	{
		mIsOffPolicy = true;
		ExploreAction(mPoliState, out_action);
	}
	else
	{
		mIsOffPolicy = false;
		ExploitPolicy(mPoliState, out_action);
	}
}

const std::unique_ptr<cNeuralNet>& cBaseControllerCacla::GetActor() const
{
	return mNet;
}

const std::unique_ptr<cNeuralNet>& cBaseControllerCacla::GetCritic() const
{
	return mCriticNet;
}

int cBaseControllerCacla::GetActorInputSize() const
{
	return GetPoliStateSize();
}

int cBaseControllerCacla::GetActorOutputSize() const
{
	return GetPoliActionSize();
}

int cBaseControllerCacla::GetCriticInputSize() const
{
	return GetPoliStateSize();
}

int cBaseControllerCacla::GetCriticOutputSize() const
{
	return 1;
}

void cBaseControllerCacla::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	Eigen::VectorXd opt_params;
	EvalNet(state, opt_params);
	assert(opt_params.size() == GetPoliActionSize());

	out_action.mID = gInvalidIdx;
	out_action.mParams = mCurrAction.mParams;

	SetOptParams(opt_params, out_action.mParams);
	
#if defined (ENABLE_DEBUG_PRINT)
	DebugPrintAction(out_action);
	printf("\n");
#endif
}

void cBaseControllerCacla::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
#if defined (ENABLE_DEBUG_PRINT)
	printf("Exploring action\n");
#endif
	
	double rand = cMathUtil::RandDouble();
	if (rand < mExpParams.mBaseActionRate)
	{
		BuildRandBaseAction(out_action);
	}
	else
	{
		ExploitPolicy(state, out_action);
		ApplyExpNoise(out_action);
	}
}

void cBaseControllerCacla::EvalNet(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const
{
	mNet->Eval(x, out_y);
}

void cBaseControllerCacla::RecordVal()
{
	if (ValidCritic())
	{
#if defined(ENABLE_DEBUG_VISUALIZATION)
		Eigen::VectorXd val;
		Eigen::VectorXd critic_x;
		BuildCriticInput(critic_x);
		mCriticNet->Eval(critic_x, val);

		double v = val[0];
		mPoliValLog.Add(v);

#if defined (ENABLE_DEBUG_PRINT)
		printf("Value: %.3f\n", v);
#endif

#endif
	}
}

void cBaseControllerCacla::BuildCriticInput(Eigen::VectorXd& out_x) const
{
	out_x = mPoliState;
}

void cBaseControllerCacla::FetchExpNoiseScale(Eigen::VectorXd& out_noise) const
{
	const Eigen::VectorXd& nn_output_scale = mNet->GetOutputScale();
	out_noise = nn_output_scale.cwiseInverse();
}

void cBaseControllerCacla::ApplyExpNoise(tAction& out_action)
{
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	assert(noise_scale.size() == num_opt_params);

	// for debugging
	Eigen::VectorXd exp_noise = Eigen::VectorXd::Zero(num_opt_params);

	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			double noise = cMathUtil::RandDoubleNorm(0, mExpParams.mNoise);
			double scale = noise_scale[opt_idx];
			noise *= scale;

			out_action.mParams[i] += noise;
			exp_noise[opt_idx] = noise;
			++opt_idx;
		}
	}
}
