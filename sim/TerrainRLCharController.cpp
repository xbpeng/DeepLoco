#include "TerrainRLCharController.h"
#include "sim/SimCharacter.h"
#include <iostream>
#include <ctime>
#include "util/json/json.h"

const int gDefaultNumGroundSamples = 200;
const double gViewMin = -0.5;
const double gDefaultViewDist = 10;
const int gPosDim = cKinTree::gPosDim - 1;

#if defined(ENABLE_DEBUG_VISUALIZATION)
const int gPoliValLogSize = 50;
#endif // ENABLE_DEBUG_VISUALIZATION

cTerrainRLCharController::tAction::tAction()
{
	mID = gInvalidIdx;
	mParams.resize(0);
	mLogp = 0;
}

cTerrainRLCharController::cTerrainRLCharController() : cNNController()
{
    SetNumGroundSamples(gDefaultNumGroundSamples);
	SetViewDist(gDefaultViewDist);
	mSkipDecideAction = false;
	mGroundSampleTrans.setIdentity();
}

cTerrainRLCharController::~cTerrainRLCharController()
{
}

void cTerrainRLCharController::Init(cSimCharacter* character)
{
	// param_file should contain parameters for the pd controllers
	cNNController::Init(character);
	ResetParams();
	InitPoliState();
	InitCurrAction();

	InitGroundSamples();

	mValid = true;

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Reserve(gPoliValLogSize);
	mPoliValLog.Clear();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cTerrainRLCharController::Reset()
{
	ApplyAction(GetDefaultAction());
	cNNController::Reset();
	ResetParams();

	InitGroundSamples();

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Clear();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cTerrainRLCharController::Clear()
{
	cNNController::Clear();
	ResetParams();
	mCurrAction.mID = gInvalidIdx;
}

void cTerrainRLCharController::Update(double time_step)
{
	cNNController::Update(time_step);
	UpdateCalcTau(time_step, mTau);
	UpdateApplyTau(mTau);
}

void cTerrainRLCharController::SkipDecideAction()
{
	mSkipDecideAction = true;
}

void cTerrainRLCharController::GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int action_size = GetPoliActionSize();
	out_min = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
	out_max = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
}

double cTerrainRLCharController::CalcActionLogp() const
{
	return mCurrAction.mLogp;
}

void cTerrainRLCharController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cNNController::BuildNNInputOffsetScaleTypes(out_types);

	int ground_offset = GetPoliStateFeatureOffset(ePoliStateGround);
	int ground_size = GetPoliStateFeatureSize(ePoliStateGround);
	for (int i = 0; i < ground_size; ++i)
	{
		out_types[ground_offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

void cTerrainRLCharController::BuildActionExpCovar(Eigen::VectorXd& out_covar) const
{
	out_covar = Eigen::VectorXd::Ones(GetPoliActionSize());
}

const cTerrainRLCharController::tAction& cTerrainRLCharController::GetCurrAction() const
{
	return mCurrAction;
}

void cTerrainRLCharController::SampleAction(tAction& out_action)
{
	// make copy of state to avoid side effects in ExploreAction that might change it
	// (e.g. CtStochController)
	Eigen::VectorXd state = mPoliState;
	ExploreAction(state, out_action);
}

void cTerrainRLCharController::SetGround(std::shared_ptr<cGround> ground)
{
	mGround = ground;
}

int cTerrainRLCharController::GetCurrActionID() const
{
	return mCurrAction.mID;
}

int cTerrainRLCharController::GetNumGroundSamples() const
{
	return mNumGroundSamples;
}

void cTerrainRLCharController::SetNumGroundSamples(int num_ground_samples)
{
	mNumGroundSamples = num_ground_samples;
}

tVector cTerrainRLCharController::GetGroundSample(int s) const
{
	tVector pos = CalcGroundSamplePos(s);
	double h = mGroundSamples[s];
	pos[1] += h;
	return pos;
}

tMatrix cTerrainRLCharController::GetGroundSampleTrans() const
{
	return mGroundSampleTrans;
}

void cTerrainRLCharController::GetViewBound(tVector& out_min, tVector& out_max) const
{
	double view_dist = GetViewDist();
	tVector root_pos = mChar->GetRootPos();
	root_pos[1] = 0;
	out_min = root_pos + tVector(-2, 0, -2, 0);
	out_max = root_pos + tVector(view_dist, 0, 2, 0);
}

const Eigen::VectorXd& cTerrainRLCharController::GetTau() const
{
	return mTau;
}

int cTerrainRLCharController::GetPoliStateSize() const
{
	int state_size = 0;
	for (int i = 0; i < ePoliStateMax; ++i)
	{
		state_size += GetPoliStateFeatureSize(static_cast<ePoliState>(i));
	}
	return state_size;
}

bool cTerrainRLCharController::IsOffPolicy() const
{
	return mIsOffPolicy;
}

int cTerrainRLCharController::GetPoliActionSize() const
{
	return GetNumActions();
}

void cTerrainRLCharController::RecordPoliState(Eigen::VectorXd& out_state) const
{
	out_state = mPoliState;
}

void cTerrainRLCharController::ResetParams()
{
	mPhase = 0;
	mFirstCycle = true;
	mIsOffPolicy = false;
	mGroundSampleTrans.setIdentity();
	mSkipDecideAction = false;
}

void cTerrainRLCharController::InitPoliState()
{
	mPoliState = Eigen::VectorXd::Zero(GetPoliStateSize());
}

void cTerrainRLCharController::InitCurrAction()
{
	mCurrAction.mID = gInvalidIdx;
	mCurrAction.mParams = Eigen::VectorXd::Zero(GetNumParams());
}

void cTerrainRLCharController::InitGroundSamples()
{
	mGroundSamples = Eigen::VectorXd::Zero(GetNumGroundSamples());
}

void cTerrainRLCharController::ApplyAction(int action_id)
{
	BuildBaseAction(action_id, mCurrAction);
	ApplyAction(mCurrAction);
}

void cTerrainRLCharController::ApplyAction(const tAction& action)
{
	mCurrAction = action;
	PostProcessAction(mCurrAction);
	NewCycleUpdate();
}

void cTerrainRLCharController::NewCycleUpdate()
{
}

void cTerrainRLCharController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

bool cTerrainRLCharController::IsOptParam(int param_idx) const
{
	return true;
}

void cTerrainRLCharController::PostProcessParams(Eigen::VectorXd& out_params) const
{
}

void cTerrainRLCharController::PostProcessAction(tAction& out_action) const
{
}

tMatrix cTerrainRLCharController::BuildGroundSampleTrans() const
{
	double ground_h = 0;
	if (mGround != nullptr)
	{
		tVector root_pos = mChar->GetRootPos();
		ground_h = SampleGroundHeight(root_pos);

		if (!std::isfinite(ground_h))
		{
			ground_h = 0;
		}
	}
	
	tMatrix trans = mChar->BuildOriginTrans();
	trans = cMathUtil::InvRigidMat(trans);
	trans(1, 3) += ground_h;
	return trans;
}

void cTerrainRLCharController::ParseGround()
{
	mGroundSampleTrans = BuildGroundSampleTrans();
	if (HasGround())
	{
		SampleGround(mGroundSamples);
	}
	else
	{
		mGroundSamples.setZero();
	}
}

bool cTerrainRLCharController::HasGround() const
{
	return mGround != nullptr;
}

void cTerrainRLCharController::SampleGround(Eigen::VectorXd& out_samples) const
{
	for (int i = 0; i < GetNumGroundSamples(); ++i)
	{
		tVector sample_pos = CalcGroundSamplePos(i);
		double h = SampleGroundHeight(sample_pos);
		h -= mGroundSampleTrans(1, 3);
		out_samples[i] = h;
	}
}

tVector cTerrainRLCharController::CalcGroundSamplePos(int s) const
{
	double view_dist = GetViewDist();
	const int num_samples = GetNumGroundSamples();
	double dist = ((view_dist - gViewMin) * s) / (num_samples - 1) + gViewMin;
	tVector sample_pos = tVector(dist, 0, 0, 1);
	sample_pos = mGroundSampleTrans * sample_pos;
	sample_pos[3] = 0;
	return sample_pos;
}

double cTerrainRLCharController::SampleGroundHeight(const tVector& pos) const
{
	double h = 0;
	if (mGround != nullptr)
	{
		h = mGround->SampleHeight(pos);
	}
	return h;
}

bool cTerrainRLCharController::SampleGroundHeightVel(const tVector& pos, double& out_h, tVector& out_vel) const
{
	bool valid_sample = false;
	out_h = 0;
	out_vel.setZero();
	
	if (mGround != nullptr)
	{
		mGround->SampleHeightVel(pos, out_h, out_vel, valid_sample);
	}
	return valid_sample;
}

void cTerrainRLCharController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	int state_size = GetPoliStateSize();
	out_state.resize(state_size);

	Eigen::VectorXd ground;
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	BuildPoliStateGround(ground);
	BuildPoliStatePose(pose);
	BuildPoliStateVel(vel);

	int ground_offset = GetPoliStateFeatureOffset(ePoliStateGround);
	int ground_size = GetPoliStateFeatureSize(ePoliStateGround);
	int pose_offset = GetPoliStateFeatureOffset(ePoliStatePose);
	int pose_size = GetPoliStateFeatureSize(ePoliStatePose);
	int vel_offset = GetPoliStateFeatureOffset(ePoliStateVel);
	int vel_size = GetPoliStateFeatureSize(ePoliStateVel);

	out_state.segment(ground_offset, ground_size) = ground;
	out_state.segment(pose_offset, pose_size) = pose;
	out_state.segment(vel_offset, vel_size) = vel;
}

void cTerrainRLCharController::BuildPoliStateGround(Eigen::VectorXd& out_ground) const
{
	out_ground = mGroundSamples;
}

void cTerrainRLCharController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::VectorXd& pose = mChar->GetPose();
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, pose);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = SampleGroundHeight(root_pos);
	tVector root_pos_rel = root_pos;
	root_pos_rel[1] -= ground_h;

	root_pos_rel[3] = 1;
	root_pos_rel = origin_trans * root_pos_rel;
	root_pos_rel[3] = 0;

	out_pose = Eigen::VectorXd::Zero(GetPoliStateFeatureSize(ePoliStatePose));
	out_pose[0] = root_pos_rel[1];
	int num_parts = mChar->GetNumBodyParts();

	int idx = 1;
	for (int i = 1; i < num_parts; ++i)
	{
		if (mChar->IsValidBodyPart(i))
		{
			const auto& curr_part = mChar->GetBodyPart(i);
			tVector curr_pos = curr_part->GetPos();
			curr_pos[1] -= ground_h;

			curr_pos[3] = 1;
			curr_pos = origin_trans * curr_pos;
			curr_pos[3] = 0;
			curr_pos -= root_pos_rel;

			out_pose.segment(idx, gPosDim) = curr_pos.segment(0, gPosDim);
			idx += gPosDim;
		}
	}
}

void cTerrainRLCharController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	out_vel.resize(GetPoliStateFeatureSize(ePoliStateVel));
	int num_parts = mChar->GetNumBodyParts();

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::VectorXd& pose = mChar->GetPose();
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, pose);

	tVector root_pos = mChar->GetRootPos();
	double ground_h = 0;
	tVector ground_vel = tVector::Zero();
	bool valid_sample = SampleGroundHeightVel(root_pos, ground_h, ground_vel);
	assert(valid_sample);
	
	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		tVector curr_vel = mChar->GetBodyPartVel(i);
		curr_vel -= ground_vel; // in case ground is moving
		curr_vel = origin_trans * curr_vel;
		out_vel.segment(idx, gPosDim) = curr_vel.segment(0, gPosDim);
		idx += gPosDim;
	}
}

int cTerrainRLCharController::GetPoliStateFeatureOffset(ePoliState feature) const
{
	int offset = 0;
	switch (feature)
	{
	case ePoliStateGround:
		offset = 0;
		break;
	case ePoliStatePose:
		offset = GetPoliStateFeatureSize(ePoliStateGround);
		break;
	case ePoliStateVel:
		offset = GetPoliStateFeatureSize(ePoliStateGround) + GetPoliStateFeatureSize(ePoliStatePose);
		break;
	default:
		assert(false); // unsupported poli state param
		break;
	}
	return offset;
}

int cTerrainRLCharController::GetPoliStateFeatureSize(ePoliState feature) const
{
	int size = 0;
	switch (feature)
	{
	case ePoliStateGround:
		size = GetGroundFeatureSize();
		break;
	case ePoliStatePose:
		size = GetPoseFeatureSize();
		break;
	case ePoliStateVel:
		size = GetVelFeatureSize();
		break;
	default:
		assert(false); // unsupported poli state param
		break;
	}
	return size;
}

int cTerrainRLCharController::GetGroundFeatureSize() const
{
	return GetNumGroundSamples();
}

int cTerrainRLCharController::GetPoseFeatureSize() const
{
	return mChar->GetNumBodyParts() * gPosDim - 1; // -1 for root x
}

int cTerrainRLCharController::GetVelFeatureSize() const
{
	return mChar->GetNumBodyParts() * gPosDim;
}

void cTerrainRLCharController::BuildDefaultAction(tAction& out_action) const
{
	BuildBaseAction(GetDefaultAction(), out_action);
}

void cTerrainRLCharController::BuildRandBaseAction(tAction& out_action) const
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	BuildBaseAction(a, out_action);

#if defined (ENABLE_DEBUG_PRINT)
	printf("rand action: %i\n", a);
#endif
}

void cTerrainRLCharController::DebugPrintAction(const tAction& action) const
{
	printf("Action ID: %i\n", action.mID);
	printf("Action params: ");
	for (int i = 0; i < action.mParams.size(); ++i)
	{
		printf("%.4f\t", action.mParams[i]);
	}
	printf("\n");
}

#if defined(ENABLE_DEBUG_VISUALIZATION)
const cCircularBuffer<double>& cTerrainRLCharController::GetPoliValLog() const
{
	return mPoliValLog;
}

void cTerrainRLCharController::GetVisCharacterFeatures(Eigen::VectorXd& out_features) const
{
	int pose_offset = GetPoliStateFeatureOffset(ePoliStatePose);
	int pose_size = GetPoliStateFeatureSize(ePoliStatePose);
	int vel_offset = GetPoliStateFeatureOffset(ePoliStateVel);
	int vel_size = GetPoliStateFeatureSize(ePoliStateVel);

	Eigen::VectorXd norm_features;
	RecordPoliState(norm_features);
	if (HasNet())
	{
		mNet->NormalizeInput(norm_features);
	}

	out_features.resize(pose_size + vel_size);
	out_features.segment(0, pose_size) = norm_features.segment(pose_offset, pose_size);
	out_features.segment(pose_size, vel_size) = norm_features.segment(vel_offset, vel_size);
}

void cTerrainRLCharController::GetVisTerrainFeatures(Eigen::VectorXd& out_features) const
{
	int ground_offset = GetPoliStateFeatureOffset(ePoliStateGround);
	int ground_size = GetPoliStateFeatureSize(ePoliStateGround);

	Eigen::VectorXd norm_features;
	RecordPoliState(norm_features);
	if (HasNet())
	{
		mNet->NormalizeInput(norm_features);
	}

	out_features = norm_features.segment(ground_offset, ground_size);
}

void cTerrainRLCharController::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	RecordPoliAction(out_features);
	if (HasNet())
	{
		mNet->NormalizeOutput(out_features);
	}
}

void cTerrainRLCharController::GetVisActionValues(Eigen::VectorXd& out_features) const
{
	out_features = mVisNNOutput;
}
#endif // ENABLE_DEBUG_VISUALIZATION
