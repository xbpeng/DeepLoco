#include "CtController.h"
#include "sim/SimCharacter.h"

cCtController::cCtController() : cBaseControllerCacla()
{
	mExpParams.mBaseActionRate = 0.2;
	mExpParams.mNoise = 0.2;
	mUpdatePeriod = 1 / 30.0;
	mUpdateCounter = std::numeric_limits<double>::infinity();
    mGroundSampleRes = 0;

	mViewDistMin = -mViewDist;
}

cCtController::~cCtController()
{
}

void cCtController::Init(cSimCharacter* character)
{
	cBaseControllerCacla::Init(character);
	SetupActionBounds();
}

void cCtController::Reset()
{
	cBaseControllerCacla::Reset();
}

void cCtController::Clear()
{
	cBaseControllerCacla::Clear();
}

bool cCtController::NewActionUpdate() const
{
	return mUpdateCounter >= (0.9999 * mUpdatePeriod);
}

void cCtController::SetUpdatePeriod(double period)
{
	mUpdatePeriod = period;
}

double cCtController::GetUpdatePeriod() const
{
	return mUpdatePeriod;
}

void cCtController::SetActionParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetPoliActionSize());
	mCurrAction.mParams = params;
	ApplyAction(mCurrAction);
}

void cCtController::UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau)
{
	if (NewActionUpdate() && !mFirstCycle)
	{
		mUpdateCounter = 0;
	}

	mUpdateCounter += time_step;

	if (NewActionUpdate())
	{
		UpdateAction();
		mFirstCycle = false;
	}

	UpdateBuildTau(time_step, out_tau);
}

void cCtController::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mTau = tau;
	mChar->ApplyControlForces(tau);
}

int cCtController::GetPoliActionSize() const
{
	int num_dofs = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	num_dofs -= root_size;
	return num_dofs;
}

void cCtController::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	out_action = mCurrAction.mParams;
}

void cCtController::GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	cBaseControllerCacla::GetPoliActionBounds(out_min, out_max);
	out_min.segment(0, mActionBoundMin.size()) = mActionBoundMin;
	out_max.segment(0, mActionBoundMax.size()) = mActionBoundMax;
}

int cCtController::GetNumActions() const
{
	return 0;
}

int cCtController::GetNumParams() const
{
	return GetPoliActionSize();
}

void cCtController::GetViewBound(tVector& out_min, tVector& out_max) const
{
	tVector origin = CalcGroundSampleOrigin();
	tVector size = CalcGroundSampleSize();
	double max_len = std::max(size[0], size[2]);
	max_len *= std::sqrt(2) * 0.5;

	tMatrix trans = BuildGroundSampleTrans();
	origin[3] = 1;
	origin = trans * origin;
	origin[3] = 0;

	out_min = origin - tVector(max_len, 0, max_len, 0);
	out_max = origin + tVector(max_len, 0, max_len, 0);
}

bool cCtController::IsCurrActionCyclic() const
{
	return false;
}

bool cCtController::LoadControllers(const std::string& file)
{
	return true;
}

void cCtController::ResetParams()
{
	cBaseControllerCacla::ResetParams();
	mUpdateCounter = std::numeric_limits<double>::infinity();
}

int cCtController::GetPosFeatureDim() const
{
	int pos_dim = cKinTree::gPosDim - 1;
	if (Is3D())
	{
		pos_dim = cKinTree::gPosDim;
	}
	return pos_dim;
}

int cCtController::GetRotFeatureDim() const
{
	int rot_dim = 0;
	if (Is3D())
	{
#if defined(ENABLE_MAX_STATE)
		rot_dim = cKinTree::gRotDim;
#endif
	}
	return rot_dim;
}

void cCtController::InitCurrAction()
{
	mCurrAction.mID = gInvalidIdx;
	mCurrAction.mParams = Eigen::VectorXd::Zero(GetPoliActionSize());
}

bool cCtController::ShouldExplore() const
{
	bool explore = false;
	if (EnabledExplore())
	{
		explore = cMathUtil::FlipCoin(mExpParams.mRate);
	}
	return explore;
}

int cCtController::GetNumGroundSamples() const
{
	int res = GetGroundSampleRes();
	return res * res;
}

tVector cCtController::CalcGroundSamplePos(int s) const
{
	tVector origin = CalcGroundSampleOrigin();
	tVector size = CalcGroundSampleSize();

	const int num_samples = GetNumGroundSamples();
	int sample_res = GetGroundSampleRes();
	double u = static_cast<double>(s % sample_res) / (sample_res - 1);
	double v = static_cast<double>(s / sample_res) / (sample_res - 1);

	double x = (u - 0.5) * size[0];
	double z = (v - 0.5) * size[2];

	if (FlipStance())
	{
		z = -z;
	}

	tVector sample_pos = origin + tVector(x, 0, z, 0);
	sample_pos[3] = 1;
	sample_pos = mGroundSampleTrans * sample_pos;
	sample_pos[3] = 0;
	return sample_pos;
}

tVector cCtController::CalcGroundSampleOrigin() const
{
	double max_dist = GetMaxViewDist();
	double min_dist = GetMinViewDist();
	tVector origin = tVector(0.5 * (max_dist + min_dist), 0, 0, 0);
	return origin;
}

tVector cCtController::CalcGroundSampleSize() const
{
	double max_dist = GetMaxViewDist();
	double min_dist = GetMinViewDist();
	double w = max_dist - min_dist;
	return tVector(w, 0, w, 0);
}

int cCtController::GetGroundSampleRes() const
{
	return mGroundSampleRes;
}

void cCtController::SetGroundSampleRes(int sample_res)
{
    mGroundSampleRes = sample_res;	
}

double cCtController::GetMaxViewDist() const
{
	return mViewDist;
}

double cCtController::GetMinViewDist() const
{
	return mViewDistMin;
}

void cCtController::SetupActionBounds()
{
	int action_size = GetPoliActionSize();
	mActionBoundMin = Eigen::VectorXd::Zero(action_size);
	mActionBoundMax = Eigen::VectorXd::Zero(action_size);
	
	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int num_joints = mChar->GetNumJoints();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			Eigen::VectorXd lim_min;
			Eigen::VectorXd lim_max;
			BuildJointActionBounds(j, lim_min, lim_max);
			assert(lim_min.size() == param_size);
			assert(lim_max.size() == param_size);

			mActionBoundMin.segment(param_offset - root_size, param_size) = lim_min;
			mActionBoundMax.segment(param_offset - root_size, param_size) = lim_max;
		}
	}
}

void cCtController::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	out_tau = Eigen::VectorXd::Zero(mChar->GetNumDof());
	
	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(root_id);
	int num_joints = mChar->GetNumJoints();
	for (int j = root_id + 1; j < num_joints; ++j)
	{
		int retarget_joint = RetargetJointID(j);
		int param_offset = mChar->GetParamOffset(j);
		int param_size = mChar->GetParamSize(j);
		int retarget_offset = mChar->GetParamOffset(retarget_joint);
		int retarget_size = mChar->GetParamSize(retarget_joint);
		assert(param_size == retarget_size);

		out_tau.segment(param_offset, param_size) = mCurrAction.mParams.segment(retarget_offset - root_size, retarget_size);
	}
}

void cCtController::UpdateAction()
{
	ParseGround();
	BuildPoliState(mPoliState);

	mIsOffPolicy = true;

	if (HasNet())
	{
		if (!mSkipDecideAction)
		{
			DecideAction(mCurrAction);
		}
		else
		{
			mIsOffPolicy = true;
			mSkipDecideAction = false;
		}
	}

	ApplyAction(mCurrAction);

#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cCtController::DecideAction(tAction& out_action)
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

void cCtController::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(action_size);
	out_scale = Eigen::VectorXd::Ones(action_size);

	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int num_joints = mChar->GetNumJoints();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		const cJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			Eigen::VectorXd curr_offset;
			Eigen::VectorXd curr_scale;
			BuildJointActionOffsetScale(j, curr_offset, curr_scale);
			assert(curr_offset.size() == param_size);
			assert(curr_scale.size() == param_size);

			out_offset.segment(param_offset - root_size, param_size) = curr_offset;
			out_scale.segment(param_offset - root_size, param_size) = curr_scale;
		}
	}
}

void cCtController::BuildActionExpCovar(Eigen::VectorXd& out_covar) const
{
	FetchExpNoiseScale(out_covar);
	out_covar *= mExpParams.mNoise;
	out_covar = out_covar.cwiseProduct(out_covar);
}

void cCtController::ForceActionUpdate()
{
	UpdateAction();
}

void cCtController::ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action)
{
	Eigen::VectorXd params;
	EvalNet(state, params);
	assert(params.size() == GetPoliActionSize());

	//Eigen::VectorXd covar;
	//BuildActionExpCovar(covar);
	double logp = 0;
	
	out_action.mID = gInvalidIdx;
	out_action.mParams = params;
	out_action.mLogp = logp;
}

void cCtController::ExploreAction(Eigen::VectorXd& state, tAction& out_action)
{
#if defined (ENABLE_DEBUG_PRINT)
	//printf("Exploring action\n");
#endif

	ExploitPolicy(state, out_action);
	Eigen::VectorXd old_action = out_action.mParams;
	ApplyExpNoise(out_action);

	PostProcessAction(out_action); // arg this is inefficient

	Eigen::VectorXd covar;
	BuildActionExpCovar(covar);
	double logp = cMathUtil::EvalGaussianLogp(old_action, covar, out_action.mParams);
	out_action.mLogp = logp;
}

void cCtController::PostProcessAction(tAction& out_action) const
{
	//out_action.mParams = out_action.mParams.cwiseMax(mActionBoundMin).cwiseMin(mActionBoundMax);
}

void cCtController::RecordVal()
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
		//printf("Value: %.3f\n", v);
#endif

#endif
	}
}

void cCtController::ApplyExpNoise(tAction& out_action)
{
	int num_params = static_cast<int>(out_action.mParams.size());
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	assert(noise_scale.size() == num_params);

	for (int i = 0; i < num_params; ++i)
	{
		double noise = cMathUtil::RandDoubleNorm(0, mExpParams.mNoise);
		double scale = noise_scale[i];
		noise *= scale;
		out_action.mParams[i] += noise;
	}
}

void cCtController::ApplyAction(int action_id)
{
	// do nothing
}

void cCtController::ApplyAction(const tAction& action)
{
	mCurrAction = action;
	PostProcessAction(mCurrAction);

#if defined (ENABLE_DEBUG_PRINT)
	//DebugPrintAction(mCurrAction);
	//printf("\n");
#endif
}

void cCtController::BuildBaseAction(int action_id, tAction& out_action) const
{
	assert(false); // unsupported method
}

int cCtController::GetPoseFeatureSize() const
{
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();
	return mChar->GetNumBodyParts() * (pos_dim + rot_dim) + 1; // +1 for root y
}

int cCtController::GetVelFeatureSize() const
{
	int size = 0;
	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	if (Is3D())
	{
		size = mChar->GetNumBodyParts() * (pos_dim + rot_dim - 1);
	}
	else
	{
		size = mChar->GetNumBodyParts() * pos_dim;
	}
	return size;
}


void cCtController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::VectorXd& pose = mChar->GetPose();
	tMatrix origin_trans = mChar->BuildOriginTrans();
	tQuaternion origin_quat = cMathUtil::RotMatToQuaternion(origin_trans);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

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

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	int idx = 1;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);
		if (mChar->IsValidBodyPart(part_id))
		{
			const auto& curr_part = mChar->GetBodyPart(part_id);
			tVector curr_pos = curr_part->GetPos();
			curr_pos[1] -= ground_h;

			curr_pos[3] = 1;
			curr_pos = origin_trans * curr_pos;
			curr_pos[3] = 0;
			curr_pos -= root_pos_rel;

			out_pose.segment(idx, pos_dim) = curr_pos.segment(0, pos_dim);
			idx += pos_dim;

#if defined(ENABLE_MAX_STATE)
			if (Is3D())
			{
				tQuaternion curr_quat = curr_part->GetRotation();
				curr_quat = origin_quat * curr_quat;

				if (flip_stance)
				{
					curr_quat = cMathUtil::MirrorQuaternion(curr_quat, cMathUtil::eAxisZ);
				}

				if (curr_quat.w() < 0)
				{
					curr_quat.w() *= -1;
					curr_quat.x() *= -1;
					curr_quat.y() *= -1;
					curr_quat.z() *= -1;
				}
				out_pose.segment(idx, rot_dim) = cMathUtil::QuatToVec(curr_quat).segment(0, rot_dim);
				idx += rot_dim;
			}
#endif
		}
	}
}

void cCtController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	out_vel.resize(GetPoliStateFeatureSize(ePoliStateVel));
	int num_parts = mChar->GetNumBodyParts();

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::VectorXd& pose = mChar->GetPose();
	tMatrix origin_trans = cKinTree::BuildOriginTrans(joint_mat, pose);

	bool flip_stance = FlipStance();
	if (flip_stance)
	{
		origin_trans.row(2) *= -1; // reflect z
	}

	int pos_dim = GetPosFeatureDim();
	int rot_dim = GetRotFeatureDim();

	tVector root_pos = mChar->GetRootPos();
	double ground_h = 0;
	tVector ground_vel = tVector::Zero();
	bool valid_sample = SampleGroundHeightVel(root_pos, ground_h, ground_vel);
	assert(valid_sample);

	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		int part_id = RetargetJointID(i);

		const auto& curr_part = mChar->GetBodyPart(part_id);
		tVector curr_vel = curr_part->GetLinearVelocity();
		curr_vel -= ground_vel;
		curr_vel = origin_trans * curr_vel;

		out_vel.segment(idx, pos_dim) = curr_vel.segment(0, pos_dim);
		idx += pos_dim;

#if defined(ENABLE_MAX_STATE)
		if (Is3D())
		{
			tVector curr_ang_vel = curr_part->GetAngularVelocity();
			curr_ang_vel = origin_trans * curr_ang_vel;
			if (flip_stance)
			{
				curr_ang_vel = -curr_ang_vel;
			}

			out_vel.segment(idx, rot_dim - 1) = curr_ang_vel.segment(0, rot_dim - 1);
			idx += rot_dim - 1;
		}
#endif
	}
}

void cCtController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsTorque(joint_mat, joint_id, out_min, out_max);
}

void cCtController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScaleTorque(joint_mat, joint_id, out_offset, out_scale);
}

cWorld::eSimMode cCtController::GetSimMode() const
{
	const auto& world = mChar->GetWorld();
	cWorld::eSimMode sim_mode = world->GetSimMode();
	return sim_mode;
}

bool cCtController::Is3D() const
{
	cWorld::eSimMode sim_mode = GetSimMode();
	return sim_mode == cWorld::eSimMode3D;
}

bool cCtController::FlipStance() const
{
	return false;
}

int cCtController::RetargetJointID(int joint_id) const
{
	return joint_id;
}
