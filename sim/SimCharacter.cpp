#include "SimCharacter.h"
#include <iostream>

#include "SimBox.h"
#include "SimCapsule.h"
#include "RBDUtil.h"

cSimCharacter::tParams::tParams()
{
	mCharFile = "";
	mStateFile = "";
	mInitPos = tVector(0, 0, 0, 0);
	mEnableFallDist = true;
	mEnableSoftContact = true;
}

cSimCharacter::cSimCharacter()
	: mWorld(nullptr)
{
	mFriction = 0.9;
}

cSimCharacter::~cSimCharacter()
{
}

bool cSimCharacter::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	bool succ = true;
	bool succ_skeleton = cCharacter::Init(params.mCharFile);
	succ &= succ_skeleton;

	mWorld = world;

	bool succ_body = true;
	if (succ_skeleton)
	{
		const tVector& root_pos = params.mInitPos;
		succ_body = BuildSimBody(params, root_pos);
		LoadDrawShapeDefs(params.mCharFile, mDrawShapeDefs);
	}

	if (params.mStateFile != "")
	{
		bool succ_state = ReadState(params.mStateFile);

		if (!succ_state)
		{
			printf("Failed to load character state from %s\n", params.mStateFile.c_str());
		}
	}

	if (succ)
	{
		mPose0 = mPose;
		mVel0 = mVel;
	}

	return succ;
}

void cSimCharacter::Clear()
{
	cCharacter::Clear();
	mJoints.clear();
	mBodyParts.clear();
	mBodyDefs.resize(0, 0);
	mDrawShapeDefs.resize(0, 0);
	mWorld.reset();

	if (HasController())
	{
		mController->Clear();
	}
}


void cSimCharacter::Reset()
{
	cCharacter::Reset();
	if (HasController())
	{
		mController->Reset();
	}
	
	ClearJointTorques();
	EnableJoints();

#if defined(ENABLE_TRAINING)
	ClearEffortBuffer();
#endif
}


void cSimCharacter::Update(double time_step)
{
	BuildPose(mPose);
	BuildVel(mVel);

	ClearJointTorques();

	if (HasController())
	{
		mController->Update(time_step);
	}

	// dont clear torques until next frame since they can be useful for visualization
	UpdateJoints();

#if defined(ENABLE_TRAINING)
	UpdateEffortBuffer(time_step);
#endif
}

tVector cSimCharacter::GetRootPos() const
{
	int root_id = GetRootID();
	const std::shared_ptr<cSimObj>& root = mBodyParts[root_id];
	tVector attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, root_id);
	tVector pos = root->LocalToWorldPos(-attach_pt);
	return pos;
}

void cSimCharacter::GetRootRotation(tVector& out_axis, double& out_theta) const
{
	int root_id = GetRootID();
	const cJoint& root = GetJoint(root_id);
	tMatrix rot_mat_test = root.BuildWorldTrans();
	cMathUtil::RotMatToAxisAngle(rot_mat_test, out_axis, out_theta);
}

tQuaternion cSimCharacter::GetRootRotation() const
{
	int root_id = GetRootID();
	const cJoint& root = GetJoint(root_id);
	tMatrix rot_mat_test = root.BuildWorldTrans();
	return cMathUtil::RotMatToQuaternion(rot_mat_test);
}

tVector cSimCharacter::GetRootVel() const
{
	int root_id = GetRootID();
	const std::shared_ptr<cSimObj>& root = mBodyParts[root_id];
	tVector attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, root_id);
	tVector vel = root->GetLinearVelocity(-attach_pt);
	return vel;
}

tVector cSimCharacter::GetRootAngVel() const
{
	const std::shared_ptr<cSimObj>& root = mBodyParts[GetRootID()];
	return root->GetAngularVelocity();
}

const Eigen::MatrixXd& cSimCharacter::GetBodyDefs() const
{
	return mBodyDefs;
}

const Eigen::MatrixXd& cSimCharacter::GetDrawShapeDefs() const
{
	return mDrawShapeDefs;
}

void cSimCharacter::SetRootPos(const tVector& pos)
{
	cCharacter::SetRootPos(pos);
	SetPose(mPose);
}

void cSimCharacter::SetRootRotation(const tQuaternion& q)
{
	cCharacter::SetRootRotation(q);
	SetPose(mPose);
}

void cSimCharacter::SetRootTransform(const tVector& pos, const tQuaternion& rot)
{
	tQuaternion root_rot = cKinTree::GetRootRot(mJointMat, mPose);
	tVector root_vel = cKinTree::GetRootVel(mJointMat, mVel);
	tVector root_ang_vel = cKinTree::GetRootAngVel(mJointMat, mVel);
	tQuaternion delta_rot = rot * root_rot.inverse();

	root_rot = rot;
	root_vel = cMathUtil::QuatRotVec(delta_rot, root_vel);
	root_ang_vel = cMathUtil::QuatRotVec(delta_rot, root_ang_vel);

	cKinTree::SetRootPos(mJointMat, pos, mPose);
	cKinTree::SetRootRot(mJointMat, root_rot, mPose);
	cKinTree::SetRootVel(mJointMat, root_vel, mVel);
	cKinTree::SetRootAngVel(mJointMat, root_ang_vel, mVel);

	SetPose(mPose);
	SetVel(mVel);
}

tQuaternion cSimCharacter::CalcHeadingRot() const
{
	tVector ref_dir = tVector(1, 0, 0, 0);
	tQuaternion root_rot = GetRootRotation();
	tVector rot_dir = cMathUtil::QuatRotVec(root_rot, ref_dir);
	double heading = std::atan2(-rot_dir[2], rot_dir[0]);
	return cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), heading);
}

int cSimCharacter::GetNumBodyParts() const
{
	return static_cast<int>(mBodyParts.size());
}

void cSimCharacter::SetVel(const Eigen::VectorXd& vel)
{
	cCharacter::SetVel(vel);

	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b))
		{
			auto& part = GetBodyPart(b);
			int joint_id = b;

			cSpAlg::tSpVec sv = cRBDUtil::CalcWorldVel(mJointMat, mPose, vel, joint_id);
			tVector pos = GetBodyPartPos(b);
			cSpAlg::tSpTrans world_to_pt = cSpAlg::BuildTrans(pos);
			sv = cSpAlg::ApplyTransM(world_to_pt, sv);

			tVector com_omega = cSpAlg::GetOmega(sv);
			tVector com_vel = cSpAlg::GetV(sv);
			part->SetAngularVelocity(com_omega);
			part->SetLinearVelocity(com_vel);
		}
	}

	if (HasController())
	{
		mController->HandleVelReset();
	}
}

tVector cSimCharacter::CalcJointPos(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	tVector pos;
	
	if (joint.IsValid())
	{
		pos = joint.GetPos();
	}
	else if (joint_id == GetRootID())
	{
		pos = GetRootPos();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		pos = parent_part->LocalToWorldPos(attach_pt);
	}
	return pos;
}

tVector cSimCharacter::CalcJointVel(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	tVector vel;

	if (joint.IsValid())
	{
		vel = joint.GetWorldVel();
	}
	else if (joint_id == GetRootID())
	{
		vel = GetRootVel();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		vel = parent_part->GetLinearVelocity(attach_pt);
	}

	return vel;
}

void cSimCharacter::CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	const auto& joint = mJoints[joint_id];
	if (joint.IsValid())
	{
		joint.CalcWorldRotation(out_axis, out_theta);
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		const auto& parent_part = GetBodyPart(parent_id);
		parent_part->GetRotation(out_axis, out_theta);
	}
}

tQuaternion cSimCharacter::CalcJointWorldRotation(int joint_id) const
{
	tQuaternion rot = tQuaternion::Identity();
	const auto& joint = mJoints[joint_id];
	if (joint.IsValid())
	{
		rot = joint.CalcWorldRotation();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		const auto& parent_part = GetBodyPart(parent_id);
		rot = parent_part->GetRotation();
	}

	return rot;
}

tVector cSimCharacter::CalcCOM() const
{
	tVector com = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_com = part->GetPos();

			com += mass * curr_com;
			total_mass += mass;
		}
	}
	com /= total_mass;
	return com;
}

tVector cSimCharacter::CalcCOMVel() const
{
	tVector com_vel = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_vel = part->GetLinearVelocity();

			com_vel += mass * curr_vel;
			total_mass += mass;
		}
	}
	com_vel /= total_mass;
	return com_vel;
}

void cSimCharacter::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min[0] = std::numeric_limits<double>::infinity();
	out_min[1] = std::numeric_limits<double>::infinity();
	out_min[2] = std::numeric_limits<double>::infinity();

	out_max[0] = -std::numeric_limits<double>::infinity();
	out_max[1] = -std::numeric_limits<double>::infinity();
	out_max[2] = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = GetBodyPart(i);

			tVector curr_min = tVector::Zero();
			tVector curr_max = tVector::Zero();
			part->CalcAABB(curr_min, curr_max);

			out_min = out_min.cwiseMin(curr_min);
			out_max = out_max.cwiseMax(curr_max);
		}
	}
}

const cJoint& cSimCharacter::GetJoint(int joint_id) const
{
	return mJoints[joint_id];
}

cJoint& cSimCharacter::GetJoint(int joint_id)
{
	return mJoints[joint_id];
}

const std::shared_ptr<cSimObj>& cSimCharacter::GetBodyPart(int idx) const
{
	return mBodyParts[idx];
}

std::shared_ptr<cSimObj>& cSimCharacter::GetBodyPart(int idx)
{
	return mBodyParts[idx];
}

tVector cSimCharacter::GetBodyPartPos(int idx) const
{
	auto& part = GetBodyPart(idx);
	tVector pos = part->GetPos();
	return pos;
}

tVector cSimCharacter::GetBodyPartVel(int idx) const
{
	auto& part = GetBodyPart(idx);
	tVector vel = part->GetLinearVelocity();
	return vel;
}

const std::shared_ptr<cSimObj>& cSimCharacter::GetRootPart() const
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

std::shared_ptr<cSimObj> cSimCharacter::GetRootPart()
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

void cSimCharacter::RegisterContacts(int contact_flag, int filter_flag)
{
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			std::shared_ptr<cSimObj>& part = mBodyParts[i];
			part->RegisterContact(contact_flag, filter_flag);
		}
	}
}

bool cSimCharacter::HasFallen() const
{
	return false;
}

bool cSimCharacter::HasStumbled() const
{
	return false;
}

bool cSimCharacter::HasExploded() const
{
	const double dist_threshold = 0.02 * 0.02;
	bool exploded = false;

	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			const auto& parent = joint.GetParent();
			const auto& child = joint.GetChild();
			const tVector& anchor_parent = joint.GetParentAnchor();
			const tVector& anchor_child = joint.GetChildAnchor();

			tVector parent_pos = parent->LocalToWorldPos(anchor_parent);
			tVector child_pos = child->LocalToWorldPos(anchor_child);

			tVector delta = child_pos - parent_pos;
			double dist = delta.squaredNorm();

			if (dist > dist_threshold)
			{
				exploded = true;
				break;
			}
		}
	}

	return exploded;
}

bool cSimCharacter::IsInContact() const
{
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			if (IsInContact(i))
			{
				return true;
			}
		}
	}
	return false;
}

bool cSimCharacter::IsInContact(int idx) const
{
	return GetBodyPart(idx)->IsInContact();
}

tVector cSimCharacter::GetContactPt(int idx) const
{
	return GetBodyPart(idx)->GetContactPt();
}

int cSimCharacter::GetState() const
{
	if (HasController())
	{
		return mController->GetState();
	}
	assert(false);
	return 0;
}

double cSimCharacter::GetPhase() const
{
	if (HasController())
	{
		return mController->GetPhase();
	}
	assert(false);
	return 0;
}


void cSimCharacter::SetController(std::shared_ptr<cCharController> ctrl)
{
	RemoveController();
	mController = ctrl;
}

void cSimCharacter::RemoveController()
{
	if (HasController())
	{
		mController.reset();
	}
}

bool cSimCharacter::HasController() const
{
	return mController != nullptr;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController()
{
	return mController;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController() const
{
	return mController;
}

void cSimCharacter::EnableController(bool enable)
{
	if (HasController())
	{
		mController->SetActive(enable);
	}
}

void cSimCharacter::ApplyControlForces(const Eigen::VectorXd& tau)
{
	assert(tau.size() == GetNumDof());
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = GetParamOffset(j);
			int param_size = GetParamSize(j);
			Eigen::VectorXd curr_tau = tau.segment(param_offset, param_size);
			joint.AddTau(curr_tau);
		}
	}
}

void cSimCharacter::PlayPossum()
{
	if (HasController())
	{
		mController->SetMode(cController::eModePassive);
	}
}


void cSimCharacter::SetPose(const Eigen::VectorXd& pose)
{
	cCharacter::SetPose(pose);

	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			auto& curr_part = mBodyParts[i];
			tVector axis;
			double theta;
			cKinTree::CalcBodyPartRotation(mJointMat, mBodyDefs, mPose, i, axis, theta);
			tVector pos = cKinTree::CalcBodyPartPos(mJointMat, mBodyDefs, mPose, i);

			curr_part->SetPos(pos);
			curr_part->SetRotation(axis, theta);
		}
	}

	if (HasController())
	{
		mController->HandlePoseReset();
	}
}

bool cSimCharacter::BuildSimBody(const tParams& params, const tVector& root_pos)
{
	bool succ = true;
	succ = LoadBodyDefs(params.mCharFile, mBodyDefs);

	int num_joints = GetNumJoints();
	mBodyParts.resize(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		auto& curr_part = mBodyParts[j];
		BuildBodyPart(j, root_pos, mFriction, curr_part);

		if (curr_part != nullptr)
		{
			curr_part->UpdateContact(cWorld::eContactFlagCharacter, cWorld::eContactFlagAll);
			curr_part->DisableDeactivation();
		}
	}

	BuildConstraints();

#if defined(ENABLE_TRAINING)
	mEffortBuffer.resize(GetNumJoints());
	ClearEffortBuffer();
#endif

	return true;
}

bool cSimCharacter::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const
{
	bool succ = cKinTree::LoadBodyDefs(char_file, out_body_defs);
	int num_joints = GetNumJoints();
	int num_body_defs = static_cast<int>(out_body_defs.rows());
	assert(num_joints == num_body_defs);
	return succ;
}

bool cSimCharacter::LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs) const
{
	bool succ = cKinTree::LoadDrawShapeDefs(char_file, out_draw_defs);
	return succ;
}


void cSimCharacter::BuildBodyPart(int part_id, const tVector& root_pos, double friction, std::shared_ptr<cSimObj>& out_part)
{
	cKinTree::eBodyShape shape = cKinTree::GetBodyShape(mBodyDefs, part_id);
	switch (shape)
	{
		case cKinTree::eBodyShapeBox:
		{
			BuildBoxPart(part_id, root_pos, friction, out_part);
			break;
		}
		case cKinTree::eBodyShapeCapsule:
		{
			BuildCapsulePart(part_id, root_pos, friction, out_part);
			break;
		}
		default:
			out_part = nullptr;
			break;
	}
}

void cSimCharacter::BuildBoxPart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimBox::tParams params;
	params.mSize = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimBox> box = std::unique_ptr<cSimBox>(new cSimBox());

	short col_group = GetPartColGroup(part_id);
	short col_mask = GetPartColMask(part_id);
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cSimCharacter::BuildCapsulePart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mBodyDefs, mPose, part_id);
	tVector pos = com_world_trans.col(3);
	pos += root_pos;
	pos[3] = 0;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimCapsule::tParams params;
	tVector size_params = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mHeight = size_params(1);
	params.mRadius = size_params(0);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimCapsule> box = std::unique_ptr<cSimCapsule>(new cSimCapsule());

	short col_group = GetPartColGroup(part_id);
	short col_mask = GetPartColMask(part_id);
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cSimCharacter::BuildConstraints()
{
	int num_joints = GetNumJoints();
	mJoints.clear();
	mJoints.resize(num_joints);
	tVector root_pos = GetRootPos();

	Eigen::VectorXd default_pose;
	cKinTree::BuildDefaultPose(mJointMat, default_pose);

	for (int j = 0; j < num_joints; ++j)
	{
		int parent_id = cKinTree::GetParent(mJointMat, j);
		tVector joint_pos = cCharacter::CalcJointPos(j);
		joint_pos += root_pos;
		std::shared_ptr<cSimObj>& curr_part = mBodyParts[j];

		bool valid_part = IsValidBodyPart(j);
		if (valid_part)
		{
			if (parent_id != cKinTree::gInvalidJointID)
			{
				tVector parent_euler = cKinTree::GetAttachTheta(mJointMat, j);
				tVector child_euler = tVector::Zero();

				if (cKinTree::IsValidBody(mBodyDefs, j))
				{
					child_euler = cKinTree::GetBodyAttachTheta(mBodyDefs, j);
					child_euler = cMathUtil::InvEuler(child_euler);
				}

				tMatrix curr_body_mat = cKinTree::BodyJointTrans(mBodyDefs, j);
				tMatrix child_parent_mat = cKinTree::ParentChildTrans(mJointMat, default_pose, j);
				tMatrix parent_body_mat = cMathUtil::InvRigidMat(cKinTree::BodyJointTrans(mBodyDefs, parent_id));
				tMatrix child_parent_body_mat = cMathUtil::InvRigidMat(parent_body_mat) * child_parent_mat * curr_body_mat;

				tVector ref_axis;
				double ref_theta;
				cMathUtil::RotMatToAxisAngle(child_parent_body_mat, ref_axis, ref_theta);
				ref_theta = -ref_theta;

				std::shared_ptr<cSimObj>& parent = mBodyParts[parent_id];

				tVector pos_child = curr_part->WorldToLocalPos(joint_pos);
				tVector pos_parent = parent->WorldToLocalPos(joint_pos);

				cWorld::tJointParams joint_params;
				joint_params.mType = cKinTree::GetJointType(mJointMat, j);
				joint_params.mAnchor0 = pos_parent;
				joint_params.mAnchor1 = pos_child;
				joint_params.mEulerAngles0 = parent_euler;
				joint_params.mEulerAngles1 = child_euler;
				joint_params.mLimLow = cKinTree::GetJointLimLow(mJointMat, j);
				joint_params.mLimHigh = cKinTree::GetJointLimHigh(mJointMat, j);
				joint_params.mEnableAdjacentCollision = false;
				joint_params.mRefTheta = ref_theta;
				joint_params.mJointChildTrans = cMathUtil::InvRigidMat(curr_body_mat);
				joint_params.mTorqueLimit = cKinTree::GetTorqueLimit(mJointMat, j);
				joint_params.mForceLimit = cKinTree::GetForceLimit(mJointMat, j);

				cJoint& curr_joint = mJoints[j];
				curr_joint.Init(mWorld, parent, curr_part, joint_params);
			}
			else
			{
				// setup root
				tMatrix curr_body_mat = cKinTree::BodyJointTrans(mBodyDefs, j);
				tMatrix child_parent_mat = cKinTree::ParentChildTrans(mJointMat, default_pose, j);
				tMatrix child_parent_body_mat = child_parent_mat * curr_body_mat;

				tVector ref_axis;
				double ref_theta;
				cMathUtil::RotMatToAxisAngle(child_parent_body_mat, ref_axis, ref_theta);
				ref_theta = -ref_theta;
			
				cWorld::tJointParams joint_params;
				joint_params.mType = cKinTree::GetJointType(mJointMat, j);
				joint_params.mAnchor0 = tVector::Zero();
				joint_params.mAnchor1 = curr_part->WorldToLocalPos(joint_pos);
				joint_params.mEulerAngles0 = cKinTree::GetAttachTheta(mJointMat, j);;
				joint_params.mLimLow = cKinTree::GetJointLimLow(mJointMat, j);
				joint_params.mLimHigh = cKinTree::GetJointLimHigh(mJointMat, j);
				joint_params.mEnableAdjacentCollision = false;
				joint_params.mRefTheta = ref_theta;
				joint_params.mJointChildTrans = cMathUtil::InvRigidMat(curr_body_mat);

				const double torque_lim = cKinTree::GetTorqueLimit(mJointMat, j);
				cJoint& curr_joint = mJoints[j];
				curr_joint.Init(mWorld, nullptr, curr_part, joint_params);
			}
		
			tVector linear_factor;
			tVector angular_factor;
			BuildConsFactor(j, linear_factor, angular_factor);
			curr_part->Constrain(linear_factor, angular_factor);
		}
	}
}

void cSimCharacter::BuildConsFactor(int joint_id, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, joint_id);
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	out_linear_factor.setOnes();
	out_angular_factor.setOnes();

	if (is_root)
	{
		BuildRootConsFactor(joint_type, out_linear_factor, out_angular_factor);
	}
}

void cSimCharacter::BuildRootConsFactor(cKinTree::eJointType joint_type, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePlanar:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePrismatic:
		out_linear_factor = tVector(0, 0, 1, 0);
		out_angular_factor = tVector::Zero();
		break;
	case cKinTree::eJointTypeFixed:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector::Zero();
		break;
	case cKinTree::eJointTypeNone:
		out_linear_factor = tVector::Ones();
		out_angular_factor = tVector::Ones();
		break;
	case cKinTree::eJointTypeSpherical:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector(1, 1, 1, 0);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

bool cSimCharacter::IsValidBodyPart(int idx) const
{
	return mBodyParts[idx] != nullptr;
}

bool cSimCharacter::EnableBodyPartFallContact(int idx) const
{
	return cKinTree::GetBodyEnableFallContact(mBodyDefs, idx);
}

tMatrix cSimCharacter::BuildJointWorldTrans(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	if (joint.IsValid())
	{
		return joint.BuildWorldTrans();
	}
	else
	{
		return cCharacter::BuildJointWorldTrans(joint_id);
	}
}

void cSimCharacter::ClearJointTorques()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			joint.ClearTau();
		}
	}
}

void cSimCharacter::EnableJoints()
{
	// sometimes if the simulation blowsup, joints will be disabled
	// so re-enable them as needed
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			joint.SetEnable(true);
		}
	}
}

void cSimCharacter::ClearBodyForces()
{
	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b))
		{
			auto& part = GetBodyPart(b);
			part->ClearForces();
		}
	}
}

void cSimCharacter::UpdateJoints()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			// if (joint.) would be nice to check joint type and apply force or troque here.
			joint.ApplyTau();
		}
	}
}

short cSimCharacter::GetPartColGroup(int part_id) const
{
	return GetPartColMask(part_id);
}

short cSimCharacter::GetPartColMask(int part_id) const
{
	int col_group = cKinTree::GetBodyColGroup(mBodyDefs, part_id);
	assert(col_group < static_cast<int>(sizeof(short) * 8));

	short flags;
	if (col_group == 0)
	{
		flags = cContactManager::gFlagNone;
	}
	else if (col_group == -1)
	{
		flags = cContactManager::gFlagAll;
	}
	else
	{
		flags = 1 << col_group;
	}
	return flags;
}

tVector cSimCharacter::GetPartColor(int part_id) const
{
	return cKinTree::GetBodyColor(mBodyDefs, part_id);
}

bool cSimCharacter::HasDrawShapes() const
{
	return mDrawShapeDefs.size() > 0;
}

const std::shared_ptr<cWorld>& cSimCharacter::GetWorld() const
{
	return mWorld;
}

void cSimCharacter::BuildJointPose(int joint_id, Eigen::VectorXd& out_pose) const
{
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	if (is_root)
	{
		int param_size = GetParamSize(joint_id);
		out_pose = Eigen::VectorXd::Zero(param_size);
		assert(out_pose.size() == cKinTree::gRootDim);

		tVector root_pos = GetRootPos();
		tQuaternion root_rot = GetRootRotation();
		out_pose.segment(0, cKinTree::gPosDim) = root_pos.segment(0, cKinTree::gPosDim);
		out_pose(cKinTree::gPosDim) = root_rot.w();
		out_pose(cKinTree::gPosDim + 1) = root_rot.x();
		out_pose(cKinTree::gPosDim + 2) = root_rot.y();
		out_pose(cKinTree::gPosDim + 3) = root_rot.z();
	}
	else
	{
		const cJoint& joint = GetJoint(joint_id);
		joint.BuildPose(out_pose);
	}
}

void cSimCharacter::BuildJointVel(int joint_id, Eigen::VectorXd& out_pose) const
{
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);
	if (is_root)
	{
		int param_size = GetParamSize(joint_id);
		out_pose = Eigen::VectorXd::Zero(param_size);
		assert(out_pose.size() == cKinTree::gRootDim);

		tVector root_vel = GetRootVel();
		tVector ang_vel = GetRootAngVel();
		out_pose.segment(0, cKinTree::gPosDim) = root_vel.segment(0, cKinTree::gPosDim);
		out_pose.segment(cKinTree::gPosDim, cKinTree::gRotDim) = ang_vel.segment(0, cKinTree::gRotDim);
	}
	else
	{
		const cJoint& joint = GetJoint(joint_id);
		joint.BuildVel(out_pose);
	}
}

void cSimCharacter::BuildPose(Eigen::VectorXd& out_pose) const
{
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_pose.resize(num_dof);
	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd joint_pose;
		BuildJointPose(j, joint_pose);

		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		assert(joint_pose.size() == param_size);
		out_pose.segment(param_offset, param_size) = joint_pose;
	}
}

void cSimCharacter::BuildVel(Eigen::VectorXd& out_vel) const
{
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_vel.resize(num_dof);

	for (int j = 0; j < num_joints; ++j)
	{
		Eigen::VectorXd joint_vel;
		BuildJointVel(j, joint_vel);

		int param_offset = GetParamOffset(j);
		int param_size = GetParamSize(j);
		assert(joint_vel.size() == param_size);
		out_vel.segment(param_offset, param_size) = joint_vel;
	}
}

#if defined(ENABLE_TRAINING)
// effort measured as sum of squared torques
double cSimCharacter::CalcEffort() const
{
	double effort = 0;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cJoint& joint = mJoints[j];
		if (joint.IsValid())
		{
			effort += mEffortBuffer[j];
		}
	}
	return effort;
}

void cSimCharacter::ClearEffortBuffer()
{
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		mEffortBuffer[j] = 0;
	}
}

void cSimCharacter::UpdateEffortBuffer(double time_step)
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			tVector torque = joint.GetTotalTorque();
			tVector force = joint.GetTotalForce();
			double effort = torque.squaredNorm() + force.squaredNorm();
			mEffortBuffer[j] += effort * time_step;
		}
	}
}
#endif // ENABLE_TRAINING
