#include <iostream>
#include "Joint.h"

cJoint::cJoint()
{
}

cJoint::~cJoint()
{
	Clear();
}

void cJoint::Init(std::shared_ptr<cWorld>& world, std::shared_ptr<cSimObj> parent, std::shared_ptr<cSimObj> child,
				const cWorld::tJointParams& params)
{
	Clear();

	mParams = params;
	mWorld = world;
	mParent = parent;
	mChild = child;

	mCons = mWorld->AddJoint(mParent.get(), mChild.get(), params);
}

void cJoint::Clear()
{
	if (IsValid())
	{
		mWorld->RemoveConstraint(mCons);
	}
	mParams = cWorld::tJointParams();
	mCons.Clear();
	mWorld.reset();
	mParent.reset();
	mChild.reset();

	ClearTau();
}

bool cJoint::IsValid() const
{
	return(mWorld != nullptr) && (mChild != nullptr) && mCons.IsValid();
}

tVector cJoint::CalcAxisWorld() const
{
	tVector axis_rel = GetAxisRel();
	axis_rel[3] = 0;
	tMatrix trans = BuildWorldTrans();
	tVector axis = trans * axis_rel;
	return axis;
}

tVector cJoint::GetAxisRel() const
{
	return tVector(0, 0, 1, 0);
}

bool cJoint::HasParent() const
{
	return mParent != nullptr;
}

cKinTree::eJointType cJoint::GetType() const
{
	return mParams.mType;
}

void cJoint::CalcRotation(tVector& out_axis, double& out_theta) const
{
	assert(IsValid());

	out_axis = tVector(0, 0, 1, 0);
	out_theta = 0;

	switch (mParams.mType)
	{
	case cKinTree::eJointTypeRevolute:
		CalcRotationRevolute(out_axis, out_theta);
		break;
	case cKinTree::eJointTypePrismatic:
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		CalcRotationSpherical(out_axis, out_theta);
		break;
	default:
		printf("Unsupported constraint type for cJoint::CalcRotation()\n");
		break;
	}
}

tQuaternion cJoint::CalcWorldRotation() const
{
	tMatrix mat = BuildWorldTrans();
	tQuaternion q = cMathUtil::RotMatToQuaternion(mat);
	return q;
}

void cJoint::CalcWorldRotation(tVector& out_axis, double& out_theta) const
{
	tMatrix mat = BuildWorldTrans();
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

void cJoint::GetChildRotation(tVector& out_axis, double& out_theta) const
{
	mChild->GetRotation(out_axis, out_theta);
}

void cJoint::GetParentRotation(tVector& out_axis, double& out_theta) const
{
	if (HasParent())
	{
		mParent->GetRotation(out_axis, out_theta);
	}
	else
	{
		out_axis = tVector(0, 0, 1, 0);
		out_theta = 0;
	}
}

tMatrix cJoint::BuildWorldTrans() const
{
	tMatrix mat = mChild->GetWorldTransform();
	mat = mat * mParams.mJointChildTrans;
	return mat;
}

void cJoint::AddTau(const Eigen::VectorXd& tau)
{
	switch (mParams.mType)
	{
	case cKinTree::eJointTypeRevolute:
		AddTauRevolute(tau);
		break;
	case cKinTree::eJointTypePlanar:
		AddTauPlanar(tau);
		break;
	case cKinTree::eJointTypePrismatic:
		AddTauPristmatic(tau);
		break;
	case cKinTree::eJointTypeFixed:
		AddTauFixed(tau);
		break;
	case cKinTree::eJointTypeSpherical:
		AddTauSpherical(tau);
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

const cSpAlg::tSpVec& cJoint::GetTau() const
{
	return mTotalTau;
}

void cJoint::ApplyTau()
{
	switch (mParams.mType)
	{
	case cKinTree::eJointTypeRevolute:
		ApplyTorque();
		break;
	case cKinTree::eJointTypePlanar:
		ApplyTorque();
		ApplyForce();
		break;
	case cKinTree::eJointTypePrismatic:
		ApplyForce();
		break;
	case cKinTree::eJointTypeFixed:
		break;
	case cKinTree::eJointTypeSpherical:
		ApplyTorque();
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

void cJoint::ApplyTorque()
{
	tVector torque = GetTotalTorque();
	ClampTotalTorque(torque);
	SetTotalTorque(torque);

	tMatrix trans = BuildWorldTrans();
	torque[3] = 0;
	torque = trans * torque;

	if (HasParent())
	{
		mParent->ApplyTorque(-torque);
	}
	mChild->ApplyTorque(torque);
}

void cJoint::ApplyForce()
{
	tVector force = GetTotalForce();
	ClampTotalForce(force);
	SetTotalForce(force);

	tMatrix trans = BuildWorldTrans();
	force[3] = 0;
	force = trans * force;

	if (HasParent())
	{
		mParent->ApplyForce(-force);
	}
	mChild->ApplyForce(force);
	// std::cout << "Knee Force: " << force.transpose() << std::endl;
}

void cJoint::ClearTau()
{
	mTotalTau.setZero();
}

tVector cJoint::GetPos() const
{
	const tVector& anchor = mParams.mAnchor1;
	tVector world_pos = mChild->LocalToWorldPos(anchor);
	return world_pos;
}

tVector cJoint::CalcWorldPos(const tVector& local_pos) const
{
	tVector world_pos = local_pos;
	world_pos[3] = 1;
	tMatrix trans = BuildWorldTrans();
	world_pos = trans * world_pos;
	world_pos[3] = 0;

	return world_pos;
}

tVector cJoint::GetWorldVel() const
{
	const tVector anchor = mParams.mAnchor1;
	tVector world_vel = mChild->GetLinearVelocity(anchor);
	return world_vel;
}

tVector cJoint::CalcWorldVel(const tVector& local_pos) const
{
	tVector attach_pt = mParams.mAnchor1 + local_pos;
	tVector world_vel = mChild->GetLinearVelocity(attach_pt);
	return world_vel;
}

double cJoint::GetTorqueLimit() const
{
	return mParams.mTorqueLimit;
}

double cJoint::GetForceLimit() const
{
	return mParams.mForceLimit;
}

void cJoint::SetTorqueLimit(double lim)
{
	mParams.mTorqueLimit = lim;
}

void cJoint::SetForceLimit(double lim)
{
	mParams.mForceLimit = lim;
}

const tVector& cJoint::GetParentAnchor() const
{
	return mParams.mAnchor0;
}

const tVector& cJoint::GetChildAnchor() const
{
	return mParams.mAnchor1;
}

const std::shared_ptr<cSimObj>& cJoint::GetParent() const
{
	return mParent;
}

const std::shared_ptr<cSimObj>& cJoint::GetChild() const
{
	return mChild;
}

void cJoint::ClampTotalTorque(tVector& out_torque) const
{
	double mag = out_torque.norm();
	double torque_lim = GetTorqueLimit();
	if (mag > torque_lim)
	{
		out_torque *= torque_lim / mag;
	}
}

void cJoint::ClampTotalForce(tVector& out_force) const
{
	double mag = out_force.norm();
	double force_lim = GetForceLimit();
	if (mag > force_lim)
	{
		out_force *= force_lim / mag;
	}
}

const cWorld::tConstraintHandle& cJoint::GetConstraintHandle() const
{
	return mCons;
}

double cJoint::GetRefTheta() const
{
	return mParams.mRefTheta;
}

const tVector& cJoint::GetLimLow() const
{
	return mParams.mLimLow;
}

const tVector& cJoint::GetLimHigh() const
{
	return mParams.mLimHigh;
}

bool cJoint::HasJointLim() const
{
	return (mParams.mLimLow[0] <= mParams.mLimLow[0]
			|| mParams.mLimLow[1] <= mParams.mLimLow[1]
			|| mParams.mLimLow[2] <= mParams.mLimLow[2]
			|| mParams.mLimLow[3] <= mParams.mLimLow[3]);
}

int cJoint::GetParamSize() const
{
	return cKinTree::GetJointParamSize(mParams.mType);
}

void cJoint::BuildPose(Eigen::VectorXd& out_pose) const
{
	if (IsValid())
	{
		switch (mParams.mType)
		{
		case cKinTree::eJointTypeRevolute:
			BuildPoseRevolute(out_pose);
			break;
		case cKinTree::eJointTypePlanar:
			BuildPosePlanar(out_pose);
			break;
		case cKinTree::eJointTypePrismatic:
			BuildPosePristmatic(out_pose);
			break;
		case cKinTree::eJointTypeFixed:
			BuildPoseFixed(out_pose);
			break;
		case cKinTree::eJointTypeSpherical:
			BuildPoseSpherical(out_pose);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}
	else
	{
		int param_size = GetParamSize();
		out_pose = Eigen::VectorXd::Zero(param_size);
	}
}

void cJoint::BuildVel(Eigen::VectorXd& out_vel) const
{
	if (IsValid())
	{
		switch (mParams.mType)
		{
		case cKinTree::eJointTypeRevolute:
			BuildVelRevolute(out_vel);
			break;
		case cKinTree::eJointTypePlanar:
			BuildVelPlanar(out_vel);
			break;
		case cKinTree::eJointTypePrismatic:
			BuildVelPristmatic(out_vel);
			break;
		case cKinTree::eJointTypeFixed:
			BuildVelFixed(out_vel);
			break;
		case cKinTree::eJointTypeSpherical:
			BuildVelSpherical(out_vel);
			break;
		default:
			assert(false); // unsupported joint type
			break;
		}
	}
	else
	{
		int param_size = GetParamSize();
		out_vel = Eigen::VectorXd::Zero(param_size);
	}
}

tVector cJoint::GetTotalTorque() const
{
	tVector torque = cSpAlg::GetOmega(mTotalTau);
	return torque;
}

void cJoint::SetTotalTorque(const tVector& torque)
{
	cSpAlg::SetOmega(torque, mTotalTau);
}

tVector cJoint::GetTotalForce() const
{
	tVector force = cSpAlg::GetV(mTotalTau);
	return force;
}

void cJoint::SetEnable(bool enable)
{
	mCons.mCons->setEnabled(enable);
}

void cJoint::SetTotalForce(const tVector& force)
{
	cSpAlg::SetV(force, mTotalTau);
}

void cJoint::BuildPoseRevolute(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	double theta;
	tVector axis;
	CalcRotationRevolute(axis, theta);

	out_pose[0] = theta;
}

void cJoint::BuildPosePlanar(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	assert(false); // unsupported
}

void cJoint::BuildPosePristmatic(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	double delta = CalcDisplacementPrismatic();
	out_pose[0] = delta;
}

void cJoint::BuildPoseFixed(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose = Eigen::VectorXd::Zero(param_size);
}

void cJoint::BuildPoseSpherical(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	btGeneric6DofSpring2Constraint* sphere = reinterpret_cast<btGeneric6DofSpring2Constraint*>(mCons.mCons);
	tVector euler = tVector(-sphere->getAngle(0), -sphere->getAngle(1), -sphere->getAngle(2), 0);
	tQuaternion q = cMathUtil::EulerToQuaternion(euler);

	out_pose.resize(param_size);
	out_pose[0] = q.w();
	out_pose[1] = q.x();
	out_pose[2] = q.y();
	out_pose[3] = q.z();
}


void cJoint::BuildVelRevolute(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	tVector vel = CalcRotationVelRevolute();
	out_pose[0] = vel[2];
}

void cJoint::BuildVelPlanar(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	assert(false); // unsupported
}

void cJoint::BuildVelPristmatic(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);

	double delta = CalcDisplacementVelPrismatic();
	out_pose[0] = delta;
}

void cJoint::BuildVelFixed(Eigen::VectorXd& out_pose) const
{
	int param_size = GetParamSize();
	out_pose.resize(param_size);
}

void cJoint::BuildVelSpherical(Eigen::VectorXd& out_vel) const
{
	int param_size = GetParamSize();
	out_vel.resize(param_size);
	tVector vel = CalcRotationVelSpherical();
	out_vel.segment(0, param_size) = vel.segment(0, param_size);
}

void cJoint::AddTauRevolute(const Eigen::VectorXd& tau)
{
	mTotalTau[2] += tau[0];
}

void cJoint::AddTauPlanar(const Eigen::VectorXd& tau)
{
	mTotalTau[3] += tau[0];
	mTotalTau[4] += tau[1];
	mTotalTau[0] += tau[2];
}

void cJoint::AddTauPristmatic(const Eigen::VectorXd& tau)
{
	mTotalTau[3] += tau[0];
}

void cJoint::AddTauFixed(const Eigen::VectorXd& tau)
{
}

void cJoint::AddTauSpherical(const Eigen::VectorXd& tau)
{
	mTotalTau[0] += tau[0];
	mTotalTau[1] += tau[1];
	mTotalTau[2] += tau[2];
}


void cJoint::CalcRotationRevolute(tVector& out_axis, double& out_theta) const
{
	assert(mParams.mType == cKinTree::eJointTypeRevolute);
	mWorld->CalcRotationRevolute(this, out_axis, out_theta);
}

tVector cJoint::CalcRotationVelRevolute() const
{
	tVector ang_velp = tVector::Zero();
	if (HasParent())
	{
		ang_velp = mParent->GetAngularVelocity();
	}

	tVector ang_velc = mChild->GetAngularVelocity();

	tVector joint_vel = ang_velc - ang_velp;
	joint_vel[3] = 0;

	tMatrix trans = BuildWorldTrans();
	joint_vel = trans.transpose() * joint_vel;
	joint_vel[3] = 0;

	return joint_vel;
}


double cJoint::CalcDisplacementPrismatic() const
{
	assert(mParams.mType == cKinTree::eJointTypePrismatic);
	return mWorld->CalcDisplacementPrismatic(this);
}

double cJoint::CalcDisplacementVelPrismatic() const
{
	assert(mParams.mType == cKinTree::eJointTypePrismatic);
	tVector velp = tVector::Zero();
	if (HasParent())
	{
		velp = mParent->GetLinearVelocity();
	}

	tVector velc = mChild->GetLinearVelocity();

	tVector delta_vel = velc - velp;
	delta_vel[3] = 0;
	double direction = velc.dot(delta_vel);
	tMatrix trans = BuildWorldTrans();
	delta_vel = trans.transpose() * delta_vel;
	delta_vel[3] = 0;
	double joint_vel = delta_vel.stableNorm();
	if (direction < 0)
	{ // more robust than * (direction/direction)
		joint_vel *= -1.0;
	}

	return joint_vel;
}

void cJoint::CalcRotationSpherical(tVector& out_axis, double& out_theta) const
{
	assert(mParams.mType == cKinTree::eJointTypeSpherical);
	mWorld->CalcRotationSpherical(this, out_axis, out_theta);
}

tVector cJoint::CalcRotationVelSpherical() const
{
	assert(mParams.mType == cKinTree::eJointTypeSpherical);
	return CalcRotationVelRevolute();
}
