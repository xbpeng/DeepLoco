#pragma once

#include "SimObj.h"
#include "SpAlg.h"

// for now joints are assumed to be hinge joints fixed along the z axis
class cJoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cJoint();
	virtual ~cJoint();

	virtual void Init(std::shared_ptr<cWorld>& world, std::shared_ptr<cSimObj> parent, std::shared_ptr<cSimObj> child,
						const cWorld::tJointParams& params);
	virtual void Clear();
	virtual bool IsValid() const;

	virtual tVector CalcAxisWorld() const;
	virtual tVector GetAxisRel() const;
	virtual bool HasParent() const;
	virtual cKinTree::eJointType GetType() const;

	virtual void CalcRotation(tVector& out_axis, double& out_theta) const;
	virtual void CalcWorldRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcWorldRotation() const;
	virtual void GetChildRotation(tVector& out_axis, double& out_theta) const;
	virtual void GetParentRotation(tVector& out_axis, double& out_theta) const;
	virtual tMatrix BuildWorldTrans() const;

	virtual void AddTau(const Eigen::VectorXd& tau);
	virtual const cSpAlg::tSpVec& GetTau() const;
	virtual void ApplyTau();
	virtual void ClearTau();

	virtual tVector GetPos() const;
	virtual tVector CalcWorldPos(const tVector& local_pos) const;
	virtual tVector GetWorldVel() const;
	virtual tVector CalcWorldVel(const tVector& local_pos) const;
	virtual double GetTorqueLimit() const;
	virtual double GetForceLimit() const;
	virtual void SetTorqueLimit(double lim);
	virtual void SetForceLimit(double lim);

	virtual const tVector& GetParentAnchor() const;
	virtual const tVector& GetChildAnchor() const;
	virtual const std::shared_ptr<cSimObj>& GetParent() const;
	virtual const std::shared_ptr<cSimObj>& GetChild() const;
	
	// arg these methods should not be public
	virtual void CalcRotationRevolute(tVector& out_axis, double& out_theta) const;
	virtual tVector CalcRotationVelRevolute() const;
	virtual double CalcDisplacementPrismatic() const;
	virtual double CalcDisplacementVelPrismatic() const;
	virtual void CalcRotationSpherical(tVector& out_axis, double& out_theta) const;
	virtual tVector CalcRotationVelSpherical() const;

	virtual void ClampTotalTorque(tVector& out_torque) const;
	virtual void ClampTotalForce(tVector& out_force) const;

	virtual const cWorld::tConstraintHandle& GetConstraintHandle() const;
	virtual double GetRefTheta() const;
	virtual const tVector& GetLimLow() const;
	virtual const tVector& GetLimHigh() const;
	virtual bool HasJointLim() const;

	virtual int GetParamSize() const;
	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel(Eigen::VectorXd& out_vel) const;

	virtual tVector GetTotalTorque() const;
	virtual tVector GetTotalForce() const;
	virtual void SetEnable(bool enable);

protected:
	cWorld::tJointParams mParams;
	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cSimObj> mParent;
	std::shared_ptr<cSimObj> mChild;
	cWorld::tConstraintHandle mCons;

	// all torques and forces are in local coordinates
	cSpAlg::tSpVec mTotalTau;

	virtual void SetTotalTorque(const tVector& torque);
	virtual void SetTotalForce(const tVector& force);
	virtual void ApplyTorque();
	virtual void ApplyForce();

	virtual void BuildPoseRevolute(Eigen::VectorXd& out_pose) const;
	virtual void BuildPosePlanar(Eigen::VectorXd& out_pose) const;
	virtual void BuildPosePristmatic(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoseFixed(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoseSpherical(Eigen::VectorXd& out_pose) const;

	virtual void BuildVelRevolute(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelPlanar(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelPristmatic(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelFixed(Eigen::VectorXd& out_pose) const;
	virtual void BuildVelSpherical(Eigen::VectorXd& out_vel) const;

	virtual void AddTauRevolute(const Eigen::VectorXd& tau);
	virtual void AddTauPlanar(const Eigen::VectorXd& tau);
	virtual void AddTauPristmatic(const Eigen::VectorXd& tau);
	virtual void AddTauFixed(const Eigen::VectorXd& tau);
	virtual void AddTauSpherical(const Eigen::VectorXd& tau);
};
