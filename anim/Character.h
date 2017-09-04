#pragma once

#include "KinTree.h"

class cCharacter
{
public:
	static const std::string gSkeletonKey;

	virtual ~cCharacter();

	virtual bool Init(const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual int GetNumDof() const;
	virtual const Eigen::MatrixXd& GetJointMat() const;
	virtual int GetNumJoints() const;

	virtual const Eigen::VectorXd& GetPose() const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual const Eigen::VectorXd& GetPose0() const;
	virtual void SetPose0(const Eigen::VectorXd& pose);

	virtual const Eigen::VectorXd& GetVel() const;
	virtual void SetVel(const Eigen::VectorXd& vel);
	virtual const Eigen::VectorXd& GetVel0() const;
	virtual void SetVel0(const Eigen::VectorXd& vel);

	virtual int GetRootID() const;
	virtual tVector GetRootPos() const;
	virtual void GetRootRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRootRotation() const;
	virtual void SetRootPos(const tVector& pos);
	virtual void SetRootPos0(const tVector& pose);
	virtual void SetRootRotation(const tQuaternion& q);

	virtual tQuaternion CalcHeadingRot() const;

	virtual double CalcHeading() const;
	virtual tMatrix BuildOriginTrans() const;

	virtual int GetParamOffset(int joint_id) const;
	virtual int GetParamSize(int joint_id) const;
	virtual bool IsEndEffector(int joint_id) const;
	virtual int GetParentJoint(int joint_id) const;

	virtual tVector CalcJointPos(int joint_id) const;
	virtual tVector CalcJointVel(int joint_id) const;
	virtual void CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const;
	virtual tQuaternion CalcJointWorldRotation(int joint_id) const;
	virtual double CalcJointChainLength(int joint_id);
	virtual tMatrix BuildJointWorldTrans(int joint_id) const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;
	virtual int CalcNumEndEffectors() const;

	// weights for each joint used to compute the pose error during training
	virtual double GetJointDiffWeight(int joint_id) const;

	virtual bool WriteState(const std::string& file) const;
	virtual bool WriteState(const std::string& file, const tMatrix& root_trans) const;
	virtual bool ReadState(const std::string& file);

	virtual bool LoadSkeleton(const Json::Value& root);
	virtual void InitDefaultState();

protected:
	Eigen::MatrixXd mJointMat;
	Eigen::VectorXd mPose;
	Eigen::VectorXd mVel;
	Eigen::VectorXd mPose0;
	Eigen::VectorXd mVel0;

	cCharacter();

	virtual void ResetParams();
	virtual bool ParseState(const Json::Value& root, Eigen::VectorXd& out_state) const;
	virtual std::string BuildStateJson(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel) const;
};