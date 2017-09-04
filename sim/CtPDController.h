#pragma once

#include "sim/CtController.h"
#include "sim/ImpPDController.h"
#include "sim/ExpPDController.h"

//#define ENABLE_EXP_PD_CTRL

class cCtPDController : public virtual cCtController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPDController();
	virtual ~cCtPDController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

protected:
#if defined(ENABLE_EXP_PD_CTRL)
	cExpPDController mPDCtrl;
#else
	cImpPDController mPDCtrl;
#endif

	virtual void UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau);
	virtual void SetupPDControllers(const std::string& param_file, const tVector& gravity);
	virtual void UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau);
	virtual void ApplyAction(const tAction& action);
	virtual void BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void ConvertActionToTargetPose(int joint_id, Eigen::VectorXd& out_theta) const;
	virtual cKinTree::eJointType GetJointType(int joint_id) const;
};