#include "CtPDController.h"
#include "sim/SimCharacter.h"

cCtPDController::cCtPDController() : cCtController()
{
}

cCtPDController::~cCtPDController()
{
}

void cCtPDController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtController::Init(character);
	SetupPDControllers(param_file, gravity);
}

void cCtPDController::Reset()
{
	cCtController::Reset();
	mPDCtrl.Reset();
}

void cCtPDController::Clear()
{
	cCtController::Clear();
	mPDCtrl.Clear();
}

void cCtPDController::SetupPDControllers(const std::string& param_file, const tVector& gravity)
{
	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);
	if (succ)
	{
#if defined(ENABLE_EXP_PD_CTRL)
		mPDCtrl.Init(mChar, pd_params);
#else
		mPDCtrl.Init(mChar, pd_params, gravity);
#endif
	}

	mValid = succ;
	if (!mValid)
	{
		printf("Failed to initialize Ct-PD controller\n");
		mValid = false;
	}
}

void cCtPDController::UpdateBuildTau(double time_step, Eigen::VectorXd& out_tau)
{
	UpdatePDCtrls(time_step, out_tau);
}

void cCtPDController::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	int num_dof = mChar->GetNumDof();
	out_tau = Eigen::VectorXd::Zero(num_dof);
	mPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cCtPDController::ApplyAction(const tAction& action)
{
	cCtController::ApplyAction(action);
	
	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(root_id);
	int num_joints = mChar->GetNumJoints();
	for (int j = root_id + 1; j < num_joints; ++j)
	{
		if (mPDCtrl.IsValidPDCtrl(j))
		{
			int retarget_joint = RetargetJointID(j);
			int param_offset = mChar->GetParamOffset(retarget_joint);
			int param_size = mChar->GetParamSize(retarget_joint);
			param_offset -= root_size;

			Eigen::VectorXd theta = mCurrAction.mParams.segment(param_offset, param_size);
			ConvertActionToTargetPose(j, theta);
			mPDCtrl.SetTargetTheta(j, theta);
		}
	}
}

void cCtPDController::BuildJointActionBounds(int joint_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildBoundsPD(joint_mat, joint_id, out_min, out_max);
}

void cCtPDController::BuildJointActionOffsetScale(int joint_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	cCtCtrlUtil::BuildOffsetScalePD(joint_mat, joint_id, out_offset, out_scale);
}

void cCtPDController::ConvertActionToTargetPose(int joint_id, Eigen::VectorXd& out_theta) const
{
#if defined(ENABLE_PD_SPHERE_AXIS)
	cKinTree::eJointType joint_type = GetJointType(joint_id);
	if (joint_type == cKinTree::eJointTypeSpherical)
	{
		double rot_theta = out_theta[0];
		tVector axis = tVector(out_theta[1], out_theta[2], out_theta[3], 0);
		if (axis.squaredNorm() == 0)
		{
			axis[2] = 1;
		}

		axis.normalize();
		tQuaternion quat = cMathUtil::AxisAngleToQuaternion(axis, rot_theta);

		if (FlipStance())
		{
			cKinTree::eJointType joint_type = GetJointType(joint_id);
			if (joint_type == cKinTree::eJointTypeSpherical)
			{
				quat = cMathUtil::MirrorQuaternion(quat, cMathUtil::eAxisZ);
			}
		}
		out_theta = cMathUtil::QuatToVec(quat);
	}
#endif
}

cKinTree::eJointType cCtPDController::GetJointType(int joint_id) const
{
	const cPDController& ctrl = mPDCtrl.GetPDCtrl(joint_id);
	const cJoint& joint = ctrl.GetJoint();
	cKinTree::eJointType joint_type = joint.GetType();
	return joint_type;
}