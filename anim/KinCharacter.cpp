#include "KinCharacter.h"
#include <assert.h>
#include <functional>

const double gDiffTimeStep = 1 / 60.0;

cKinCharacter::cKinCharacter()
{
	mOrigin.setZero();
	mOriginRot.setIdentity();
	mCycleRootDelta.setZero();
	mEnableVelUpdate = true;
}

cKinCharacter::~cKinCharacter()
{

}

bool cKinCharacter::Init(const std::string& char_file, const std::string& motion_file)
{
	bool succ = Init(char_file);
	if (succ && (motion_file != ""))
	{
		bool succ_motion = LoadMotion(motion_file);
		if (!succ_motion)
		{
			printf("Failed to load motion from %s\n", motion_file.c_str());
		}
		succ &= succ_motion;
	}

	return succ;
}

bool cKinCharacter::Init(const std::string& char_file)
{
	return cCharacter::Init(char_file);
}

void cKinCharacter::Clear()
{
	cCharacter::Clear();
	mMotion.Clear();

	if (HasController())
	{
		mController->Clear();
	}
}

void cKinCharacter::Update(double time_step)
{
	cCharacter::Update(time_step);
	mTime += time_step;

	if (HasController())
	{
		mController->Update(time_step);
	}
	
	Pose(mTime);
}

void cKinCharacter::Reset()
{
	cCharacter::Reset();
	if (HasController())
	{
		mController->Reset();
	}
}

const cMotion& cKinCharacter::GetMotion() const
{
	return mMotion;
}

double cKinCharacter::GetMotionDuration() const
{
	if (mMotion.IsValid())
	{
		return mMotion.GetDuration();
	}
	return 0;
}

void cKinCharacter::SetTime(double time)
{
	mTime = time;

	if (HasController())
	{
		mController->SetTime(time);
	}
}

double cKinCharacter::GetTime() const
{
	return mTime;
}

double cKinCharacter::GetPhase() const
{
	double phase = mTime / mMotion.GetDuration();
	phase -= static_cast<int>(phase);
	phase = (phase < 0) ? (1 + phase) : phase;
	return phase;
}

void cKinCharacter::Pose(double time)
{
	Eigen::VectorXd pose;
	CalcPose(time, pose);
	SetPose(pose);

	if (mEnableVelUpdate)
	{
		Eigen::VectorXd pose1;
		CalcPose(time + gDiffTimeStep, pose1);
		cKinTree::CalcVel(mJointMat, pose, pose1, gDiffTimeStep, mVel);
	}
}

void cKinCharacter::BuildAcc(Eigen::VectorXd& out_acc) const
{
	CalcAcc(mTime, out_acc);
}

bool cKinCharacter::HasMotion() const
{
	return mMotion.IsValid();
}

const tVector& cKinCharacter::GetOriginPos() const
{
	return mOrigin;
}

void cKinCharacter::SetOriginPos(const tVector& origin)
{
	tVector delta = origin - mOrigin;
	MoveOrigin(delta);
	mOrigin = origin; // this is needed in canse of NaNs
}

void cKinCharacter::MoveOrigin(const tVector& delta)
{
	mOrigin += delta;

	tVector root0 = cKinTree::GetRootPos(mJointMat, mPose0);
	root0 += delta;
	cKinTree::SetRootPos(mJointMat, root0, mPose0);

	tVector root = cKinTree::GetRootPos(mJointMat, mPose);
	root += delta;
	cKinTree::SetRootPos(mJointMat, root, mPose);
}

const tQuaternion& cKinCharacter::GetOriginRot() const
{
	return mOriginRot;
}

void cKinCharacter::SetOriginRot(const tQuaternion& rot)
{
	tQuaternion delta_rot = cMathUtil::QuatDiff(mOriginRot, rot);
	RotateOrigin(delta_rot);
	mOriginRot = rot; // this is needed in canse of NaNs
}

void cKinCharacter::RotateOrigin(const tQuaternion& rot)
{
	mOriginRot = rot * mOriginRot;

	tQuaternion root0 = cKinTree::GetRootRot(mJointMat, mPose0);
	root0 = rot * root0;
	cKinTree::SetRootRot(mJointMat, root0, mPose0);

	tQuaternion root = cKinTree::GetRootRot(mJointMat, mPose);
	root = rot * root;
	cKinTree::SetRootRot(mJointMat, root, mPose);

	tVector vel0 = cKinTree::GetRootVel(mJointMat, mVel0);
	vel0 = cMathUtil::QuatRotVec(rot, vel0);
	cKinTree::SetRootVel(mJointMat, vel0, mVel0);

	tVector vel = cKinTree::GetRootVel(mJointMat, mVel);
	vel = cMathUtil::QuatRotVec(rot, vel);
	cKinTree::SetRootVel(mJointMat, vel, mVel);

	tVector ang_vel0 = cKinTree::GetRootAngVel(mJointMat, mVel0);
	ang_vel0 = cMathUtil::QuatRotVec(rot, ang_vel0);
	cKinTree::SetRootVel(mJointMat, ang_vel0, mVel0);

	tVector ang_vel = cKinTree::GetRootAngVel(mJointMat, mVel);
	ang_vel = cMathUtil::QuatRotVec(rot, ang_vel);
	cKinTree::SetRootVel(mJointMat, ang_vel, mVel);
}

void cKinCharacter::ResetParams()
{ 
	cCharacter::ResetParams();
	mTime = 0;
}

bool cKinCharacter::LoadMotion(const std::string& motion_file)
{
	bool succ = mMotion.Load(motion_file);

	if (succ)
	{
		int char_dof = GetNumDof();
		int motion_dof = mMotion.GetNumDof();

		if (char_dof != motion_dof)
		{
			printf("DOF mismatch, char dof: %i, motion dof: %i\n", char_dof, motion_dof);
			mMotion.Clear();
			succ = false;
		}
	}

	if (succ)
	{
		cMotion::tBlendFunc blend_func = std::bind(&cKinCharacter::BlendFrames, this, 
					std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
		mMotion.SetBlendFunc(blend_func);

		mCycleRootDelta = CalcCycleRootDelta();
		Pose(mTime);
		mPose0 = GetPose();
		mVel0 = GetVel();
	}

	return succ;
}

void cKinCharacter::SetController(const std::shared_ptr<cKinController>& ctrl)
{
	RemoveController();
	mController = ctrl;

	Pose(mTime);
	mPose0 = GetPose();
	mVel0 = GetVel();
}

const std::shared_ptr<cKinController>& cKinCharacter::GetController() const
{
	return mController;
}

void cKinCharacter::RemoveController()
{
	if (HasController())
	{
		mController.reset();
	}
}

bool cKinCharacter::HasController() const
{
	return mController != nullptr;
}

tVector cKinCharacter::CalcCycleRootDelta() const
{
	int num_frames = mMotion.GetNumFrames();
	Eigen::VectorXd frame_beg = mMotion.GetFrame(0);
	Eigen::VectorXd  frame_end = mMotion.GetFrame(num_frames - 1);

	tVector root_pos_beg = cKinTree::GetRootPos(mJointMat, frame_beg);
	tVector root_pos_end = cKinTree::GetRootPos(mJointMat, frame_end);

	tVector delta = root_pos_end - root_pos_beg;
	return delta;
}

void cKinCharacter::CalcPose(double time, Eigen::VectorXd& out_pose) const
{
	tVector root_delta = tVector::Zero();
	tQuaternion root_delta_rot = tQuaternion::Identity();

	if (HasController())
	{
		mController->CalcPose(time, out_pose);
	}
	else if (HasMotion())
	{
		out_pose = mMotion.CalcFrame(time);
		if (mMotion.IsLoop())
		{
			int cycle_count = mMotion.CalcCycleCount(time);
			root_delta = cycle_count * mCycleRootDelta;
		}
	}
	else
	{
		out_pose = mPose0;
	}

	tVector root_pos = cKinTree::GetRootPos(mJointMat, out_pose);
	tQuaternion root_rot = cKinTree::GetRootRot(mJointMat, out_pose);

	root_delta_rot = mOriginRot * root_delta_rot;
	root_rot = root_delta_rot * root_rot;
	root_pos = cMathUtil::QuatRotVec(root_delta_rot, root_pos);

	root_delta += mOrigin;
	root_pos += root_delta;

	cKinTree::SetRootPos(mJointMat, root_pos, out_pose);
	cKinTree::SetRootRot(mJointMat, root_rot, out_pose);
}

void cKinCharacter::CalcVel(double time, Eigen::VectorXd& out_vel) const
{
	// approximate velocity with finite difference
	Eigen::VectorXd pose0;
	Eigen::VectorXd pose1;
	CalcPose(time, pose0);
	CalcPose(time + gDiffTimeStep, pose1);
	cKinTree::CalcVel(mJointMat, pose0, pose1, gDiffTimeStep, out_vel);
}

void cKinCharacter::CalcAcc(double time, Eigen::VectorXd& out_acc) const
{
	Eigen::VectorXd vel0;
	Eigen::VectorXd vel1;
	CalcVel(time - gDiffTimeStep, vel0);
	CalcVel(time, vel1);
	out_acc = (vel1 - vel0) / gDiffTimeStep;
}

void cKinCharacter::EnableVelUpdate(bool enable)
{
	mEnableVelUpdate = enable;
}

bool cKinCharacter::IsMotionOver() const
{
	bool over = true;
	if (HasController())
	{
		over = mController->IsMotionOver();
	}
	else if (HasMotion())
	{
		over = mMotion.IsOver(mTime);
	}
	return over;
}

cMotion::tFrame cKinCharacter::BlendFrames(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp) const
{
	cMotion::tFrame blend_frame;
	cKinTree::LerpPoses(mJointMat, *a, *b, lerp, blend_frame);
	return blend_frame;
}