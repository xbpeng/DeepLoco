#pragma once

#include "anim/Character.h"
#include "anim/Motion.h"
#include "anim/KinController.h"

class cKinCharacter : public cCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cKinCharacter();
	virtual ~cKinCharacter();

	virtual bool Init(const std::string& char_file, const std::string& motion_file);
	virtual bool Init(const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual const cMotion& GetMotion() const;
	virtual double GetMotionDuration() const;
	virtual void SetTime(double time);
	virtual double GetTime() const;
	virtual double GetPhase() const;

	virtual void Pose(double time);
	virtual void BuildAcc(Eigen::VectorXd& out_acc) const;
	
	virtual bool HasMotion() const;

	virtual const tVector& GetOriginPos() const;
	virtual void SetOriginPos(const tVector& origin);
	virtual void MoveOrigin(const tVector& delta);
	virtual const tQuaternion& GetOriginRot() const;
	virtual void SetOriginRot(const tQuaternion& rot);
	virtual void RotateOrigin(const tQuaternion& rot);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;
	virtual void CalcVel(double time, Eigen::VectorXd& out_vel) const;
	virtual void CalcAcc(double time, Eigen::VectorXd& out_vel) const;
	virtual void EnableVelUpdate(bool enable);

	virtual bool IsMotionOver() const;
	cMotion::tFrame BlendFrames(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp) const;

	virtual bool LoadMotion(const std::string& motion_file);
	virtual void SetController(const std::shared_ptr<cKinController>& ctrl);
	virtual const std::shared_ptr<cKinController>& GetController() const;
	virtual void RemoveController();
	virtual bool HasController() const;

protected:
	double mTime;
	bool mEnableVelUpdate;
	cMotion mMotion;
	std::shared_ptr<cKinController> mController;

	tVector mCycleRootDelta;
	tVector mOrigin;
	tQuaternion mOriginRot;

	virtual void ResetParams();
	virtual tVector CalcCycleRootDelta() const;
};