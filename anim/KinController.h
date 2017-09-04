#pragma once

#include <memory>
#include "util/MathUtil.h"

class cKinCharacter;

class cKinController : public std::enable_shared_from_this<cKinController>
{
public:

	virtual ~cKinController();

	virtual void Init(cKinCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;

	virtual void SetTime(double time);
	virtual double GetTime() const;

	virtual bool IsMotionOver() const;

protected:
	cKinCharacter* mChar;
	double mTime;
	
	cKinController();
};