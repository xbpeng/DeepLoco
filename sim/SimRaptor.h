#pragma once

#include "sim/SimCharSoftFall.h"


class cSimRaptor : public cSimCharSoftFall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointSpine0,
		eJointSpine1,
		eJointSpine2,
		eJointSpine3,
		eJointHead,
		eJointTail0,
		eJointTail1,
		eJointTail2,
		eJointTail3,
		eJointTail4,
		eJointRightHip,
		eJointRightKnee,
		eJointRightAnkle,
		eJointRightToe,
		eJointLeftHip,
		eJointLeftKnee,
		eJointLeftAnkle,
		eJointLeftToe,
		eJointMax,
		eJointInvalid
	};

	cSimRaptor();
	virtual ~cSimRaptor();

	virtual bool HasStumbled() const;

protected:
	
	virtual bool FailFallMisc() const;
};