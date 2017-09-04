/*
 * SimBiped.h
 *
 *  Created on: 2016-06-28
 *      Author: gberseth
 */


#ifndef SIMBIPED_H_
#define SIMBIPED_H_
#include "sim/SimCharSoftFall.h"

class cSimBiped : public cSimCharSoftFall
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
		eJointRightHip,
		eJointRightKnee,
		eJointRightAnkle,
		eJointLeftHip,
		eJointLeftKnee,
		eJointLeftAnkle,
		eJointMax,
		eJointInvalid
	};

	cSimBiped();
	virtual ~cSimBiped();

	virtual bool HasStumbled() const;

protected:

	virtual bool FailFallMisc() const;
};

#endif /* SIMBIPED_H_ */
