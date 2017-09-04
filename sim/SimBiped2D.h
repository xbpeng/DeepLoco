/*
 * SimBiped2D.h
 *
 *  Created on: Sep 1, 2016
 *      Author: Glen
 */

#ifndef SIM_SIMBIPED2D_H_
#define SIM_SIMBIPED2D_H_

#include "sim/SimCharSoftFall.h"

class cSimBiped2D : public cSimCharSoftFall
{
	public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointRightHip,
		eJointRightKnee,
		eJointRightAnkle,
		eJointLeftHip,
		eJointLeftKnee,
		eJointLeftAnkle,
		eJointMax,
		eJointInvalid
	};

	cSimBiped2D();
	virtual ~cSimBiped2D();

	virtual bool HasStumbled() const;

protected:
	
	virtual bool FailFallMisc() const;

};

#endif /* SIM_SIMBIPED2D_H_ */
