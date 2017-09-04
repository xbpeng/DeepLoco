/*
* SimBiped3D.h
*
*  Created on: Oct 1, 2016
*      Author: Glen
*/

#ifndef SIM_SIMBIPED3D_H_
#define SIM_SIMBIPED3D_H_

#include "sim/SimCharSoftFall.h"

class cSimBiped3D : public cSimCharSoftFall
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

	cSimBiped3D();
	virtual ~cSimBiped3D();

	virtual bool HasStumbled() const;

protected:

	virtual bool FailFallMisc() const;

};

#endif /* SIM_SIMBIPED3D_H_ */
