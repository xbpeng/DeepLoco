/*
* SimBiped2D.cpp
*
*  Created on: Sep 1, 2016
*      Author: Glen
*/

#include "SimBiped3D.h"

#include <iostream>

/*
const double gPoseJointWeights[cSimBiped3D::eJointMax] =
{
1, // eJointRoot,
0.5, // eJointSpine0,
1, // eJointRightHip,
0.5, // eJointRightKnee,
0.5, // eJointRightAnkle,
1, // eJointLeftHip,
0.5, // eJointLeftKnee,
0.5, // eJointLeftAnkle,
};
*/

const double gPoseJointWeights[cSimBiped3D::eJointMax] =
{
	1, // eJointRoot,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
};

cSimBiped3D::cSimBiped3D()
{

}

cSimBiped3D::~cSimBiped3D()
{

}

bool cSimBiped3D::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimBiped3D::eJointMax; ++i)
	{
		cSimBiped3D::eJoint joint_id = static_cast<cSimBiped3D::eJoint>(i);

		if (joint_id != cSimBiped3D::eJointRightAnkle
			&& joint_id != cSimBiped3D::eJointLeftAnkle
			&& joint_id != cSimBiped3D::eJointLeftKnee
			&& joint_id != cSimBiped3D::eJointRightKnee)
		{
			const auto& curr_part = GetBodyPart(joint_id);
			bool contact = curr_part->IsInContact();
			if (contact)
			{
				stumbled = true;
				break;
			}
		}
	}
	return stumbled;
}

bool cSimBiped3D::FailFallMisc() const
{
	bool fallen = false;

	const tVector ref_axis = tVector(0, 0, 1, 0);
	tVector root_axis;
	double root_theta;
	GetRootRotation(root_axis, root_theta);

	if (root_axis.dot(ref_axis) < 0)
	{
		root_theta = -root_theta;
	}

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}


