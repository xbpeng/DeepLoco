/*
 * SimBiped2D.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Glen
 */

#include "SimBiped2D.h"

#include <iostream>

/*
const double gPoseJointWeights[cSimBiped2D::eJointMax] =
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

const double gPoseJointWeights[cSimBiped2D::eJointMax] =
{
	1, // eJointRoot,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
};

cSimBiped2D::cSimBiped2D()
{

}

cSimBiped2D::~cSimBiped2D()
{

}

bool cSimBiped2D::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimBiped2D::eJointMax; ++i)
	{
		cSimBiped2D::eJoint joint_id = static_cast<cSimBiped2D::eJoint>(i);

		if (joint_id != cSimBiped2D::eJointRightAnkle
			&& joint_id != cSimBiped2D::eJointLeftAnkle
			&& joint_id != cSimBiped2D::eJointLeftKnee
			&& joint_id != cSimBiped2D::eJointRightKnee)
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

bool cSimBiped2D::FailFallMisc() const
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

	if (this->GetRootVel()[0] < 0.0)
	{ // Moving backwards
		fallen |= true;
	}

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}


