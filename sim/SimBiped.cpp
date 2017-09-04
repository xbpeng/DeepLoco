/*
 * SimBiped.cpp
 *
 *  Created on: 2016-06-28
 *      Author: gberseth
 */

#include "sim/SimBiped.h"

#include <iostream>

/*
const double gPoseJointWeights[cSimBiped::eJointMax] =
{
	1, // eJointRoot,
	0.5, // eJointSpine0,
	0.5, // eJointSpine1,
	0.5, // eJointSpine2,
	0.5, // eJointSpine3,
	0.5, // eJointHead,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
};
*/

const double gPoseJointWeights[cSimBiped::eJointMax] =
{
	1, // eJointRoot,
	0.5, // eJointSpine0,
	0.5, // eJointSpine1,
	0.5, // eJointSpine2,
	0.5, // eJointSpine3,
	0.5, // eJointHead,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
};

cSimBiped::cSimBiped()
{

}

cSimBiped::~cSimBiped()
{

}

bool cSimBiped::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimBiped::eJointMax; ++i)
	{
		cSimBiped::eJoint joint_id = static_cast<cSimBiped::eJoint>(i);

		if (joint_id != cSimBiped::eJointRightAnkle
			&& joint_id != cSimBiped::eJointLeftAnkle)
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

bool cSimBiped::FailFallMisc() const
{
	bool fallen = false;
	tQuaternion root_q = GetRootRotation();
	double root_theta = cMathUtil::QuatTheta(root_q);

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}
