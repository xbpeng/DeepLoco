#include "sim/SimRaptor.h"
#include <iostream>

/*
const double gPoseJointWeights[cSimRaptor::eJointMax] =
{
	1, // eJointRoot,
	0.5, // eJointSpine0,
	0.5, // eJointSpine1,
	0.5, // eJointSpine2,
	0.5, // eJointSpine3,
	0.5, // eJointHead,
	0.01, // eJointTail0,
	0.01, // eJointTail1,
	0.01, // eJointTail2,
	0.01, // eJointTail3,
	0.01, // eJointTail4,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	0.2, // eJointRightToe,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
	0.2, // eJointLeftToe,
};
*/

const double gPoseJointWeights[cSimRaptor::eJointMax] =
{
	1, // eJointRoot,
	0.5, // eJointSpine0,
	0.5, // eJointSpine1,
	0.5, // eJointSpine2,
	0.5, // eJointSpine3,
	0.5, // eJointHead,
	0.25, // eJointTail0,
	0.25, // eJointTail1,
	0.25, // eJointTail2,
	0.25, // eJointTail3,
	0.25, // eJointTail4,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	0.5, // eJointRightToe,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
	0.5, // eJointLeftToe,
};

cSimRaptor::cSimRaptor()
{
}

cSimRaptor::~cSimRaptor()
{
}

bool cSimRaptor::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimRaptor::eJointMax; ++i)
	{
		eJoint joint_id = static_cast<eJoint>(i);

		if (joint_id != eJointRightToe 
			&& joint_id != eJointLeftToe
			&& joint_id != eJointRightAnkle
			&& joint_id != eJointLeftAnkle)
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

bool cSimRaptor::FailFallMisc() const
{
	bool fallen = false;
	tQuaternion root_q = GetRootRotation();
	double root_theta = cMathUtil::QuatTheta(root_q);
	
	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}