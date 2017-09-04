#include "sim/SimCharGeneral.h"
#include <iostream>

cSimCharGeneral::cSimCharGeneral()
{
}

cSimCharGeneral::~cSimCharGeneral()
{
}

bool cSimCharGeneral::HasStumbled() const
{
	bool stumbled = false;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		if (!IsEndEffector(j))
		{
			const auto& curr_part = GetBodyPart(j);
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