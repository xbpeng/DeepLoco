/*
 * MotionDB.h
 *
 *  Created on: Nov 10, 2016
 *      Author: Glen
 */

#ifndef UTIL_MOTIONDB_H_
#define UTIL_MOTIONDB_H_
#include <vector>
#include "MathUtil.h"
#include "BVHReader.h"


/*
 * This class will encapsulate all of the storage and querying of motion data.
 */

enum StanceFoot
{
	LeftStance,
	RightStance
};

struct MotionClip
{
	size_t num_frames;            // 
	size_t num_motion_channels = 0;   // 
	StanceFoot stanceFoot;
	tVector nextStepLocation; // This is a relative location
	tVector rootHeadingAtNextStep;
	tVector rootHeadingAtCurrentStep;
	double frame_step = 0.0;     // The timestep of the recording
	Eigen::MatrixXd data;                        // motion data of rotations
};

class cMotionDB {
public:
	cMotionDB();
	virtual ~cMotionDB();
	/// processes the motion capture data and adds footstep clips to the DB
	virtual void processMotionClips(const cBVHReader & BVHData);
	virtual size_t getNumClips() const { return this->_motionClips.size(); }
	virtual MotionClip getClip(size_t clip) const { return this->_motionClips[clip]; }

private:
	virtual MotionClip getMotionClip(size_t startFrame, size_t endFrame, const cBVHReader & BVHData) const;

	std::vector<MotionClip> _motionClips;
	std::shared_ptr<cBVHReader::tJointData> rootJoint;
};

#endif /* UTIL_MOTIONDB_H_ */
