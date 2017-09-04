/*
 * MotionDB.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: Glen
 */

#include "MotionDB.h"
#include <iostream>

cMotionDB::cMotionDB() {
	// TODO Auto-generated constructor stub

}

cMotionDB::~cMotionDB() {
	// TODO Auto-generated destructor stub
}

MotionClip cMotionDB::getMotionClip(size_t startFrame, size_t endFrame, const cBVHReader & BVHData) const
{
	MotionClip clip;
	tVector pos;
	tVector nextStepRelativeLocation;
	tVector root_rot_v;
	root_rot_v(0) = 1.0;
	tMatrix trans;
	BVHData.GetJointLocation("Hips", startFrame, pos, trans);
	int hip_id = BVHData.FindJoint("Hips");
	const cBVHReader::tJointData& root_joint = BVHData.GetJointData(hip_id);
	tMatrix root_rot = BVHData.getRotationForFrame(root_joint, startFrame);
	clip.rootHeadingAtCurrentStep = root_rot * root_rot_v;
	root_rot = BVHData.getRotationForFrame(root_joint, endFrame);
	clip.rootHeadingAtNextStep = root_rot * root_rot_v;
	// clip.frame_step = motionData.frame_step;
	clip.stanceFoot = LeftStance;
	if (clip.stanceFoot == LeftStance)
	{
		tVector pos2;
		BVHData.GetJointLocation("LeftFoot", startFrame, pos, trans);
		BVHData.GetJointLocation("RightFoot", endFrame, pos2, trans);
		nextStepRelativeLocation = pos2 - pos;
	}
	clip.nextStepLocation = nextStepRelativeLocation;
	clip.num_frames = endFrame - startFrame;
	clip.data = BVHData.getMotoinData().block(startFrame, 0, endFrame - startFrame, BVHData.getMotoinData().cols());
	return clip;
}

void cMotionDB::processMotionClips(const cBVHReader & BVHData)
{


	// Detect when heel hits the ground.
	// when heel velocity goes from negative to posative. 
	// need to get location of heel

	tVector position = tVector::Zero(); // stores previous foot location
	position(3) = 1.0;
	tMatrix motionTransLeftFoot;
	/*
	JointData* leftThigh = getJoint("LeftUpLeg");
	JointData* leftChin = getJoint("LeftLeg");
	JointData* leftFoot = getJoint("LeftFoot");
	tMatrix motionTransRoot = rootJoint->matrix * getTransformationForFrame(rootJoint->channel_start, rootJoint->channels_order, motionData.data, 0);
	tMatrix motionTransLeftThigh = leftThigh->matrix * getTransformationForFrame(leftThigh->channel_start, leftThigh->channels_order, motionData.data, 0);
	tMatrix motionTransLeftChin = leftChin->matrix * getTransformationForFrame(leftChin->channel_start, leftChin->channels_order, motionData.data, 0);
	motionTransLeftFoot = leftFoot->matrix * getTransformationForFrame(leftFoot->channel_start, leftFoot->channels_order, motionData.data, 0);
	std::cout << "root joint mat: " << rootJoint->matrix << std::endl;
	std::cout << "motion transformation mat: root: " << motionTransRoot << std::endl;
	std::cout << "motion transformation mat: left thigh: " << motionTransLeftThigh << std::endl;
	std::cout << "motion transformation mat: left chin" << motionTransLeftChin << std::endl;
	std::cout << "motion transformation mat: left foot" << motionTransLeftFoot << std::endl;

	position = motionTransRoot * motionTransLeftThigh * motionTransLeftChin * motionTransLeftFoot * position;
	std::cout << "computed position: " << position << std::endl;
	*/

	BVHData.GetJointLocation("LeftFoot", 1, position, motionTransLeftFoot);
	// std::cout << "computed position2: " << position << std::endl;
	std::vector<double> stepFrames;
	tVector position3 = tVector::Zero(); // store the position 2 frames back
	BVHData.GetJointLocation("LeftFoot", 0, position3, motionTransLeftFoot);

	for (size_t f = 2; f < BVHData.GetNumFrames() && (f < 500); f++)
	{
		tVector position2 = tVector::Zero(); // stores the current frame
		BVHData.GetJointLocation("LeftFoot", f, position2, motionTransLeftFoot);
		// getJointLocation("LeftFoot", f-1, position, motionTransLeftFoot);
		// getJointLocation("LeftFoot", f-2, position2, motionTransLeftFoot);

		// position = tVector::Zero(); // stores previous foot location
		// position(3) = 1.0;
		// JointData* leftThigh = getJoint("LeftUpLeg");
		// JointData* leftChin = getJoint("LeftLeg");
		// JointData* leftFoot = getJoint("LeftFoot");
		// tMatrix motionTransRoot = rootJoint->matrix * getTransformationForFrame(rootJoint->channel_start, rootJoint->channels_order, motionData.data, f);
		// tMatrix motionTransLeftThigh = leftThigh->matrix * getTransformationForFrame(leftThigh->channel_start, leftThigh->channels_order, motionData.data, f);
		// tMatrix motionTransLeftChin = leftChin->matrix * getTransformationForFrame(leftChin->channel_start, leftChin->channels_order, motionData.data, f);
		// Eigen::Vector3d ea = motionTransLeftChin.eulerAngles(0, 1, 2);
		// motionTransLeftChin.
		// tMatrix motionTransLeftFoot2 = leftFoot->matrix * getTransformationForFrame(leftFoot->channel_start, leftFoot->channels_order, motionData.data, f);
		// std::cout << "root joint mat: " << rootJoint->matrix << std::endl;
		// std::cout << "motion transformation mat: root: " << motionTransRoot << std::endl;
		// std::cout << "motion transformation mat: left thigh: " << motionTransLeftThigh << std::endl;
		// std::cout << "motion transformation mat: left chin" << motionTransLeftChin << std::endl;
		// std::cout << "motion transformation mat: left foot" << motionTransLeftFoot2 << std::endl;

		// position2 = motionTransRoot * motionTransLeftThigh * motionTransLeftChin * motionTransLeftFoot2 * position;

		// std::cout << " 1 " << position3.transpose() << std::endl;
		// std::cout << " 2 " << position.transpose() << std::endl;
		std::cout << " 3 " << position2.transpose() << std::endl;
		if ((position2(1) > position(1)) && (position3(1) > position(1)))
		{
			stepFrames.push_back(f);
			std::cout << "Found a left foot step at frame: " << f << std::endl;
		}
		position3 = position;
		position = position2;
	}
	// Eigen::VectorXd v2(v1.data());
	this->_motionClips.clear();
	for (size_t step = 1; step < stepFrames.size(); step++)
	{
		MotionClip clip = this->getMotionClip(stepFrames[step - 1], stepFrames[step], BVHData);
		this->_motionClips.push_back(clip);
	}
	Eigen::VectorXd data__ = Eigen::VectorXd::Map(stepFrames.data(), stepFrames.size());
	Eigen::MatrixXd data_;
	std::cout << "Frames with steps: " << data__.transpose() << std::endl;
	return;
}



