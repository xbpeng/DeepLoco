#pragma once

#include "sim/WaypointController.h"

class cWaypointVelController : public virtual cWaypointController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cWaypointVelController();
	virtual ~cWaypointVelController();

	virtual tVector GetGroundVelSample(int s) const;
	
protected:

	static const int gVelSampleDim;
	tVector mGroundOriginVel;
	Eigen::MatrixXd mGroundVelSamples;

	virtual void InitGroundSamples();
	virtual void ParseGround();
	virtual void SampleGround(Eigen::VectorXd& out_h, Eigen::MatrixXd& out_vel) const;
	
	virtual int GetGroundFeatureSize() const;
	virtual void BuildPoliStateGround(Eigen::VectorXd& out_ground) const;
};