#pragma once

#include "sim/WaypointController.h"

class cSoccerController : public virtual cWaypointController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSoccerController();
	virtual ~cSoccerController();

	virtual void SetBall(const std::shared_ptr<cSimObj>& ball);
	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	
protected:

	enum eBallState
	{
		eBallStatePosX,
		eBallStatePosY,
		eBallStatePosZ,
		eBallStateVelX,
		eBallStateVelY,
		eBallStateVelZ,
		eBallStateAngVelX,
		eBallStateAngVelY,
		eBallStateAngVelZ,
		eBallStateMax,
	};

	std::shared_ptr<cSimObj> mBall;

	virtual int GetGroundSampleRes() const;

	virtual int GetPoliStateSize() const;
	virtual int GetBallStateOffset() const;
	virtual int GetBallStateSize() const;
	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildBallState(Eigen::VectorXd& out_state) const;
	virtual void BuildTargetPosState(Eigen::VectorXd& out_state) const;
};