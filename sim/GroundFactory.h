#pragma once

#include "sim/Ground.h"

class cGroundFactory
{
public:

	static bool ParseParamsJson(const std::string& param_file, cGround::tParams& out_params);
	static void BuildGround(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);

protected:

	static bool ParseParamsJason(cGround::eType type, const Json::Value& json, Eigen::VectorXd& out_params);
	static void BuildGroundPlane(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundVar2D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundVar3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundHills3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundTrail3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundObstacles3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundDynamicObstacles3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
	static void BuildGroundConveyor3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground);
};