#include "GroundFactory.h"
#include "sim/GroundPlane.h"
#include "sim/GroundVar2D.h"
#include "sim/GroundVar3D.h"
#include "sim/GroundTrail3D.h"
#include "sim/GroundObstacles3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/GroundConveyor3D.h"

bool cGroundFactory::ParseParamsJson(const std::string& param_file, cGround::tParams& out_params)
{
	std::ifstream f_stream(param_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[cGround::gTypeKey].isNull())
		{
			std::string type_str = root[cGround::gTypeKey].asString();
			cGround::ParseType(type_str, out_params.mType);
		}

		if (!root[cGround::gParamsKey].isNull())
		{
			Json::Value params_arr = root[cGround::gParamsKey];
			assert(params_arr.isArray());
			int num = params_arr.size();

			out_params.mParamArr.resize(0, 0);
			if (num > 0)
			{
				for (int i = 0; i < num; ++i)
				{
					Eigen::VectorXd curr_params;
					succ &= ParseParamsJason(out_params.mType, params_arr.get(i, 0), curr_params);

					if (succ)
					{
						if (i == 0)
						{
							size_t m = num;
							size_t n = curr_params.size();
							out_params.mParamArr.resize(m, n);
						}
						out_params.mParamArr.row(i) = curr_params;
					}
				}
			}
		}
	}

	return succ;
}

void cGroundFactory::BuildGround(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	cGround::eClass ground_class = cGround::GetClassFromType(params.mType);
	switch (ground_class)
	{
	case cGround::eClassPlane:
		BuildGroundPlane(world, params, out_ground);
		break;
	case cGround::eClassVar2D:
		BuildGroundVar2D(world, params, out_ground);
		break;
	case cGround::eClassVar3D:
		BuildGroundVar3D(world, params, out_ground);
		break;
	case cGround::eClassHills3D:
		BuildGroundHills3D(world, params, out_ground);
		break;
	case cGround::eClassTrail3D:
		BuildGroundTrail3D(world, params, out_ground);
		break;
	case cGround::eClassObstacles3D:
		BuildGroundObstacles3D(world, params, out_ground);
		break;
	case cGround::eClassDynamicObstacles3D:
		BuildGroundDynamicObstacles3D(world, params, out_ground);
		break;
	case cGround::eClassConveyor3D:
		BuildGroundConveyor3D(world, params, out_ground);
		break;
	default:
		assert(false); // unsupported ground class
		break;
	}
}

bool cGroundFactory::ParseParamsJason(cGround::eType type, const Json::Value& json, Eigen::VectorXd& out_params)
{
	cGround::eClass ground_class = cGround::GetClassFromType(type);
	bool succ = false;
	switch (ground_class)
	{
	case cGround::eClassPlane:
		succ = cGroundPlane::ParseParamsJson(json, out_params);
		break;
	case cGround::eClassVar2D:
		succ = cGroundVar2D::ParseParamsJson(json, out_params);
		break;
	case cGround::eClassVar3D:
	case cGround::eClassTrail3D:
	case cGround::eClassObstacles3D:
	case cGround::eClassDynamicObstacles3D:
	case cGround::eClassConveyor3D:
		succ = cGroundVar3D::ParseParamsJson(json, out_params);
		break;
	default:
		assert(false); // unsupported ground class
		break;
	}
	return succ;
}

void cGroundFactory::BuildGroundPlane(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	std::shared_ptr<cGroundPlane> ground_plane = std::shared_ptr<cGroundPlane>(new cGroundPlane());
	ground_plane->Init(world, params);
	out_ground = ground_plane;
}

void cGroundFactory::BuildGroundVar2D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	const double spawn_offset = -1; // some padding to prevent parts of character from getting spawned inside obstacles

	std::shared_ptr<cGroundVar2D> ground_var2d = std::shared_ptr<cGroundVar2D>(new cGroundVar2D());
	auto terrain_func = cTerrainGen2D::GetTerrainFunc(params.mType);
	double half_width = params.mGroundWidth / 2;

	tVector bound_min = tVector(-half_width + spawn_offset, 0, 0, 0);
	tVector bound_max = tVector(half_width + spawn_offset, 0, 0, 0);

	ground_var2d->SetTerrainFunc(terrain_func);
	ground_var2d->Init(world, params, bound_min, bound_max);

	out_ground = ground_var2d;
}

void cGroundFactory::BuildGroundVar3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	const double spawn_offset = 0;

	std::shared_ptr<cGroundVar3D> ground_var3d = std::shared_ptr<cGroundVar3D>(new cGroundVar3D());
	auto terrain_func = cTerrainGen3D::GetTerrainFunc(params.mType);
	double half_width = params.mGroundWidth / 2;

	tVector bound_min = tVector(-half_width + spawn_offset, 0, -half_width + spawn_offset, 0);
	tVector bound_max = tVector(half_width + spawn_offset, 0, half_width + spawn_offset, 0);

	ground_var3d->SetTerrainFunc(terrain_func);
	ground_var3d->Init(world, params, bound_min, bound_max);
	out_ground = ground_var3d;
}

void cGroundFactory::BuildGroundHills3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	const double spawn_offset = 0;

	auto ground_hills3d = std::shared_ptr<cGroundHills3D>(new cGroundHills3D());
	double half_width = params.mGroundWidth / 2;

	tVector bound_min = tVector(-half_width + spawn_offset, 0, -half_width + spawn_offset, 0);
	tVector bound_max = tVector(half_width + spawn_offset, 0, half_width + spawn_offset, 0);

	ground_hills3d->Init(world, params, bound_min, bound_max);
	out_ground = ground_hills3d;
}

void cGroundFactory::BuildGroundTrail3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	const double spawn_offset = 0;

	auto ground_trail3d = std::shared_ptr<cGroundTrail3D>(new cGroundTrail3D());
	double half_width = params.mGroundWidth / 2;

	tVector bound_min = tVector(-half_width + spawn_offset, 0, -half_width + spawn_offset, 0);
	tVector bound_max = tVector(half_width + spawn_offset, 0, half_width + spawn_offset, 0);

	ground_trail3d->Init(world, params, bound_min, bound_max);
	out_ground = ground_trail3d;
}

void cGroundFactory::BuildGroundObstacles3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	const double spawn_offset = 0;

	auto ground = std::shared_ptr<cGroundObstacles3D>(new cGroundObstacles3D());
	double half_width = params.mGroundWidth / 2;

	tVector bound_min = tVector(-half_width + spawn_offset, 0, -half_width + spawn_offset, 0);
	tVector bound_max = tVector(half_width + spawn_offset, 0, half_width + spawn_offset, 0);

	ground->Init(world, params, bound_min, bound_max);
	out_ground = ground;
}

void cGroundFactory::BuildGroundDynamicObstacles3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	auto ground = std::shared_ptr<cGroundDynamicObstacles3D>(new cGroundDynamicObstacles3D());
	ground->Init(world, params);
	out_ground = ground;
}

void cGroundFactory::BuildGroundConveyor3D(const std::shared_ptr<cWorld>& world, const cGround::tParams& params, std::shared_ptr<cGround>& out_ground)
{
	auto ground = std::shared_ptr<cGroundConveyor3D>(new cGroundConveyor3D());
	ground->Init(world, params);
	out_ground = ground;
}