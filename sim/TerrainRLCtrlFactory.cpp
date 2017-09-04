#include "sim/TerrainRLCtrlFactory.h"

#include "sim/CtController.h"
#include "sim/CtPhaseController.h"
#include "sim/CtPDPhaseController.h"
#include "sim/CtTargetController.h"
#include "sim/WaypointController.h"
#include "sim/WaypointVelController.h"
#include "sim/SoccerController.h"
#include "sim/BipedStepController3D.h"

const std::string gCharCtrlName[cTerrainRLCtrlFactory::eCharCtrlMax] =
{
	"none",
	"ct",
	"ct_phase",
	"ct_pd_phase",
	"ct_target",
	"waypoint",
	"waypoint_vel",
	"soccer",
	"biped3d_step"
};

cTerrainRLCtrlFactory::tCtrlParams::tCtrlParams()
{
	mCharCtrl = eCharCtrlNone;
	mCtrlParamFile = "";
	mCtrlParamFile = "";

	mChar = nullptr;
	mGround = nullptr;
	mGravity = tVector(0, -9.8, 0, 0);

	mCycleDur = 1;
	mCtQueryRate = 60; // policy queries per second
    
    // hack: don't override unless specified
    mNumGroundSamples = -1;
    mGroundSampleRes3d = -1;
    mViewDist = -1;
	mWaypointInitStepLen = 0;

	InitNetFileArray(mNetFiles);
}

void cTerrainRLCtrlFactory::InitNetFileArray(std::vector<std::string>& out_arr)
{
	out_arr.resize(eNetFileMax);
	for (int f = 0; f < eNetFileMax; ++f)
	{
		out_arr[f] = "";
	}
}

void cTerrainRLCtrlFactory::ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl)
{
	bool found = false;
	if (char_ctrl_str == "")
	{
		out_char_ctrl = eCharCtrlNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharCtrlMax; ++i)
		{
			const std::string& name = gCharCtrlName[i];
			if (char_ctrl_str == name)
			{
				out_char_ctrl = static_cast<eCharCtrl>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported character controller"); // unsupported character controller
	}
}

bool cTerrainRLCtrlFactory::BuildController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	switch (params.mCharCtrl)
	{
	case eCharCtrlNone:
		break;
	case eCharCtrlCt:
		succ = BuildCtController(params, out_ctrl);
		break;
	case eCharCtrlCtPhase:
		succ = BuildCtPhaseController(params, out_ctrl);
		break;
	case eCharCtrlCtPDPhase:
		succ = BuildCtPDPhaseController(params, out_ctrl);
		break;
	case eCharCtrlCtTarget:
		succ = BuildCtTargetController(params, out_ctrl);
		break;
	case eCharCtrlWaypoint:
		succ = BuildWaypointController(params, out_ctrl);
		break;
	case eCharCtrlWaypointVel:
		succ = BuildWaypointVelController(params, out_ctrl);
		break;
	case eCharCtrlSoccer:
		succ = BuildSoccerController(params, out_ctrl);
		break;
	case eCharCtrlBiped3DStep:
		succ = BuildBipedStepController3D(params, out_ctrl);
		break;
	default:
		assert(false && "Failed Building Unsupported Controller"); // unsupported controller
		break;
	}

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtController> ctrl = std::shared_ptr<cCtController>(new cCtController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPhaseController> ctrl = std::shared_ptr<cCtPhaseController>(new cCtPhaseController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtPDPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtPDPhaseController> ctrl = std::shared_ptr<cCtPDPhaseController>(new cCtPDPhaseController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);
    if(params.mNumGroundSamples >= 0)
    {
        ctrl->SetNumGroundSamples(params.mNumGroundSamples);
    }
    if(params.mViewDist >= 0)
    {
        ctrl->SetViewDist(params.mViewDist);
    }
	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildCtTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	std::shared_ptr<cCtTargetController> ctrl = std::shared_ptr<cCtTargetController>(new cCtTargetController());
	ctrl->SetGround(params.mGround);
	ctrl->Init(params.mChar.get());
	ctrl->SetUpdatePeriod(update_period);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);

		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildWaypointController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	BuildBipedStepController3D(params, llc);
	
	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cWaypointController>(new cWaypointController());

	ctrl->SetGround(params.mGround);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildWaypointVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	BuildBipedStepController3D(params, llc);

	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cWaypointVelController>(new cWaypointVelController());
	
	ctrl->SetGround(params.mGround);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildSoccerController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cCharController> llc;
	BuildBipedStepController3D(params, llc);

	auto step_ctrl = std::dynamic_pointer_cast<cBipedStepController3D>(llc);
	auto ctrl = std::shared_ptr<cSoccerController>(new cSoccerController());
	
	ctrl->SetGround(params.mGround);
	ctrl->SetInitStepLen(params.mWaypointInitStepLen);
	ctrl->Init(params.mChar.get());
	ctrl->SetLLC(step_ctrl);

	const std::string& poli_net_file = params.mNetFiles[eNetFileActor1];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActor1Model];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic1];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCritic1Model];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}

bool cTerrainRLCtrlFactory::BuildBipedStepController3D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl)
{
	const double update_period = 1 / params.mCtQueryRate;
	bool succ = true;

	auto ctrl = std::shared_ptr<cBipedStepController3D>(new cBipedStepController3D());
	ctrl->SetGround(params.mGround);
    if(params.mGroundSampleRes3d >= 0)
    {
	    ctrl->SetGroundSampleRes(params.mGroundSampleRes3d);
    }
	ctrl->Init(params.mChar.get(), params.mGravity, params.mCtrlParamFile);
	ctrl->SetUpdatePeriod(update_period);
	ctrl->SetCycleDur(params.mCycleDur);
    if(params.mGroundSampleRes3d >= 0)
    {
	    ctrl->SetGroundSampleRes(params.mGroundSampleRes3d);
    }
	const std::string& poli_net_file = params.mNetFiles[eNetFileActor];
	const std::string& poli_model_file = params.mNetFiles[eNetFileActorModel];
	const std::string& critic_net_file = params.mNetFiles[eNetFileCritic];
	const std::string& critic_model_file = params.mNetFiles[eNetFileCriticModel];

	if (poli_net_file != "")
	{
		succ &= ctrl->LoadNet(poli_net_file);
		if (succ && poli_model_file != "")
		{
			ctrl->LoadModel(poli_model_file);
		}
	}

	if (critic_net_file != "")
	{
		bool critic_succ = ctrl->LoadCriticNet(critic_net_file);
		succ &= critic_succ;
		if (critic_succ && critic_model_file != "")
		{
			ctrl->LoadCriticModel(critic_model_file);
		}
	}

	out_ctrl = ctrl;

	return succ;
}