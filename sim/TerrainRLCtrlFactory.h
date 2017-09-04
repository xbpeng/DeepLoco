#pragma once

#include <vector>
#include <string>
#include <memory>

#include "sim/SimCharacter.h"
#include "sim/TerrainRLCharController.h"
#include "sim/Ground.h"

class cTerrainRLCtrlFactory
{
public:
	enum eCharCtrl
	{
		eCharCtrlNone,
		eCharCtrlCt,
		eCharCtrlCtPhase,
		eCharCtrlCtPDPhase,
		eCharCtrlCtTarget,
		eCharCtrlWaypoint,
		eCharCtrlWaypointVel,
		eCharCtrlSoccer,
		eCharCtrlBiped3DStep,
		eCharCtrlMax
	};

	struct tCtrlParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eCharCtrl mCharCtrl;
		std::vector<std::string> mNetFiles;
		std::string mCtrlParamFile;

		std::shared_ptr<cSimCharacter> mChar;
		std::shared_ptr<cGround> mGround;
		tVector mGravity;

		double mCycleDur;
		double mCtQueryRate;

        // 2D terrain samples 
        int mNumGroundSamples;
        // 3D terrain samples 
        int mGroundSampleRes3d;
        //
        int mViewDist;


		// waypoint params
		double mWaypointInitStepLen;
		
		tCtrlParams();
	};
	
	enum eNetFile
	{
		eNetFileActor,
		eNetFileCritic,
		eNetFileActor1,
		eNetFileCritic1,

		eNetFileActorSolver,
		eNetFileCriticSolver,
		eNetFileActor1Solver,
		eNetFileCritic1Solver,

		eNetFileActorModel,
		eNetFileCriticModel,
		eNetFileActor1Model,
		eNetFileCritic1Model,
		eNetFileActorTerrainModel,
		eNetFileActorActionModel,
		eNetFileCriticTerrainModel,
		eNetFileCriticValueModel,
		eNetFileForwardDynamics,
		eNetFileForwardDynamicsSolver,
		eNetFileForwardDynamicsModel,
		eNetFileMax
	};

	static void InitNetFileArray(std::vector<std::string>& out_arr);
	static void ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl);
	static bool BuildController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	
protected:

	static bool BuildDogController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerCaclaDQ(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerDMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerDPG(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildDogControllerMACEDPG(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	
	static bool BuildGoatControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildMonopedHopperController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildMonopedHopperControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildMonopedHopperControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildRaptorController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildRaptorControllerCacla(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildRaptorControllerMACE(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildBipedController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedController2D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedController2DCACLA(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedController2DCaclaFD(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedController2DParameterized(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	// static bool BuildBipedController3D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedController3DCACLA(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtRNNController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtNPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtMTUController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtVelTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtNPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtMTUTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtVelPhaseController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	
	static bool BuildCtQController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtQPDController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtQVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtQTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtQPDTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtQVelTrackController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildCtTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDTrackTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildCtPDPhaseTargetController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);

	static bool BuildWaypointController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildWaypointVelController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildSoccerController(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
	static bool BuildBipedStepController3D(const tCtrlParams& params, std::shared_ptr<cCharController>& out_ctrl);
};
