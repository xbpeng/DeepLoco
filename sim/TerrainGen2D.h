#pragma once

#include <vector>
#include "util/Rand.h"
#include "util/ArgParser.h"
#include "util/MathUtil.h"
#include "util/JsonUtil.h"
#include "Ground.h"

class cTerrainGen2D
{
public:

	enum eParams
	{
		eParamsGapSpacingMin,
		eParamsGapSpacingMax,
		eParamsGapWidthMin,
		eParamsGapWidthMax,
		eParamsGapDepthMin,
		eParamsGapDepthMax,

		eParamsWallSpacingMin,
		eParamsWallSpacingMax,
		eParamsWallWidthMin,
		eParamsWallWidthMax,
		eParamsWallHeightMin,
		eParamsWallHeightMax,

		eParamsStepSpacingMin,
		eParamsStepSpacingMax,
		eParamsStepHeight0Min,
		eParamsStepHeight0Max,
		eParamsStepHeight1Min,
		eParamsStepHeight1Max,

		eParamsBumpHeightMin,
		eParamsBumpHeightMax,

		eParamsNarrowGapSpacingMin,
		eParamsNarrowGapSpacingMax,
		eParamsNarrowGapDistMin,
		eParamsNarrowGapDistMax,
		eParamsNarrowGapWidthMin,
		eParamsNarrowGapWidthMax,
		eParamsNarrowGapDepthMin,
		eParamsNarrowGapDepthMax,
		eParamsNarrowGapCountMin,
		eParamsNarrowGapCountMax,

		eParamsCliffSpacingMin,
		eParamsCliffSpacingMax,
		eParamsCliffHeight0Min,
		eParamsCliffHeight0Max,
		eParamsCliffHeight1Min,
		eParamsCliffHeight1Max,
		eParamsCliffMiniCountMax,

		eParamsSlopeDeltaRange,
		eParamsSlopeDeltaMin,
		eParamsSlopeDeltaMax,

		eParamsMax
	};

	struct tParamDef
	{
		std::string mName;
		double mDefaultVal;
	};

	static const tParamDef gParamDefs[];
	static void GetDefaultParams(Eigen::VectorXd& out_params);
	static void LoadParams(const Json::Value& root, Eigen::VectorXd& out_params);

	typedef double (*tTerrainFunc)(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static tTerrainFunc GetTerrainFunc(cGround::eType terrain_type);
	static const float gVertSpacing;

	static double BuildFlat(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildGaps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSteps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildWalls(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildBumps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildMixed(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildNarrowGaps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	
	static double BuildSlopes(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSlopesGaps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSlopesSteps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSlopesWalls(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSlopesMixed(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	static double BuildSlopesNarrowGaps(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);
	
	static double BuildCliffs(double width, const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data);

	static double AddFlat(double width, std::vector<float>& out_data);
	static double AddStep(double width, double height, std::vector<float>& out_data);
	static double AddBox(double spacing, double width, double depth, std::vector<float>& out_data);
	static double AddSlope(double width, double slope, std::vector<float>& out_data);
	
	static void OverlaySlopes(double delta_range, double delta_min, double delta_max, double init_slope, int beg_idx, int end_idx, cRand& rand, std::vector<float>& out_data);
	static void OverlayBumps(double min_delta_h, double max_delta_h, int beg_idx, int end_idx, cRand& rand, std::vector<float>& out_data);

protected:
	static int CalcNumVerts(double w);
};