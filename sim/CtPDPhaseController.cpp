#include "CtPDPhaseController.h"
#include "sim/SimCharacter.h"

cCtPDPhaseController::cCtPDPhaseController() : cCtController(),
											   cCtPhaseController(),
											   cCtPDController()
{
}

cCtPDPhaseController::~cCtPDPhaseController()
{
}

void cCtPDPhaseController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cCtPhaseController::Init(character);
	cCtPDController::Init(character, gravity, param_file);
    // hack to default the num samples to 0 because thats what all the old args files are expecting and TerrainRLChar defaults to 200 for some reason...
    cTerrainRLCharController::SetNumGroundSamples(0);
}

void cCtPDPhaseController::GetViewBound(tVector& out_min, tVector& out_max) const
{
    cTerrainRLCharController::GetViewBound(out_min, out_max);
    /*  
    double view_dist = GetViewDist();
	tVector root_pos = mChar->GetRootPos();
	root_pos[1] = 0;
	out_min = root_pos + tVector(-2, 0, -2, 0);
	out_max = root_pos + tVector(view_dist, 0, 2, 0);
    */
}

int cCtPDPhaseController::GetNumGroundSamples() const
{
    return cTerrainRLCharController::GetNumGroundSamples();
}

tVector cCtPDPhaseController::CalcGroundSamplePos(int s) const
{
    return cTerrainRLCharController::CalcGroundSamplePos(s);
}
