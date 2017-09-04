#pragma once
#include "sim/CtPhaseController.h"
#include "sim/CtPDController.h"

class cCtPDPhaseController : public virtual cCtPhaseController, public virtual cCtPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cCtPDPhaseController();
	virtual ~cCtPDPhaseController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);

    // Hack to get 2D terrain working, someone should fix the inheritance
    // Needs to skip over the 3D terrain stuff in CtController from which this inherits
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;
	virtual int GetNumGroundSamples() const;
	virtual tVector CalcGroundSamplePos(int s) const;
protected:

};
