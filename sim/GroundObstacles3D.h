#pragma once

#include "sim/GroundTrail3D.h"

class cGroundObstacles3D : public cGroundTrail3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundObstacles3D();
	virtual ~cGroundObstacles3D();

	virtual tVector FindRandFlatPos();
	virtual eClass GetGroundClass() const;

protected:

	virtual void InitTrail(const tVector& bound_min, const tVector& bound_max);
	virtual void UpdateTrail(const tVector& bound_min, const tVector& bound_max);
	virtual void BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max,
									std::vector<float>& out_data, std::vector<int>& out_flags);

	virtual void CullSegs(const tVector& bound_min, const tVector& bound_max);
	virtual void RasterizeSeg(const tTrailSeg& seg, const tVector& bound_min, const tVector& bound_max, std::vector<float>& out_data);
};
