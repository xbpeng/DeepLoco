#pragma once

#include "sim/GroundVar3D.h"

class cGroundHills3D : public cGroundVar3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundHills3D();
	virtual ~cGroundHills3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params,
						const tVector& bound_min, const tVector& bound_max);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual eClass GetGroundClass() const;

protected:
	
	Eigen::MatrixXd mNoise;
	tVector mUVOffset;

	virtual void InitSlabs(const tVector& bound_min, const tVector& bound_max);
	virtual void BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
									std::vector<float>& out_data, std::vector<int>& out_flags);

	virtual void InitNoise(int res);
	virtual double SampleNoise(const tVector& uv) const;
	virtual double CalcNoiseHeight(const tVector& pos) const;
};
