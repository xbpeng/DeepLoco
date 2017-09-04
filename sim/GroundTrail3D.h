#pragma once

#include "sim/GroundHills3D.h"

class cGroundTrail3D : public cGroundHills3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tTrailSeg
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tVector mStart;
		tVector mEnd;
		double mWidth;

		tTrailSeg();
		void BuildAABB(tVector& out_min, tVector& out_max) const;
	};

	cGroundTrail3D();
	virtual ~cGroundTrail3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params,
						const tVector& bound_min, const tVector& bound_max);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual eClass GetGroundClass() const;
	virtual int GetNumSegs() const;
	virtual const tTrailSeg& GetSeg(int i) const;

	virtual int FindNearestSegment(const tVector& pos) const;
	virtual double CalcTrailDist(const tVector& pos_beg, const tVector& pos_end) const;
	virtual void FindRandTrailPlacement(tVector& out_pos, tQuaternion& out_rot);

protected:
	
	std::vector<tTrailSeg, Eigen::aligned_allocator<tTrailSeg>> mTrail;
	
	virtual void InitSlabs(const tVector& bound_min, const tVector& bound_max);
	virtual void InitTrail(const tVector& bound_min, const tVector& bound_max);
	virtual void UpdateTrail(const tVector& bound_min, const tVector& bound_max);
	virtual void BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
									std::vector<float>& out_data, std::vector<int>& out_flags);

	virtual double CalcNoiseHeight(const tVector& pos) const;

	virtual void RasterizeSeg(const tTrailSeg& seg, const tVector& bound_min, const tVector& bound_max, std::vector<float>& out_data);
};
