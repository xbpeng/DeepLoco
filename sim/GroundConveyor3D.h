#pragma once

#include "sim/GroundDynamicObstacles3D.h"

class cGroundConveyor3D : public cGroundDynamicObstacles3D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundConveyor3D();
	virtual ~cGroundConveyor3D();

	virtual eClass GetGroundClass() const;

protected:
	
	struct tStrip
	{
		tVector mAnchorPos;
		double mLen;
		double mWidth;
		int mHead;
		std::vector<int> mSliceObstacles;

		tStrip();

		double GetSliceLength() const;
		int GetNumSlices() const;
		int GetTailID() const;
		void IncHead();
	};

	std::vector<tStrip> mStrips;

	virtual void BuildObstacles();
	virtual void BuildStrip(int num_slices, double strip_width, double strip_len, double speed,
							const tVector& pos, tStrip& out_strip);
	virtual void BuildStripSlice(const tVector& pos, double strip_width, double strip_len, tObstacle::eDir dir, 
								double speed, tObstacle& out_obstacle);

	virtual int GetNumStrips() const;

	virtual void UpdateObstacles(double time_elapsed);
	virtual void UpdateStrips();
};