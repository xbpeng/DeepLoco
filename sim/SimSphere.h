#pragma once

#include "sim/SimObj.h"

class cSimSphere : public cSimObj
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		eType mType;
		double mMass;
		double mFriction;
		tVector mPos;
		tVector mVel;
		double mRadius;
		tVector mAxis;
		double mTheta;
	};

	cSimSphere();
	virtual ~cSimSphere();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
	double GetRadius() const;
	tVector GetSize() const;

	virtual eShape GetShape() const;

protected:
};