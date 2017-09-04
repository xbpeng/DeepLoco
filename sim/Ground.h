#pragma once

#include "sim/SimObj.h"

class cGround : public cSimObj
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eClass
	{
		eClassPlane,
		eClassVar2D,
		eClassVar3D,
		eClassHills3D,
		eClassTrail3D,
		eClassObstacles3D,
		eClassDynamicObstacles3D,
		eClassConveyor3D,
		eClassMax,
		eClassInvalid
	};

	enum eType
	{
		eTypePlane,
		eTypeVar2DFlat,
		eTypeVar2DGaps,
		eTypeVar2DSteps,
		eTypeVar2DWalls,
		eTypeVar2DBumps,
		eTypeVar2DMixed,
		eTypeVar2DNarrowGaps,
		eTypeVar2DSlopes,
		eTypeVar2DSlopesGaps,
		eTypeVar2DSlopesWalls,
		eTypeVar2DSlopesSteps,
		eTypeVar2DSlopesMixed,
		eTypeVar2DSlopesNarrowGaps,
		eTypeVar2DCliffs,
		eTypeVar3DFlat,
		eTypeVar3DPath,
		eTypeVar3DCliff,
		eTypeVar3DRamp,
		eTypeHills3D,
		eTypeTrail3D,
		eTypeObstacles3D,
		eTypeDynamicObstacles3D,
		eTypeConveyor3D,
        eTypeVar3DCheckers,
        eTypeVar3DStairs,
		eTypeMax
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eType mType;
		double mFriction;
		tVector mOrigin;
		double mBlend;

		double mGroundWidth;

		unsigned long mRandSeed;
		bool mHasRandSeed;

		Eigen::MatrixXd mParamArr;

		tParams();
	};

	static const std::string gTypeKey;
	static const std::string gParamsKey;
	
	static eClass GetClassFromType(eType ground_type);
	static void ParseType(const std::string& str, eType& out_type);
	
	virtual ~cGround();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;
	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;

	virtual eClass GetGroundClass() const;
	virtual void SetBlendParams(const Eigen::VectorXd& params);
	virtual void SetParamBlend(double blend);

	virtual size_t GetUpdateCount() const;
	virtual void SeedRand(unsigned long seed);

	virtual bool Output(const std::string& out_file) const;

protected:

	cRand mRand;
	tParams mParams;
	Eigen::VectorXd mBlendParams;
	size_t mUpdateCount;
	double mTime;

	cGround();

	virtual void SetupRandGen();
	virtual void CalcBlendParams(double blend, Eigen::VectorXd& out_params) const;
	virtual int GetNumParamSets() const;
	virtual int GetBlendParamSize() const;

	virtual void FlagUpdate();
};
