#pragma once

#include "sim/GroundPlane.h"
#include "sim/SimObj.h"
#include "sim/SimCharacter.h"

class cGroundDynamicObstacles3D : public cGroundPlane
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundDynamicObstacles3D();
	virtual ~cGroundDynamicObstacles3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
	virtual void Clear();
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);

	virtual int GetNumObstacles() const;
	virtual const cSimObj& GetObj(int i) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;

	virtual tVector FindRandFlatPos(const tVector& buffer_size);
	virtual void SetChar(const std::shared_ptr<cSimCharacter>& character);

	virtual eClass GetGroundClass() const;

protected:
	
	struct tObstacle
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		enum eDir
		{
			eDirForward,
			eDirBackward,
			eDirMax
		};

		enum eMove
		{
			eMovePingPong,
			eMoveForward,
			eMoveMax
		};

		std::shared_ptr<cSimObj> mObj;
		tVector mPosStart;
		tVector mPosEnd;
		double mSpeed;
		double mPhase;
		eDir mDir;
		eMove mMoveType;

		tObstacle();
		virtual ~tObstacle();

		virtual void Update(double time_elapsed, const tVector& char_pos);
		virtual tVector CalcPos() const;
		virtual tVector CalcVel() const;
		virtual double CalcCycleDur() const;
	};

	std::shared_ptr<cSimCharacter> mChar;
	std::vector<tObstacle, Eigen::aligned_allocator<tObstacle>> mObstacles;

	virtual int GetBlendParamSize() const;
	virtual tVector GetCharPos() const;

	virtual void BuildObstacles();
	virtual void ClearObstacles();
	virtual void SortObstacles();
	virtual void BuildObstacle(tObstacle& out_obstacle);
	virtual void UpdateObstacles(double time_elapsed);

	virtual double FindMaxBoundHeight(const tVector& aabb_min, const tVector& aabb_max) const;
};