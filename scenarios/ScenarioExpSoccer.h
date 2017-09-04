#pragma once

#include "scenarios/ScenarioExpHike.h"

class cScenarioExpSoccer : virtual public cScenarioExpHike
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpSoccer();
	virtual ~cScenarioExpSoccer();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();

	virtual void Update(double time_elapsed);
	virtual void SetBallPos(const tVector& pos);
	virtual int GetNumBalls() const;
	virtual void RemoveObj(int handle);

	virtual std::string GetName() const;

protected:

	std::vector<int> mBallObjHandles;
	int mNumBallSpawns;
	bool mRemoveBallAtGoal;

	tVector mPrevBallPos;

	double mRandBallPosTimeMin;
	double mRandBallPosTimeMax;
	double mRandBallPosTimer;
	
	virtual void ResetParams();
	virtual bool CheckResetTarget() const;
	virtual void ClearObjs();
	
	virtual double CalcReward() const;
	virtual double GetRandTargetMaxDist() const;
	virtual double GetRandBallMaxDist() const;
	virtual void HandleNewActionUpdate();

	virtual bool EndEpisode() const;

	virtual void BuildBalls();
	virtual int BuildBall();
	virtual const std::shared_ptr<cSimObj>& GetBall() const;
	virtual const std::shared_ptr<cSimObj>& GetBall(int ball_handle) const;
	virtual tVector GetBallPos() const;
	virtual tVector GetBallPos(int ball_handle) const;

	virtual tVector CalcTargetPosDefault();
	virtual void UpdateBallPos(double time_elapsed);
	virtual void ResetBallPosAll();
	virtual void ResetBallPos(int ball_handle);
	virtual void SetBallPos(int ball_handle, const tVector& pos);
	virtual void ResetBallTimer();

	virtual int GetTargetBallHandle() const;
	virtual void UpdateTargetBall();
	virtual int FindNearestBall(const tVector& pos) const;
};