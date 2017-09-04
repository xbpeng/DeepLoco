#pragma once

#include "scenarios/ScenarioExpImitate.h"
#include "sim/Ground.h"

class cScenarioExpImitateTarget : virtual public cScenarioExpImitate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExpImitateTarget();
	virtual ~cScenarioExpImitateTarget();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Update(double time_elapsed);

	virtual const tVector& GetTargetPos() const;
	virtual void SetTargetPos(const tVector& target_pos);
	virtual void EnableRandTargetPos(bool enable);
	virtual bool EnabledRandTargetPos() const;
	virtual double GetTargetResetDist() const;

	virtual std::string GetName() const;

protected:

	bool mEnableRandCharPlacement;
	bool mEnableRandTargetPos;
	double mRandTargetPosTimeMin;
	double mRandTargetPosTimeMax;
	double mRandTargetPosTimer;
	double mTargetResetDist;
	tVector mTargetPos;

	virtual bool CheckResetTarget() const;
	virtual void InitCharacterPos(const std::shared_ptr<cSimCharacter>& out_char);
	virtual void SetCharRandPlacement(const std::shared_ptr<cSimCharacter>& out_char);
	virtual void CalcCharRandPlacement(tVector& out_pos, tQuaternion& out_rot);
	virtual void CalcCharRandPlacementTrail3D(tVector& out_pos, tQuaternion& out_rot);
	virtual void CalcCharRandPlacementObstacles3D(tVector& out_pos, tQuaternion& out_rot);
	virtual void CalcCharRandPlacementDynamicObstacles3D(tVector& out_pos, tQuaternion& out_rot);
	virtual void CalcCharRandPlacementDefault(tVector& out_pos, tQuaternion& out_rot);

	virtual double CalcReward() const;
	virtual double CalcTargetReward(const tVector& tar_pos) const;
	virtual void UpdateTargetController();
	virtual void UpdateTargetPos(double time_elapsed);

	virtual double GetRandTargetMaxDist() const;
	virtual void ResetTargetPos();
	virtual tVector CalcTargetPosTrail3D();
	virtual tVector CalcTargetPosObstacle3D();
	virtual tVector CalcTargetPosDynamicObstacle3D();
	virtual tVector CalcTargetPosDefault();

	virtual int GetTargetPosTrail3dForwardSegs() const;
};