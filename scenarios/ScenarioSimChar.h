#pragma once

#include "scenarios/Scenario.h"
#include "sim/World.h"
#include "sim/SimBox.h"
#include "sim/SimPlane.h"
#include "sim/SimCapsule.h"
#include "sim/SimCharacter.h"
#include "sim/Perturb.h"
#include "sim/Ground.h"
#include "sim/TerrainGen2D.h"
#include "sim/TerrainRLCtrlFactory.h"

class cScenarioSimChar : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eCharType
	{
		eCharNone,
		cCharGeneral,
		eCharMax
	};

	struct tObjEntry
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		std::shared_ptr<cSimObj> mObj;
		double mEndTime;
		tVector mColor;

		tObjEntry();
	};

	cScenarioSimChar();
	virtual ~cScenarioSimChar();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	virtual const std::shared_ptr<cSimCharacter>& GetCharacter() const;
	virtual const std::shared_ptr<cWorld>& GetWorld() const;
	virtual tVector GetCharPos() const;
	virtual const std::shared_ptr<cGround>& GetGround() const;
	virtual const tVector& GetGravity() const;
	virtual bool LoadControlParams(const std::string& param_file);

	virtual void FilterPartIDs(std::vector<int>& out_ids) const;
	virtual void AddPerturb(const tPerturb& perturb);
	virtual void ApplyRandForce(double min_force, double max_force, 
								double min_dur, double max_dur, cSimObj* obj);
	virtual void ApplyRandForce();
	virtual int GetRandPerturbPartID();
	virtual void RayTest(const tVector& beg, const tVector& end, cWorld::tRayTestResult& out_result) const;

	virtual void SetGroundParamBlend(double lerp);
	virtual int GetNumParamSets() const;
	virtual double GetTime() const;
	virtual void OutputCharState(const std::string& out_file) const;
	virtual void OutputGround(const std::string& out_file) const;

	virtual bool HasFallen() const;
	virtual bool HasStumbled() const;
	virtual eCharType GetCharType() const;

	virtual void SpawnProjectile();
	virtual void SpawnBigProjectile();
	virtual const std::vector<tObjEntry, Eigen::aligned_allocator<tObjEntry>>& GetObjs() const;

	virtual void SetPreSubstepCallback(tTimeCallbackFunc func);
	virtual void SetPostSubstepCallback(tTimeCallbackFunc func);

	virtual bool HasRandSeed() const;
	virtual void SetRandSeed(unsigned long seed);
	virtual unsigned long GetRandSeed() const;
	virtual cWorld::eSimMode GetSimMode() const;

	virtual std::string GetName() const;

protected:

	static const double gGroundSpawnOffset;
	bool mHasRandSeed;
	unsigned long mRandSeed;

	cWorld::tParams mWorldParams;
	double mTime; // in seconds
	int mNumUpdateSteps;

	cSimCharacter::tParams mCharParams;
	std::string mCharCtrlParamFile;

	cRand mRand;

	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cGround> mGround;
	std::shared_ptr<cSimCharacter> mChar;
	eCharType mCharType;
	cTerrainRLCtrlFactory::tCtrlParams mCtrlParams;
	std::string mExpLayer; // mostly for action exploration

	cGround::tParams mGroundParams;

	bool mEnableRandPerturbs;
	double mRandPerturbTimer;
	double mPerturbTimeMin;
	double mPerturbTimeMax;
	double mRandPertubTime;
	double mMinPerturb;
	double mMaxPerturb;
	double mMinPerturbDuration;
	double mMaxPerturbDuration;
	std::vector<int> mPerturbPartIDs;

	std::vector<tObjEntry, Eigen::aligned_allocator<tObjEntry>> mObjs;
	tTimeCallbackFunc mPreSubstepCallback;
	tTimeCallbackFunc mPostSubstepCallback;

	std::string mOutMotionFile;

	virtual void BuildWorld();
	virtual bool BuildCharacter();
	virtual void BuildGround();
	virtual void SetupGround();
	virtual const std::string& GetCtrlParamFile() const;
	virtual void SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const;
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);

	virtual void CreateCharacter(eCharType char_type, std::shared_ptr<cSimCharacter>& out_char) const;
	virtual tVector GetDefaultCharPos() const;
	virtual void InitCharacterPos(const std::shared_ptr<cSimCharacter>& out_char);
	virtual void SetupCharRootPos(const std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const;

	virtual void UpdateWorld(double time_step);
	virtual void UpdateCharacter(double time_step);
	virtual void UpdateGround(double time_elapsed);
	virtual void UpdateRandPerturb(double time_step);

	virtual void ResetCharacters();
	virtual void ResetWorld();
	virtual void ResetGround();

	virtual void PreSubstepUpdate(double time_step);
	virtual void PostSubstepUpdate(double time_step);

	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;

	virtual void ParseCharType(const std::string& char_ctrl_str, eCharType& out_char_type) const;
	virtual void ParseGroundParams(const std::shared_ptr<cArgParser>& parser, cGround::tParams& out_params) const;

	virtual void UpdateObjs(double time_step);
	virtual void ClearObjs();
	virtual int AddObj(const tObjEntry& obj_entry);
	virtual void RemoveObj(int handle);
	virtual int GetNumObjs() const;

	virtual void SpawnProjectile(double density, double min_size, double max_size,
									double min_speed, double max_speed, double y_offset, double life_time);

	virtual void ResetRandPertrub();
};
