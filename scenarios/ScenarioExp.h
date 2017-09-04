#pragma once

#include "scenarios/ScenarioSimChar.h"
#include "learning/ExpTuple.h"
#include "sim/SimCharacter.h"
#include "sim/NNController.h"

class cScenarioExp : virtual public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eTimeLimType
	{
		eTimeLimUniform,
		eTimeLimExp,
		eTimeLimMax
	};

	cScenarioExp();
	virtual ~cScenarioExp();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual void SetBufferSize(int size);
	virtual bool IsTupleBufferFull() const;
	virtual void ResetTupleBuffer();
	virtual const std::vector<tExpTuple>& GetTuples() const;

	virtual void EnableExplore(bool enable);
	virtual const cCharController::tExpParams& GetExpParams() const;
	virtual void SetExpParams(const cCharController::tExpParams& params);
	virtual void SetCurriculumPhase(double phase);

	virtual double GetEpisodeMaxTime() const;

	virtual void SetEpisodeTimeLimType(eTimeLimType lim_type);
	virtual void SetEpisodeTimeLim(double lim_min, double lim_max);
	virtual void GetEpisodeTimeLim(double& out_min, double& out_max) const;
	virtual void SetEpisodeTimeLimExpLambda(double lambda);
	virtual double GetEpisodeTimeLimExpLambda(double lambda) const;

	virtual void SetPoliModelFile(const std::string& model_file);
	virtual void SetCriticModelFile(const std::string& model_file);
	virtual std::vector<std::string>& GetNetFiles();

	virtual std::string GetName() const;

protected:

	double mCurriculumPhase;
	bool mEnableExplore;
	cCharController::tExpParams mExpParams;

	bool mEnableFallReset;
	bool mEnableRandEpisodeTimeLim;
	eTimeLimType mEpisodeTimeLimType;
	double mEpisodeTimeLimMin;
	double mEpisodeTimeLimMax;
	double mEpisodeTimeLimExpLambda;
	double mEpisodeMaxTime;

	tVector mPrevCOM;
	double mPrevTime;
	int mCycleCount;

	int mTupleBufferSize;
	int mTupleCount;
	tExpTuple mCurrTuple;
	std::vector<tExpTuple> mTupleBuffer;

	virtual void ResetParams();

	virtual bool NewActionUpdate() const;
	virtual void PostSubstepUpdate(double time_step);
	virtual void HandleNewActionUpdate();
	virtual void HandleFallUpdate();
	virtual void IncCycleCount();

	virtual void ClearReward(tExpTuple& out_tuple) const;
	virtual void UpdateRewardBeg(tExpTuple& out_tuple) const;
	virtual void UpdateRewardSubstep(double time_step, tExpTuple& out_tuple) const;
	virtual void UpdateRewardEnd(tExpTuple& out_tuple) const;

	virtual std::shared_ptr<const cNNController> GetNNController() const;
	virtual std::shared_ptr<cNNController> GetNNController();
	
	virtual bool EnableEvalRecord() const;
	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual double GetActionLogp() const;

	virtual bool CheckFail() const;

	virtual void ClearFlags(tExpTuple& out_tuple) const;
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	virtual void RecordFlagsEnd(tExpTuple& out_tuple) const;
	virtual void RecordTuple(const tExpTuple& tuple);

	virtual bool EnableRandInitAction() const;
	virtual void CommandRandAction();
	virtual bool IsFirstValidCycle() const;
	virtual bool IsValidTuple(const tExpTuple& tuple) const;
	virtual double CalcReward() const;

	virtual void SetRandTimeLim();
	virtual void ParseTimeLimType(const std::string& str, eTimeLimType& out_type) const;

	virtual bool IsValidCycle() const;
	virtual int GetNumWarmupCycles() const;
	virtual bool EndEpisode() const;
	virtual bool EnableRandTimeLim() const;
	virtual bool CheckEpisodeTimeLimit() const;

	// hooks used mostly for poli eval, exp scenes should not use these
	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void InitMisc();
	virtual void ClearMisc();
	virtual void ResetMisc();
	virtual void ResetMiscParams();
	virtual void UpdateMisc(double time_step);
	virtual void PostSubstepUpdateMisc(double time_step);
	virtual void HandleEpisodeEnd();
	virtual void UpdateMiscRecord();
};