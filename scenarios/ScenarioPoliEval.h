#pragma once

#include "scenarios/ScenarioExp.h"

class cScenarioPoliEval : virtual public cScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioPoliEval();
	virtual ~cScenarioPoliEval();

	virtual void ResetRecord();

	virtual double GetAvgDist() const;
	virtual double CalcAvgReward() const;
	virtual void ResetAvgDist();
	virtual int GetNumEpisodes() const;
	virtual int GetNumTotalCycles() const;
	virtual const std::vector<double>& GetDistLog() const;
	virtual const std::vector<double>& GetRewardLog() const;

	virtual bool EnableRecordReward();
	virtual void EnableRecordActions(bool enable);
	virtual bool EnableRecordActions() const;
	virtual void EnableRecordCtrlForce(bool enable);
	virtual bool EnableRecordCtrlForce() const;
	virtual void EndEpisodeRecord();

	virtual double GetCurrCumulativeReward() const;
	virtual double CalcAvgCumulativeReward() const;

	virtual void SetPoliModelFile(const std::string& model_file);
	virtual void SetCriticModelFile(const std::string& model_file);

	virtual std::string GetName() const;

protected:
	tVector mPosStart;
	
	double mAvgDist;
	int mEpisodeCount;
	int mTotalCycles;
	std::vector<double> mDistLog;
	std::vector<double> mRewardLog;

	// analysis stuff
	bool mRecordNNActivation;
	std::string mNNActivationOutputFile;
	std::string mNNActivationLayer;

	bool mRecordActions;
	std::string mActionOutputFile;

	bool mRecordCtrlForce;
	std::string mCtrlForceOutputFile;

	bool mRecordActionIDState;
	std::string mActionIDStateOutputFile;

	bool mRecordReward;
	double mTotalReward;
	
	virtual void RecordDistTraveled();
	virtual double CalcDistTraveled() const;
	virtual double CalcDistTraveledDefault() const;
	virtual double CalcDistTraveledTrail3D() const;

	virtual bool EnableRandInitAction() const;
	virtual bool IsValidCycle() const;
	
	virtual void InitNNActivation(const std::string& out_file);
	virtual bool EnableRecordNNActivation() const;
	virtual void RecordNNActivation(const std::string& layer_name, const std::string& out_file);

	virtual void InitActionRecord(const std::string& out_file) const;
	virtual void RecordAction(const std::string& out_file);

	virtual void InitCtrlForceRecord(const std::string& out_file) const;
	virtual void RecordCtrlForce(const std::string& out_file);

	virtual void InitActionIDState(const std::string& out_file) const;
	virtual bool EnableRecordActionIDState() const;
	virtual void RecordActionIDState(const std::string& out_file);

	virtual void UpdateTotalReward(double curr_reward);
	virtual void RecordTotalReward();

	virtual void ParseMiscArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void InitMisc();
	virtual void ClearMisc();
	virtual void ResetMisc();
	virtual void ResetMiscParams();
	virtual void PostSubstepUpdateMisc(double time_step);
	virtual void HandleEpisodeEnd();
	virtual void UpdateMiscRecord();
};