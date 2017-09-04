#pragma once

#include "sim/Ground.h"
#include "sim/NNController.h"
#include "util/CircularBuffer.h"

class cTerrainRLCharController : public cNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tAction
	{
		int mID;
		double mLogp;
		Eigen::VectorXd mParams;

		tAction();
	};
	
	virtual ~cTerrainRLCharController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_step);
	virtual void UpdateCalcTau(double time_step, Eigen::VectorXd& out_tau) = 0;
	virtual void UpdateApplyTau(const Eigen::VectorXd& tau) = 0;

	virtual void SetGround(std::shared_ptr<cGround> ground);
	
	virtual int GetNumActions() const = 0;
	virtual int GetCurrActionID() const;
	virtual int GetNumParams() const = 0;

	virtual int GetNumGroundSamples() const;
    virtual void SetNumGroundSamples(int num_ground_samples);
	virtual tVector GetGroundSample(int s) const;
	virtual tMatrix GetGroundSampleTrans() const;
	virtual void GetViewBound(tVector& out_min, tVector& out_max) const;

	virtual const Eigen::VectorXd& GetTau() const;

	virtual bool IsOffPolicy() const;
	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliState(Eigen::VectorXd& out_state) const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const = 0;
	virtual void SkipDecideAction();
	virtual void GetPoliActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual double CalcActionLogp() const;
	
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const = 0;
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActionExpCovar(Eigen::VectorXd& out_covar) const;

	virtual const tAction& GetCurrAction() const;
	virtual void SampleAction(tAction& out_action);

protected:
	enum ePoliState
	{
		ePoliStateGround,
		ePoliStatePose,
		ePoliStateVel,
		ePoliStateMax
	};

	bool mFirstCycle;
	bool mIsOffPolicy;
	tAction mCurrAction;
	Eigen::VectorXd mPoliState;
	Eigen::VectorXd mTau;

    int mNumGroundSamples;
	tMatrix mGroundSampleTrans;
	std::shared_ptr<cGround> mGround;
	Eigen::VectorXd mGroundSamples;
	bool mSkipDecideAction;

	cTerrainRLCharController();

	virtual void ResetParams();
	virtual void InitPoliState();
	virtual void InitCurrAction();
	virtual void InitGroundSamples();

	virtual void ApplyAction(int action_id);
	virtual void ApplyAction(const tAction& action);
	virtual void NewCycleUpdate();
	virtual void PostProcessParams(Eigen::VectorXd& out_params) const;
	virtual void PostProcessAction(tAction& out_action) const;
	virtual void SetParams(const Eigen::VectorXd& params);
	virtual bool IsOptParam(int param_idx) const;

	virtual tMatrix BuildGroundSampleTrans() const;
	virtual void ParseGround();
	virtual bool HasGround() const;
	virtual void SampleGround(Eigen::VectorXd& out_samples) const;
	virtual tVector CalcGroundSamplePos(int s) const;
	virtual double SampleGroundHeight(const tVector& pos) const;
	virtual bool SampleGroundHeightVel(const tVector& pos, double& out_h, tVector& out_vel) const;

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildPoliStateGround(Eigen::VectorXd& out_ground) const;
	virtual void BuildPoliStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoliStateVel(Eigen::VectorXd& out_vel) const;
	virtual int GetPoliStateFeatureOffset(ePoliState feature) const;
	virtual int GetPoliStateFeatureSize(ePoliState feature) const;
	virtual int GetGroundFeatureSize() const;
	virtual int GetPoseFeatureSize() const;
	virtual int GetVelFeatureSize() const;

	virtual void DecideAction(tAction& out_action) = 0;
	virtual void ExploitPolicy(const Eigen::VectorXd& state, tAction& out_action) = 0;
	virtual void ExploreAction(Eigen::VectorXd& state, tAction& out_action) = 0;

	virtual void BuildDefaultAction(tAction& out_action) const;
	virtual void BuildBaseAction(int action_id, tAction& out_action) const = 0;
	virtual void BuildRandBaseAction(tAction& out_action) const;

	virtual void DebugPrintAction(const tAction& action) const;

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	const cCircularBuffer<double>& GetPoliValLog() const;
	virtual void GetVisCharacterFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisTerrainFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;

protected:
	cCircularBuffer<double> mPoliValLog;
	Eigen::VectorXd mVisNNOutput;
#endif // ENABLE_DEBUG_VISUALIZATION
};
