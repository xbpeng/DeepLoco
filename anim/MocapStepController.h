#pragma once

#include "anim/KinController.h"
#include "anim/Motion.h"
#include "sim/BipedStepController3D.h"

class cMocapStepController : public cKinController {
public:
	
	typedef cBipedStepController3D::eStance eStance;
	typedef cBipedStepController3D::tStepPlan tStepPlan;

	enum eMotionFeature
	{
		eMotionFeatureX0,
		eMotionFeatureZ0,
		eMotionFeatureX1,
		eMotionFeatureZ1,
		eMotionFeatureX2,
		eMotionFeatureZ2,
		eMotionFeatureHeading,
		eMotionFeatureMax
	};
	typedef Eigen::Matrix<double, eMotionFeatureMax, 1> tMotionFeatures;
	
	cMocapStepController();
	virtual ~cMocapStepController();

	virtual void Init(cKinCharacter* character, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual void SetTime(double time);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;
	virtual void SetCyclePeriod(double period);
	virtual void SetFootJoints(int right_end_id, int left_end_id);

	virtual void SetStepPlan(const tStepPlan& step_plan);
	virtual eStance GetStance() const;
	virtual eStance GetStance(double time) const;

	virtual void EnableAutoStepUpdate(bool enable);
	virtual void ForceStepUpdate();

protected:

	bool mClusterMotions;
	double mClusterDist;

	double mCyclePeriod;
	std::vector<cMotion> mMotions;
	std::vector<tMotionFeatures, Eigen::aligned_allocator<tMotionFeatures>> mMotionFeatures;

	tMotionFeatures mMotionFeatureWeights;
	int mCurrMotionID;

	int mLeftFootID;
	int mRightFootID;
	std::vector<int> mRightLegJoints;
	std::vector<int> mLeftLegJoints;

	eStance mPrevStance;
	tVector mOriginTrans;
	tQuaternion mOriginRot;

	tStepPlan mStepPlan;
	bool mEnableAutoStepUpdate;

	virtual void SetupMirrorStanceJoints();

	virtual bool LoadParams(const std::string& param_file);
	virtual bool LoadMotions(const Json::Value& json);
	virtual void LoadMotions(const std::vector<std::string>& motion_files);

	virtual void ResetParams();
	virtual void UpdateNewStep();

	virtual const cMotion& GetMotion(int m) const;
	virtual const cMotion& GetCurrMotion() const;
	virtual int GetNumMotions() const;
	virtual double CalcMotionPhase(double time) const;
	virtual double CalcStepPhase(double motion_phase) const;

	virtual void ApplyOriginTrans(Eigen::VectorXd& out_pose) const;

	virtual void CalcMotionPose(double time, int motion_id, Eigen::VectorXd& out_pose) const;
	virtual bool ShouldMirrorPose(double time) const;
	virtual void MirrorPoseStance(Eigen::VectorXd& out_pose) const;

	virtual void InitMotionFeatureWeights(tMotionFeatures& out_weights) const;
	virtual void UpdateMotionFeatureWeights(tMotionFeatures& out_weights) const;
	virtual void ExtractMotionFeatures();
	virtual void ExtractMotionFeatures(const cMotion& motion, tMotionFeatures& out_features) const;
	virtual void ExtractMotionFeatures(const cMotion& motion, double phase_beg, tMotionFeatures& out_features) const;
	virtual void ExtractMotionFeatures(const Eigen::VectorXd& pose_beg, const Eigen::VectorXd& pose_end, 
										tMotionFeatures& out_features) const;
	virtual int SelectNewMotion() const;
	virtual void BuildPoseMotionFeatures(tMotionFeatures& out_features) const;
	virtual void FindNearestMotion(const tMotionFeatures& pose_features, int& out_motion_id, double& out_min_dist) const;
	virtual const tMotionFeatures& GetPhaseMotionFeature(double phase, int motion_id) const;
	virtual double CalcMotionFeatureDist(const tMotionFeatures& feature0, const tMotionFeatures& feature1) const;
	
	virtual void ClusterMotions();
};
