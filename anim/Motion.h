#pragma once

#include "util/json/json.h"
#include "KinTree.h"

class cMotion
{
public:
	enum eFrameParams
	{
		eFrameTime,
		eFrameMax
	};
	typedef Eigen::VectorXd tFrame;
	typedef std::function<tFrame(const Eigen::VectorXd*, const Eigen::VectorXd*, double)> tBlendFunc;

	static const std::string gFrameKey;
	static const std::string gLoopKey;
	static std::string BuildFrameJson(const Eigen::VectorXd& frame);

	cMotion();
	virtual ~cMotion();

	virtual void Clear();
	virtual bool Load(const std::string& file);
	virtual void Init(int num_frames, int num_dofs);
	virtual bool IsValid() const;

	virtual int GetNumFrames() const;
	virtual int GetNumDof() const;
	virtual tFrame GetFrame(int i) const;
	virtual void SetFrame(int i, const tFrame& frame);
	virtual void SetFrameTime(int i, double time);
	virtual tFrame BlendFrames(int a, int b, double lerp) const;

	virtual tFrame CalcFrame(double time) const;
	virtual tFrame CalcFramePhase(double phase) const;
	virtual double GetDuration() const;
	virtual double GetFrameTime(int f) const;

	virtual int CalcCycleCount(double time) const;
	virtual void CalcIndexPhase(double time, int& out_idx, double& out_phase) const;

	virtual bool IsOver(double time) const;

	virtual void SetBlendFunc(tBlendFunc blend_func);
	virtual bool HasBlendFunc() const;
	virtual bool IsLoop() const;

	virtual void Output(const std::string& out_filepath) const;

protected:
	bool mLoop;
	Eigen::MatrixXd mFrames;
	tBlendFunc mBlendFunc;

	virtual bool LoadJson(const Json::Value& root);
	virtual bool ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const;

	virtual void PostProcessFrames(Eigen::MatrixXd& frames) const;
	virtual int GetFrameSize() const;

	virtual tFrame BlendFramesIntern(const cMotion::tFrame* a, const cMotion::tFrame* b, double lerp) const;
};
