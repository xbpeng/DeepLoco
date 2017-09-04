#pragma once

#include "scenarios/OptScenario.h"
#include "scenarios/ScenarioSimChar.h"
#include "util/IndexManager.h"

class cOptScenarioSimChar : public  cOptScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cOptScenarioSimChar(cOptimizer& optimizer);
	virtual ~cOptScenarioSimChar();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();

	virtual bool LoadControlParams(const std::string& param_file);

	virtual int GetDimensions() const = 0;
	virtual void InitPt(tPoint& pt) const = 0;
	virtual void InitScale(tPoint& scale) const = 0;
	virtual double EvalPt(const tPoint& pt);

	virtual bool IsThreadSafe() const;

	virtual void OutputPt(FILE*, const tPoint& pt) const = 0;
	virtual void SetPoolSize(int size);

	virtual std::string GetName() const;

protected:
	std::shared_ptr<cArgParser> mArgParser;
	std::vector<std::shared_ptr<cScenarioSimChar>> mScenePool;
	int mPoolSize;
	cIndexManagerMT mPoolManager;

	virtual void BuildScenePool(const std::shared_ptr<cArgParser>& parser, int pool_size);
	virtual void BuildScene(int id, std::shared_ptr<cScenarioSimChar>& out_scene) = 0;

	virtual void UpdateScene(double time_step, const std::shared_ptr<cScenarioSimChar>& scene);
	
	virtual void InitScenePool();
	virtual const std::shared_ptr<cScenarioSimChar>& GetDefaultScene() const;
	virtual void ResetScenePool();
	virtual const std::shared_ptr<cScenarioSimChar>& RequestScene(int& out_idx);
	virtual void FreeScene(int idx);

	virtual double EvalPt(const tPoint& pt, const std::shared_ptr<cScenarioSimChar>& scene) = 0;
};