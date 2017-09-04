#include "OptScenarioSimChar.h"

cOptScenarioSimChar::cOptScenarioSimChar(cOptimizer& optimizer)
	: cOptScenario(optimizer)
{
}

cOptScenarioSimChar::~cOptScenarioSimChar()
{

}

void cOptScenarioSimChar::Init()
{
	cOptScenario::Init();
	BuildScenePool(mArgParser, mPoolSize);
	InitScenePool();
}

void cOptScenarioSimChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	mArgParser = parser;
	cOptScenario::ParseArgs(parser);
}

void cOptScenarioSimChar::Reset()
{
	cOptScenario::Reset();
	ResetScenePool();
}

bool cOptScenarioSimChar::LoadControlParams(const std::string& param_file)
{
	bool succ = true;
	for (size_t s = 0; s < mScenePool.size(); ++s)
	{
		succ &= mScenePool[s]->LoadControlParams(param_file);
	}
	return succ;
}

double cOptScenarioSimChar::EvalPt(const tPoint& pt)
{
	int idx = cIndexManager::gInvalidIndex;
	const auto& scene = RequestScene(idx);
	double obj_val = EvalPt(pt, scene);
	FreeScene(idx);

	return obj_val;
}

bool cOptScenarioSimChar::IsThreadSafe() const
{
	return false;
}

void cOptScenarioSimChar::SetPoolSize(int size)
{
	mPoolSize = size;
}

std::string cOptScenarioSimChar::GetName() const
{
	return "Optimize Sim Char";
}

void cOptScenarioSimChar::BuildScenePool(const std::shared_ptr<cArgParser>& parser, int pool_size)
{
	pool_size = std::max(1, pool_size);
	mScenePool.resize(pool_size);
	mPoolManager.Resize(pool_size);

	for (int i = 0; i < pool_size; ++i)
	{
		BuildScene(i, mScenePool[i]);
		mScenePool[i]->ParseArgs(parser);
	}
}

void cOptScenarioSimChar::UpdateScene(double time_step, const std::shared_ptr<cScenarioSimChar>& scene)
{
	scene->Update(gTimeStep);
}

void cOptScenarioSimChar::InitScenePool()
{
	for (size_t s = 0; s < mScenePool.size(); ++s)
	{
		const auto& curr_scene = mScenePool[s];
		curr_scene->Init();
	}
}

const std::shared_ptr<cScenarioSimChar>& cOptScenarioSimChar::GetDefaultScene() const
{
	return mScenePool[0];
}

void cOptScenarioSimChar::ResetScenePool()
{
	for (size_t s = 0; s < mScenePool.size(); ++s)
	{
		const auto& scene = mScenePool[s];
		scene->Reset();
	}
	mPoolManager.Reset();
}

const std::shared_ptr<cScenarioSimChar>& cOptScenarioSimChar::RequestScene(int& out_idx)
{
	out_idx = mPoolManager.RequestIndex();
	return mScenePool[out_idx];
}

void cOptScenarioSimChar::FreeScene(int idx)
{
	mPoolManager.FreeIndex(idx);
}