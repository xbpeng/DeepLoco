#include "ScenarioSimChar.h"

#include <memory>
#include <ctime>
#include "sim/SimCharGeneral.h"
#include "sim/GroundPlane.h"
#include "sim/GroundVar2D.h"
#include "sim/GroundVar3D.h"
#include "sim/GroundDynamicObstacles3D.h"
#include "sim/GroundFactory.h"

#include "util/FileUtil.h"

//#define SIM_CHAR_PROFILER

const tVector gLineColor = tVector(0, 0, 0, 1);
const double gCharViewDistPad = 1;
// hack get rid of this or the one in ground factory, dont keep both
const double cScenarioSimChar::gGroundSpawnOffset = -1; // some padding to prevent parts of character from getting spawned inside obstacles

const std::string gCharName[cScenarioSimChar::eCharMax] =
{
	"none",
	"general",
};

cScenarioSimChar::tObjEntry::tObjEntry()
{
	mObj = nullptr;
	mEndTime = std::numeric_limits<double>::infinity();
	mColor = tVector(0.5, 0.5, 0.5, 1);
}

cScenarioSimChar::cScenarioSimChar()
{
	mRand.Seed(cMathUtil::RandUint());
	mRandSeed = 0;
	mHasRandSeed = false;

	mWorldParams.mSimMode = cWorld::eSimMode2D;
	mWorldParams.mNumSubsteps = 1;
	mWorldParams.mScale = 1;
	mWorldParams.mGravity = gGravity;

	mTime = 0;
	mNumUpdateSteps = 20;
	
	mCharType = eCharNone;
	mExpLayer = "";

	mEnableRandPerturbs = false;
	mRandPerturbTimer = 0;
	mPerturbTimeMin = std::numeric_limits<double>::infinity();
	mPerturbTimeMax = std::numeric_limits<double>::infinity();
	mRandPertubTime = 0;
	mMinPerturb = 50;
	mMaxPerturb = 100;
	mMinPerturbDuration = 0.1;
	mMaxPerturbDuration = 0.5;

	mPreSubstepCallback = nullptr;
	mPostSubstepCallback = nullptr;

	mOutMotionFile = "output/record_motion.txt";
}

cScenarioSimChar::~cScenarioSimChar()
{

}

void cScenarioSimChar::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	bool succ = true;
	succ = parser->ParseString("character_file", mCharParams.mCharFile);
	if (!succ)
	{
		printf("No character file specified.\n");
	}

	int rand_seed = 0;
	mHasRandSeed = parser->ParseInt("rand_seed", rand_seed);
	mRandSeed = static_cast<unsigned long>(rand_seed);

	std::string sim_mode_str = "";
	parser->ParseString("sim_mode", sim_mode_str);
	cWorld::ParseSimMode(sim_mode_str, mWorldParams.mSimMode);
	parser->ParseInt("num_sim_substeps", mWorldParams.mNumSubsteps);
	parser->ParseDouble("world_scale", mWorldParams.mScale);

	parser->ParseString("state_file", mCharParams.mStateFile);
	parser->ParseBool("enable_char_fall_dist", mCharParams.mEnableFallDist);
	parser->ParseBool("enable_char_soft_contact", mCharParams.mEnableSoftContact);

	parser->ParseBool("enable_rand_perturbs", mEnableRandPerturbs);
	parser->ParseDouble("perturb_time_min", mPerturbTimeMin);
	parser->ParseDouble("perturb_time_max", mPerturbTimeMax);
	parser->ParseDouble("min_perturb", mMinPerturb);
	parser->ParseDouble("max_perturb", mMaxPerturb);
	parser->ParseDouble("min_pertrub_duration", mMinPerturbDuration);
	parser->ParseDouble("max_perturb_duration", mMaxPerturbDuration);

	mPerturbPartIDs.clear();
	parser->ParseIntArray("perturb_part_ids", mPerturbPartIDs);

	parser->ParseInt("num_update_steps", mNumUpdateSteps);
	parser->ParseString("exp_layer", mExpLayer);

	std::string char_type_str = "";
	parser->ParseString("char_type", char_type_str);
	ParseCharType(char_type_str, mCharType);

	std::string char_ctrl_str = "";
	parser->ParseString("char_ctrl", char_ctrl_str);

	parser->ParseString("char_ctrl_param_file", mCharCtrlParamFile);
	cTerrainRLCtrlFactory::ParseCharCtrl(char_ctrl_str, mCtrlParams.mCharCtrl);
	parser->ParseDouble("char_ctrl_ct_query_rate", mCtrlParams.mCtQueryRate);
	parser->ParseDouble("char_ctrl_cycle_dur", mCtrlParams.mCycleDur);
    parser->ParseInt("char_ctrl_num_ground_samples", mCtrlParams.mNumGroundSamples);
	parser->ParseInt("char_ctrl_ground_sample_res_3d", mCtrlParams.mGroundSampleRes3d);
	parser->ParseInt("char_ctrl_ground_view_dist", mCtrlParams.mViewDist);
	parser->ParseDouble("char_ctrl_waypoint_init_step_len", mCtrlParams.mWaypointInitStepLen);

	parser->ParseString("policy_net", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor]);
	parser->ParseString("critic_net", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic]);
	parser->ParseString("policy_net1", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1]);
	parser->ParseString("critic_net1", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1]);
	parser->ParseString("policy_model", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorModel]);
	parser->ParseString("critic_model", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticModel]);
	parser->ParseString("policy_model1", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActor1Model]);
	parser->ParseString("critic_model1", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCritic1Model]);
	parser->ParseString("policy_model_terrain", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorTerrainModel]);
	parser->ParseString("policy_model_action", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileActorActionModel]);
	parser->ParseString("policy_model_terrain", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticTerrainModel]);
	parser->ParseString("policy_model_value", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileCriticValueModel]);

	parser->ParseString("forward_dynamics_net", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamics]);
	parser->ParseString("forward_dynamics_solver", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsSolver]);
	parser->ParseString("forward_dynamics_model", mCtrlParams.mNetFiles[cTerrainRLCtrlFactory::eNetFileForwardDynamicsModel]);

	ParseGroundParams(parser, mGroundParams);

	parser->ParseDouble("char_init_pos_x", mCharParams.mInitPos[0]);

	parser->ParseString("out_motion_file", mOutMotionFile);
}

void cScenarioSimChar::Init()
{
	mTime = 0;

	if (HasRandSeed())
	{
		SetRandSeed(mRandSeed);
	}

	if (mEnableRandPerturbs)
	{
		ResetRandPertrub();
	}

	BuildWorld();
	BuildGround();
	BuildCharacter();
	SetupGround();

	InitCharacterPos(mChar);
	ResolveCharGroundIntersect(mChar);

	ClearObjs();
	FilterPartIDs(mPerturbPartIDs);
}

void cScenarioSimChar::Reset()
{
	mTime = 0;

	if (mEnableRandPerturbs)
	{
		ResetRandPertrub();
	}

	ResetCharacters();
	ResetWorld();
	ResetGround();
	ClearObjs();

	InitCharacterPos(mChar);
	ResolveCharGroundIntersect(mChar);

	if (mResetCallback != nullptr)
	{
		mResetCallback();
	}
}

void cScenarioSimChar::Clear()
{
	mChar->Clear();
	mChar.reset();
	mGround.reset();
	ClearObjs();
}

void cScenarioSimChar::Update(double time_elapsed)
{
#if defined(SIM_CHAR_PROFILER)
	static int time_count = 0;
	static double avg_time = 0;
	std::clock_t total_time_beg = std::clock();
#endif
	if (time_elapsed <= 0)
	{
		return;
	}

	if (mEnableRandPerturbs)
	{
		UpdateRandPerturb(time_elapsed);
	}

	double prev_time = mTime;

#if defined(ENABLE_TRAINING)
	mChar->ClearEffortBuffer();
#endif
	
	double update_step = time_elapsed / mNumUpdateSteps;
	int num_update_steps = (time_elapsed == 0) ? 1 : mNumUpdateSteps;
	for (int i = 0; i < num_update_steps; ++i)
	{  
		mTime += update_step;
		PreSubstepUpdate(update_step);

		// order matters!
		UpdateWorld(update_step);
		UpdateGround(update_step);
		UpdateCharacter(update_step);
		UpdateObjs(update_step);

		PostSubstepUpdate(update_step);
	}

#if defined(SIM_CHAR_PROFILER)
	std::clock_t total_time_end = std::clock();
	double delta_time = static_cast<double>(total_time_end - total_time_beg) / CLOCKS_PER_SEC;
	++time_count;
	avg_time = avg_time * ((time_count - 1.0) / time_count) + delta_time / time_count;
	printf("Sim Char Update Time: %.8f, count: %i\n", avg_time, time_count);
#endif
}

const std::shared_ptr<cSimCharacter>& cScenarioSimChar::GetCharacter()  const
{
	return mChar;
}

const std::shared_ptr<cWorld>& cScenarioSimChar::GetWorld() const
{
	return mWorld;
}

tVector cScenarioSimChar::GetCharPos() const
{
	return GetCharacter()->GetRootPos();
}

const std::shared_ptr<cGround>& cScenarioSimChar::GetGround() const
{
	return mGround;
}

const tVector& cScenarioSimChar::GetGravity() const
{
	return mWorldParams.mGravity;
}

bool cScenarioSimChar::LoadControlParams(const std::string& param_file)
{
	const auto& ctrl = mChar->GetController();
	bool succ = ctrl->LoadParams(param_file);
	return succ;
}

void cScenarioSimChar::FilterPartIDs(std::vector<int>& out_ids) const
{
	int num_ids = static_cast<int>(out_ids.size());
	int num_valid_ids = 0;
	int num_parts = mChar->GetNumBodyParts();
	for (int i = 0; i < num_ids; ++i)
	{
		int curr_id = out_ids[i];
		if (curr_id >= 0 && curr_id < num_parts)
		{
			bool valid_part = mChar->IsValidBodyPart(curr_id);
			if (valid_part)
			{
				out_ids[num_valid_ids] = curr_id;
				++num_valid_ids;
			}
		}
	}
	out_ids.resize(num_valid_ids);
}

void cScenarioSimChar::AddPerturb(const tPerturb& perturb)
{
	mWorld->AddPerturb(perturb);
}

void cScenarioSimChar::ApplyRandForce(double min_force, double max_force, 
									double min_dur, double max_dur, cSimObj* obj)
{
	assert(obj != nullptr);
	tPerturb perturb = tPerturb::BuildForce();
	perturb.mObj = obj;
	perturb.mLocalPos.setZero();
	perturb.mPerturb[0] = mRand.RandDouble(-1, 1);
	perturb.mPerturb[1] = mRand.RandDouble(-1, 1);
	perturb.mPerturb[2] = mRand.RandDouble(-1, 1);
	perturb.mPerturb = mRand.RandDouble(min_force, max_force) * perturb.mPerturb.normalized();
	perturb.mDuration = mRand.RandDouble(min_dur, max_dur);

	AddPerturb(perturb);
}

void cScenarioSimChar::ApplyRandForce()
{
	int num_parts = mChar->GetNumBodyParts();
	int part_idx = GetRandPerturbPartID();
	assert(part_idx != gInvalidIdx);
	const auto& part = mChar->GetBodyPart(part_idx);
	ApplyRandForce(mMinPerturb, mMaxPerturb, mMinPerturbDuration, mMaxPerturbDuration, part.get());
}

int cScenarioSimChar::GetRandPerturbPartID()
{
	int rand_id = gInvalidIdx;
	int num_part_ids = static_cast<int>(mPerturbPartIDs.size());
	if (num_part_ids > 0)
	{
		int idx = mRand.RandInt(0, num_part_ids);
		rand_id = mPerturbPartIDs[idx];
	}
	else
	{
		int num_parts = mChar->GetNumBodyParts();
		rand_id = mRand.RandInt(0, num_parts);
	}
	return rand_id;
}

void cScenarioSimChar::RayTest(const tVector& beg, const tVector& end, cWorld::tRayTestResult& out_result) const
{
	cWorld::tRayTestResults results;
	mWorld->RayTest(beg, end, results);

	out_result.mObj = nullptr;
	if (results.size() > 0)
	{
		out_result = results[0];
	}
}

void cScenarioSimChar::SetGroundParamBlend(double lerp)
{
	mGround->SetParamBlend(lerp);
}

int cScenarioSimChar::GetNumParamSets() const
{
	return static_cast<int>(mGroundParams.mParamArr.rows());
}

double cScenarioSimChar::GetTime() const
{
	return mTime;
}

void cScenarioSimChar::OutputCharState(const std::string& out_file) const
{
	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tMatrix trans = mChar->BuildOriginTrans();
	trans(1, 3) -= ground_h;

	mChar->WriteState(out_file, trans);
}

void cScenarioSimChar::OutputGround(const std::string& out_file) const
{
	mGround->Output(out_file);
}

bool cScenarioSimChar::HasFallen() const
{
	return mChar->HasFallen();
}

bool cScenarioSimChar::HasStumbled() const
{
	return mChar->HasStumbled();
}

cScenarioSimChar::eCharType cScenarioSimChar::GetCharType() const
{
	return mCharType;
}

std::string cScenarioSimChar::GetName() const
{
	return "Sim Character";
}

bool cScenarioSimChar::BuildCharacter()
{
	CreateCharacter(mCharType, mChar);

	bool succ = mChar->Init(mWorld, mCharParams);
	if (succ)
	{
		mChar->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);

		SetupCharRootPos(mChar);
		//InitCharacterPos(mChar);

		std::shared_ptr<cCharController> ctrl;
		succ = BuildController(ctrl);
		if (succ && ctrl != nullptr)
		{
			mChar->SetController(ctrl);
		}
	}
	return succ;
}

void cScenarioSimChar::BuildWorld()
{
	mWorld = std::shared_ptr<cWorld>(new cWorld());
	mWorld->Init(mWorldParams);
}

void cScenarioSimChar::BuildGround()
{
	double char_view_dist = 10;
	mGroundParams.mGroundWidth = 2 * char_view_dist;
	mGroundParams.mHasRandSeed = mHasRandSeed;
	mGroundParams.mRandSeed = mRandSeed;

	cGroundFactory::BuildGround(mWorld, mGroundParams, mGround);
}

void cScenarioSimChar::SetupGround()
{
	auto dynamic_obstacles = std::dynamic_pointer_cast<cGroundDynamicObstacles3D>(mGround);
	if (dynamic_obstacles != nullptr)
	{
		dynamic_obstacles->SetChar(mChar);
	}
}

const std::string& cScenarioSimChar::GetCtrlParamFile() const
{
	const std::string& file = (mCharCtrlParamFile == "") ? mCharParams.mCharFile : mCharCtrlParamFile;
	return file;
}

void cScenarioSimChar::SetupControllerParams(cTerrainRLCtrlFactory::tCtrlParams& out_params) const
{
	out_params.mCtrlParamFile = GetCtrlParamFile();
	out_params.mChar = mChar;
	out_params.mGravity = mWorldParams.mGravity;
	out_params.mGround = mGround;
}

bool cScenarioSimChar::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	SetupControllerParams(mCtrlParams);
	bool succ = cTerrainRLCtrlFactory::BuildController(mCtrlParams, out_ctrl);
	return succ;
}

void cScenarioSimChar::CreateCharacter(eCharType char_type, std::shared_ptr<cSimCharacter>& out_char) const
{
	if (char_type == cCharGeneral)
	{
		out_char = std::shared_ptr<cSimCharGeneral>(new cSimCharGeneral());
	}
	else
	{
		printf("No valid character specified\n");
		assert(false);
	}
}

tVector cScenarioSimChar::GetDefaultCharPos() const
{
	return mCharParams.mInitPos;
}

void cScenarioSimChar::InitCharacterPos(const std::shared_ptr<cSimCharacter>& out_char)
{
	tVector root_pos = out_char->GetRootPos();
	root_pos[0] = mCharParams.mInitPos[0];

	double h = mGround->SampleHeight(root_pos);
	root_pos[1] += h;
	out_char->SetRootPos(root_pos);
}

void cScenarioSimChar::SetupCharRootPos(const std::shared_ptr<cSimCharacter>& out_char) const
{
	tVector root_pos = out_char->GetRootPos();
	root_pos[0] = mCharParams.mInitPos[0];
	out_char->SetRootPos(root_pos);
	out_char->SetRootPos0(root_pos);
}

void cScenarioSimChar::ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const
{
	const double pad = 0.001;

	int num_parts = out_char->GetNumBodyParts();
	double min_violation = 0;
	for (int b = 0; b < num_parts; ++b)
	{
		if (out_char->IsValidBodyPart(b))
		{
			tVector aabb_min;
			tVector aabb_max;
			const auto& part = out_char->GetBodyPart(b);
			part->CalcAABB(aabb_min, aabb_max);

			tVector mid = 0.5 * (aabb_min + aabb_max);
			tVector sw = tVector(aabb_min[0], 0, aabb_min[2], 0);
			tVector nw = tVector(aabb_min[0], 0, aabb_max[2], 0);
			tVector ne = tVector(aabb_max[0], 0, aabb_max[2], 0);
			tVector se = tVector(aabb_max[0], 0, aabb_min[2], 0);

			double max_ground_height = 0;
			max_ground_height = mGround->SampleHeight(aabb_min);
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(mid));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(sw));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(nw));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(ne));
			max_ground_height = std::max(max_ground_height, mGround->SampleHeight(se));
			max_ground_height += pad;

			double min_height = aabb_min[1];
			min_violation = std::min(min_violation, min_height - max_ground_height);
		}
	}

	if (min_violation < 0)
	{
		tVector root_pos = out_char->GetRootPos();
		root_pos[1] += -min_violation;
		out_char->SetRootPos(root_pos);
	}
}

void cScenarioSimChar::UpdateWorld(double time_step)
{
	mWorld->Update(time_step);
}

void cScenarioSimChar::UpdateCharacter(double time_step)
{
	mChar->Update(time_step);
}

void cScenarioSimChar::UpdateGround(double time_elapsed)
{
	tVector view_min;
	tVector view_max;
	GetViewBound(view_min, view_max);
	mGround->Update(time_elapsed, view_min, view_max);
}

void cScenarioSimChar::UpdateRandPerturb(double time_step)
{
	mRandPerturbTimer += time_step;
	if (mRandPerturbTimer >= mRandPertubTime)
	{
		ApplyRandForce();
		ResetRandPertrub();
	}
}

void cScenarioSimChar::ResetCharacters()
{
	mChar->Reset();
}

void cScenarioSimChar::ResetWorld()
{
	mWorld->Reset();
}

void cScenarioSimChar::ResetGround()
{
	mGround->Clear();

	tVector view_min;
	tVector view_max;
	GetViewBound(view_min, view_max);
	view_min [0] += gGroundSpawnOffset;
	view_max[0] += gGroundSpawnOffset;
	view_min[2] += gGroundSpawnOffset;
	view_max[2] += gGroundSpawnOffset;

	mGround->Update(0, view_min, view_max);
}

void cScenarioSimChar::PreSubstepUpdate(double time_step)
{
	if (mPreSubstepCallback != nullptr)
	{
		mPreSubstepCallback(time_step);
	}
}

void cScenarioSimChar::PostSubstepUpdate(double time_step)
{
	if (mPostSubstepCallback != nullptr)
	{
		mPostSubstepCallback(time_step);
	}
}

void cScenarioSimChar::GetViewBound(tVector& out_min, tVector& out_max) const
{
	const std::shared_ptr<cSimCharacter>& character = GetCharacter();
	const std::shared_ptr<cCharController>& ctrl = character->GetController();

	out_min.setZero();
	out_max.setZero();
	if (ctrl != nullptr)
	{
		ctrl->GetViewBound(out_min, out_max);
	}
	out_min += tVector(-gCharViewDistPad, 0, -gCharViewDistPad, 0);
	out_max += tVector(gCharViewDistPad, 0, gCharViewDistPad, 0);
}

void cScenarioSimChar::ParseCharType(const std::string& char_type_str, eCharType& out_char_type) const
{
	bool found = false;
	if (char_type_str == "")
	{
		out_char_type = eCharNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharMax; ++i)
		{
			const std::string& name = gCharName[i];
			if (char_type_str == name)
			{
				out_char_type = static_cast<eCharType>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported character controller"); // unsupported character controller
	}
}

void cScenarioSimChar::ParseGroundParams(const std::shared_ptr<cArgParser>& parser, cGround::tParams& out_params) const
{
	std::string terrain_file = "";
	parser->ParseString("terrain_file", terrain_file);
	parser->ParseDouble("terrain_blend", out_params.mBlend);

	if (terrain_file != "")
	{
		bool succ = cGroundFactory::ParseParamsJson(terrain_file, out_params);
		if (!succ)
		{
			printf("Failed to parse terrain params from %s\n", terrain_file.c_str());
			assert(false);
		}
	}
}


void cScenarioSimChar::UpdateObjs(double time_step)
{
	int idx = 0;
	int num_objs = static_cast<int>(mObjs.size());
	for (size_t i = 0; i < num_objs; ++i)
	{
		const tObjEntry& obj = mObjs[i];
		if (obj.mEndTime > mTime)
		{
			mObjs[idx] = obj;
			++idx;
		}
	}

	if (idx != num_objs)
	{
		mObjs.resize(idx);
	}
}

void cScenarioSimChar::ClearObjs()
{
	mObjs.clear();
}

int cScenarioSimChar::AddObj(const tObjEntry& obj_entry)
{
	int handle = static_cast<int>(mObjs.size());
	mObjs.push_back(obj_entry);
	return handle;
}

void cScenarioSimChar::RemoveObj(int handle)
{
	assert(handle != gInvalidIdx);
	int num_objs = static_cast<int>(mObjs.size());
	mObjs[handle] = mObjs[num_objs - 1];
	mObjs.pop_back();
}

int cScenarioSimChar::GetNumObjs() const
{
	return static_cast<int>(mObjs.size());
}

void cScenarioSimChar::SpawnProjectile()
{
	double density = 100;
	double min_size = 0.1;
	double max_size = 0.3;
	double min_speed = 10;
	double max_speed = 20;
	double life_time = 2;
	double y_offset = 0;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

void cScenarioSimChar::SpawnBigProjectile()
{
	double density = 100;
	double min_size = 1.25;
	double max_size = 1.75;
	double min_speed = 11;
	double max_speed = 12;
	double life_time = 2;
	double y_offset = 0.5;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

const std::vector<cScenarioSimChar::tObjEntry, Eigen::aligned_allocator<cScenarioSimChar::tObjEntry>>& cScenarioSimChar::GetObjs() const
{
	return mObjs;
}

void cScenarioSimChar::SetPreSubstepCallback(tTimeCallbackFunc func)
{
	mPreSubstepCallback = func;
}

void cScenarioSimChar::SetPostSubstepCallback(tTimeCallbackFunc func)
{
	mPostSubstepCallback = func;
}

bool cScenarioSimChar::HasRandSeed() const
{
	return mHasRandSeed;
}

void cScenarioSimChar::SetRandSeed(unsigned long seed)
{
	mHasRandSeed = true;
	mRandSeed = seed;
	mRand.Seed(seed);
	if (mGround != nullptr)
	{
		mGround->SeedRand(seed);
	}
}

unsigned long cScenarioSimChar::GetRandSeed() const
{
	return mRandSeed;
}

cWorld::eSimMode cScenarioSimChar::GetSimMode() const
{
	return mWorldParams.mSimMode;
}

void cScenarioSimChar::SpawnProjectile(double density, double min_size, double max_size,
	double min_speed, double max_speed, double y_offset,
	double life_time)
{
	double min_dist = 1;
	double max_dist = 2;
	tVector aabb_min;
	tVector aabb_max;
	mChar->CalcAABB(aabb_min, aabb_max);

	tVector aabb_center = (aabb_min + aabb_max) * 0.5;
	tVector obj_size = tVector(1, 1, 1, 0) * mRand.RandDouble(min_size, max_size);
	
	cWorld::eSimMode sim_mode = mWorld->GetSimMode();

	double rand_theta = (sim_mode == cWorld::eSimMode3D) ? mRand.RandDouble(0, M_PI) : 0;
	double rand_dist = mRand.RandDouble(min_dist, max_dist);

	double aabb_size_x = (aabb_max[0] - aabb_min[0]);
	double aabb_size_z = (aabb_max[2] - aabb_min[2]);
	double buffer_dist = std::sqrt(aabb_size_x * aabb_size_x + aabb_size_z * aabb_size_z);

	double rand_x = 0.5 * buffer_dist + rand_dist * std::cos(rand_theta);
	rand_x *= mRand.RandSign();
	rand_x += aabb_center[0];
	double rand_y = mRand.RandDouble(aabb_min[1], aabb_max[1]) + obj_size[1] * 0.5;
	rand_y += y_offset;

	double rand_z = aabb_center[2];
	
	if (sim_mode == cWorld::eSimMode3D)
	{
		rand_z = 0.5 * buffer_dist + rand_dist * std::sin(rand_theta);
		rand_z *= mRand.RandSign();
		rand_z += aabb_center[2];
	}

	tVector pos = tVector(rand_x, rand_y, rand_z, 0);
	tVector target = tVector(mRand.RandDouble(aabb_min[0], aabb_max[0]),
		mRand.RandDouble(aabb_min[1], aabb_max[1]), aabb_center[2], 0);

	tVector com_vel = mChar->CalcCOMVel();
	tVector vel = (target - pos).normalized();
	vel *= mRand.RandDouble(min_speed, max_speed);
	vel[0] += com_vel[0];
	vel[2] += com_vel[2];

	cSimBox::tParams params;
	params.mSize = obj_size;
	params.mPos = pos;
	params.mVel = vel;
	params.mFriction = 0.7;
	params.mMass = density * params.mSize[0] * params.mSize[1] * params.mSize[2];
	std::shared_ptr<cSimBox> box = std::shared_ptr<cSimBox>(new cSimBox());
	box->Init(mWorld, params);
	box->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);

	tObjEntry obj_entry;
	obj_entry.mObj = box;
	obj_entry.mEndTime = mTime + life_time;
	
	AddObj(obj_entry);
}

void cScenarioSimChar::ResetRandPertrub()
{
	mRandPerturbTimer = 0;
	mRandPertubTime = mRand.RandDouble(mPerturbTimeMin, mPerturbTimeMax);
}