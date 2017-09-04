#include "Ground.h"
#include "TerrainGen2D.h"

std::string gGroundTypeNames[cGround::eTypeMax] =
{
	"plane",
	"var2d_flat",
	"var2d_gaps",
	"var2d_steps",
	"var2d_walls",
	"var2d_bumps",
	"var2d_mixed",
	"var2d_narrow_gaps",
	"var2d_slopes",
	"var2d_slopes_gaps",
	"var2d_slopes_walls",
	"var2d_slopes_steps",
	"var2d_slopes_mixed",
	"var2d_slopes_narrow_gaps",
	"var2d_cliffs",
	"var3d_flat",
	"var3d_path",
	"var3d_cliff",
	"var3d_ramp",
	"hills3d",
	"trail3d",
	"obstacles3d",
	"dynamic_obstacles3d",
	"conveyor3d",
    "var3d_checkers",
    "var3d_stairs"
};

const std::string cGround::gTypeKey = "Type";
const std::string cGround::gParamsKey = "Params";

cGround::tParams::tParams()
{
	mType = eTypeVar2DFlat;
	mFriction = 0.9;
	mOrigin.setZero();
	mBlend = 0;

	mGroundWidth = 20;

	mRandSeed = 0;
	mHasRandSeed = false;
}

cGround::eClass cGround::GetClassFromType(eType ground_type)
{
	eClass ground_class = eClassInvalid;
	switch (ground_type)
	{
		case eTypePlane:
			ground_class = eClassPlane;
			break;
		case eTypeVar2DFlat:
		case eTypeVar2DGaps:
		case eTypeVar2DSteps:
		case eTypeVar2DWalls:
		case eTypeVar2DBumps:
		case eTypeVar2DMixed:
		case eTypeVar2DNarrowGaps:
		case eTypeVar2DSlopes:
		case eTypeVar2DSlopesGaps:
		case eTypeVar2DSlopesWalls:
		case eTypeVar2DSlopesSteps:
		case eTypeVar2DSlopesMixed:
		case eTypeVar2DSlopesNarrowGaps:
		case eTypeVar2DCliffs:
			ground_class = eClassVar2D;
			break;
		case eTypeVar3DFlat:
		case eTypeVar3DPath:
		case eTypeVar3DCliff:
		case eTypeVar3DRamp:
			ground_class = eClassVar3D;
			break;
		case eTypeHills3D:
			ground_class = eClassHills3D;
			break;
		case eTypeTrail3D:
			ground_class = eClassTrail3D;
			break;
		case eTypeObstacles3D:
			ground_class = eClassObstacles3D;
			break;
		case eTypeDynamicObstacles3D:
			ground_class = eClassDynamicObstacles3D;
			break;
		case eTypeConveyor3D:
			ground_class = eClassConveyor3D;
			break;
        case eTypeVar3DCheckers:
			ground_class = eClassVar3D;
			break;
        case eTypeVar3DStairs:
			ground_class = eClassVar3D;
			break;
		default:
			ground_class = eClassInvalid;
			break;
	}
	return ground_class;
}

void cGround::ParseType(const std::string& str, eType& out_type)
{
	bool valid = false;
	for (int i = 0; i < eTypeMax; ++i)
	{
		const std::string& curr_name = gGroundTypeNames[i];
		if (curr_name == str)
		{
			out_type = static_cast<eType>(i);
			valid = true;
			break;
		}
	}
	
	if (!valid)
	{
		printf("Invalid ground Type%s\n", str.c_str());
		assert(false);
	}
}


cGround::cGround()
{
	SeedRand(cMathUtil::RandUint());
	mType = eTypeStatic;
	mUpdateCount = 0;
}

cGround::~cGround()
{
}

void cGround::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	mParams = params;
	SetupRandGen();

	cSimObj::Init(world);
	UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
	SetParamBlend(params.mBlend);
	mUpdateCount = 0;
	mTime = 0;
	FlagUpdate();
}

void cGround::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	mTime += time_elapsed;
}

void cGround::Clear()
{
	mUpdateCount = 0;
	mTime = 0;
}

double cGround::SampleHeight(const tVector& pos) const
{
	bool dummy_valid = true;
	return SampleHeight(pos, dummy_valid);
}

double cGround::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	out_valid_sample = true;
	return 0;
}

void cGround::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
{
	int num_pos = static_cast<int>(pos.rows());
	out_h = Eigen::VectorXd::Zero(num_pos);
}

void cGround::SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel,
							bool& out_valid_sample) const
{
	out_h = SampleHeight(pos, out_valid_sample);
	out_vel.setZero();
}

cGround::eClass cGround::GetGroundClass() const
{
	return eClassInvalid;
}

void cGround::SetBlendParams(const Eigen::VectorXd& params)
{
	int blend_param_size = GetBlendParamSize();
	assert(params.size() == blend_param_size);
	mBlendParams = params;
}

void cGround::SetParamBlend(double blend)
{
	mParams.mBlend = blend;
	CalcBlendParams(blend, mBlendParams);
}

size_t cGround::GetUpdateCount() const
{
	return mUpdateCount;
}

void cGround::SeedRand(unsigned long seed)
{
	mRand.Seed(seed);
}

bool cGround::Output(const std::string& out_file) const
{
	return false;
}

void cGround::SetupRandGen()
{
	if (mParams.mHasRandSeed)
	{
		SeedRand(mParams.mRandSeed);
	}
}

void cGround::CalcBlendParams(double blend, Eigen::VectorXd& out_params) const
{
	int num_params = GetNumParamSets();
	if (num_params > 0)
	{
		blend = cMathUtil::Clamp(blend, 0.0, num_params - 1.0);

		int idx0 = static_cast<int>(blend);
		int idx1 = std::min(idx0 + 1, num_params - 1);
		blend -= idx0;

		auto params0 = mParams.mParamArr.row(idx0);
		auto params1 = mParams.mParamArr.row(idx1);

		int blend_param_size = GetBlendParamSize();
		assert(params0.size() == blend_param_size);
		assert(params1.size() == blend_param_size);

		Eigen::VectorXd blend_params = (1 - blend) * params0 + blend * params1;

		out_params = blend_params;
	}
}

int cGround::GetNumParamSets() const
{
	return static_cast<int>(mParams.mParamArr.rows());
}

int cGround::GetBlendParamSize() const
{
	return 0;
}

void cGround::FlagUpdate()
{
	++mUpdateCount;
}
