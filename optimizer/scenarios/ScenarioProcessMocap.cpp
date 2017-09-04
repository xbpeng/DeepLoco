#include "scenarios/ScenarioProcessMocap.h"
#include "util/FileUtil.h"
#include "anim/Character.h"

cScenarioProcessMocap::cScenarioProcessMocap()
{
	mTargetFramerate = 0;
}

cScenarioProcessMocap::~cScenarioProcessMocap()
{
}

void cScenarioProcessMocap::Init()
{
}

void cScenarioProcessMocap::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	parser->ParseStringArray("input_files", mInputFiles);
	parser->ParseString("output_path", mOutputPath);
	parser->ParseDouble("target_framerate", mTargetFramerate);
}


void cScenarioProcessMocap::Clear()
{
	cScenario::Clear();
	mInputFiles.clear();
	mOutputPath = "";
}

void cScenarioProcessMocap::Run()
{
	ConvertMocapFiles(mInputFiles, mOutputPath);
}

std::string cScenarioProcessMocap::GetName() const
{
	return "Process Mocap";
}

void cScenarioProcessMocap::ConvertMocapFiles(const std::vector<std::string>& input_files, const std::string& output_path)
{
	// rotate model so that it is facing along positive x
	const tMatrix model_trans = cMathUtil::RotateMat(tVector(0, 1, 0, 0), M_PI / 2);

	int num_input_files = static_cast<int>(input_files.size());
	for (int f = 0; f < num_input_files; ++f)
	{
		cBVHReader reader;
		reader.SetModelTransform(model_trans);
		const std::string& filepath = input_files[f];
		reader.parseBVH(filepath);

		std::string ext = "txt";
		std::string filename = cFileUtil::GetFilename(filepath);
		
		std::string skeleton_file = output_path + filename + "_skeleton." + ext;
		std::string motion_file = output_path + filename + "_motion." + ext;

		OutputSkeleton(reader, skeleton_file);
		OutputMotion(reader, motion_file);
	}
}

void cScenarioProcessMocap::OutputSkeleton(const cBVHReader& reader, const std::string& out_filepath) const
{
	Eigen::MatrixXd joint_mat;
	reader.BuildJointMat(joint_mat);

	std::string json = cKinTree::BuildJointMatJson(joint_mat);
	json = "{\n\"" + cCharacter::gSkeletonKey + "\":\n" + json + "\n}";

	FILE* file = cFileUtil::OpenFile(out_filepath, "w");
	fprintf(file, "%s", json.c_str());
	cFileUtil::CloseFile(file);
}

void cScenarioProcessMocap::OutputMotion(const cBVHReader& reader, const std::string& out_filepath) const
{
	cMotion motion;
	reader.BuildMotion(mTargetFramerate, motion);
	motion.Output(out_filepath);
}