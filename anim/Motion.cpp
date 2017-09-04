#include "Motion.h"
#include <assert.h>
#include <iostream>

#include "util/FileUtil.h"
#include "util/JsonUtil.h"

const double gMinTime = 0;

// Json keys
const std::string cMotion::gFrameKey = "Frames";
const std::string cMotion::gLoopKey = "Loop";

std::string cMotion::BuildFrameJson(const Eigen::VectorXd& frame)
{
	std::string json = cJsonUtil::BuildVectorJson(frame);
	return json;
}

cMotion::cMotion()
{
	Clear();
	mLoop = false;
}

cMotion::~cMotion()
{

}

void cMotion::Clear()
{
	mFrames.resize(0, 0);
	mBlendFunc = nullptr;
}

bool cMotion::Load(const std::string& file)
{
	Clear();
	
	std::ifstream f_stream(file);
	Json::Value root;
	Json::Reader reader;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ = LoadJson(root);
		if (succ)
		{
			PostProcessFrames(mFrames);
		}
		else
		{
			printf("Failed to load motion from file %s\n", file.c_str());
			assert(false);
		}
	}
	else
	{
		printf("Failed to parse Json from %s\n", file.c_str());
		assert(false);
	}
	return succ;
}

void cMotion::Init(int num_frames, int num_dofs)
{
	Clear();
	mFrames = Eigen::MatrixXd::Zero(num_frames, num_dofs + 1);
}

bool cMotion::IsValid() const
{
	return GetNumFrames() > 0;
}

int cMotion::GetNumDof() const
{
	return GetFrameSize() - 1;
}

int cMotion::GetNumFrames() const
{
	return static_cast<int>(mFrames.rows());
}

int cMotion::GetFrameSize() const
{
	return static_cast<int>(mFrames.cols());
}

cMotion::tFrame cMotion::GetFrame(int i) const
{
	int frame_size = GetFrameSize();
	return mFrames.row(i).segment(1, frame_size - 1);
}

void cMotion::SetFrame(int i, const tFrame& frame)
{
	int frame_size = GetFrameSize();
	assert(frame.size() == frame_size - 1);
	mFrames.row(i).segment(1, frame_size - 1) = frame;
}

void cMotion::SetFrameTime(int i, double time)
{
	mFrames(i, 0) = time;
}

cMotion::tFrame cMotion::BlendFrames(int a, int b, double lerp) const
{
	lerp = cMathUtil::Saturate(lerp);

	// remove time params
	tFrame frame0 = GetFrame(a);
	tFrame frame1 = GetFrame(b);
	
	if (HasBlendFunc())
	{
		return mBlendFunc(&frame0, &frame1, lerp);
	}
	else
	{
		return BlendFramesIntern(&frame0, &frame1, lerp);
	}
}

cMotion::tFrame cMotion::CalcFrame(double time) const
{
	int idx;
	double phase;
	CalcIndexPhase(time, idx, phase);

	tFrame frame = BlendFrames(idx, idx + 1, phase);
	return frame;
}

cMotion::tFrame cMotion::CalcFramePhase(double phase) const
{
	double max_time = GetDuration();
	double time = phase * max_time;
	return CalcFrame(time);
}

bool cMotion::LoadJson(const Json::Value& root)
{
	bool succ = true;
	if (!root[gLoopKey].isNull())
	{
		mLoop = root[gLoopKey].asBool();
	}

	if (!root[gFrameKey].isNull())
	{
		Json::Value frames = root.get(gFrameKey, 0);
		assert(frames.isArray());
		int num_frames = frames.size();

		int data_size = 0;
		if (num_frames > 0)
		{
			int idx0 = 0;
			Json::Value frame_json = frames.get(idx0, 0);
			data_size = frame_json.size();
			mFrames.resize(num_frames, data_size);
		}

		for (int f = 0; f < num_frames; ++f)
		{
			Eigen::VectorXd curr_frame;
			succ = ParseFrameJson(frames.get(f, 0), curr_frame);
			if (succ)
			{
				assert(mFrames.cols() == curr_frame.size());
				mFrames.row(f) = curr_frame;
			}
			else
			{
				mFrames.resize(0, 0);
				break;
			}
		}
	}
	return succ;
}

bool cMotion::ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const
{
	bool succ = false;
	if (root.isArray())
	{
		int data_size = root.size();
		out_frame.resize(data_size);
		for (int i = 0; i < data_size; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_frame[i] = json_elem.asDouble();
		}

		succ = true;
	}
	return succ;
}

void cMotion::PostProcessFrames(Eigen::MatrixXd& frames) const
{
	int num_frames = static_cast<int>(frames.rows());
	double curr_time = gMinTime;
	for (int f = 0; f < num_frames; ++f)
	{
		double duration = frames.row(f)(0, eFrameTime);
		frames.row(f)(0, eFrameTime) = curr_time;
		curr_time += duration;
	}
}

double cMotion::GetDuration() const
{
	int num_frames = GetNumFrames();
	double max_time = mFrames(num_frames - 1, eFrameTime);
	return max_time;
}

double cMotion::GetFrameTime(int f) const
{
	return mFrames(f, eFrameTime);
}

int cMotion::CalcCycleCount(double time) const
{
	double dur = GetDuration();
	double phases = time / dur;
	int count = static_cast<int>(std::floor(phases));
	count = (mLoop) ? count : cMathUtil::Clamp(count, 0, 1);
	return count;
}

void cMotion::CalcIndexPhase(double time, int& out_idx, double& out_phase) const
{
	double max_time = GetDuration();

	if (!mLoop)
	{
		if (time <= gMinTime)
		{
			out_idx = 0;
			out_phase = 0;
			return;
		}
		else if (time >= max_time)
		{
			out_idx = GetNumFrames() - 2;
			out_phase = 1;
			return;
		}
	}

	time = std::fmod(time, max_time);
	if (time < 0)
	{
		time += max_time;
	}

	const Eigen::VectorXd& frame_times = mFrames.col(eFrameTime);
	auto it = std::upper_bound(frame_times.data(), frame_times.data() + frame_times.size(), time);
	out_idx = static_cast<int>(it - frame_times.data() - 1);

	double time0 = frame_times(out_idx);
	double time1 = frame_times(out_idx + 1);
	out_phase = (time - time0) / (time1 - time0);
}

bool cMotion::IsOver(double time) const
{
	return !mLoop && (time > GetDuration());
}

void cMotion::SetBlendFunc(tBlendFunc blend_func)
{
	mBlendFunc = blend_func;
}

bool cMotion::HasBlendFunc() const
{
	return mBlendFunc != nullptr;
}

bool cMotion::IsLoop() const
{
	return mLoop;
}

cMotion::tFrame cMotion::BlendFramesIntern(const Eigen::VectorXd* a, const Eigen::VectorXd* b, double lerp) const
{
	tFrame frame = (1 - lerp) * (*a) + lerp * (*b);
	return frame;
}

void cMotion::Output(const std::string& out_filepath) const
{
	FILE* file = cFileUtil::OpenFile(out_filepath, "w");

	fprintf(file, "{\n");
	fprintf(file, "\"%s\": ", gLoopKey.c_str());

	if (mLoop)
	{
		fprintf(file, "true,\n");
	}
	else
	{
		fprintf(file, "false,\n");
	}

	fprintf(file, "\"Frames\":\n[\n");

	int num_frames = GetNumFrames();
	for (int f = 0; f < num_frames; ++f)
	{
		if (f != 0)
		{
			fprintf(file, ",\n");
		}

		Eigen::VectorXd curr_frame = mFrames.row(f);
		double curr_time = curr_frame[eFrameTime];
		double dur = 0;
		if (f < num_frames - 1)
		{
			double next_time = GetFrameTime(f + 1);
			dur = next_time - curr_time;
		}
		curr_frame[eFrameTime] = dur;
		std::string frame_json = cJsonUtil::BuildVectorJson(curr_frame);
		fprintf(file, "%s", frame_json.c_str());
	}

	fprintf(file, "\n]");
	fprintf(file, "\n}");
	cFileUtil::CloseFile(file);
}