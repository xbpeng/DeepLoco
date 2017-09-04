#include "learning/ExpBuffer.h"

cExpBuffer::tParams::tParams()
{
	mNumEntries = 0;
	mStateBegSize = 0;
	mActionSize = 0;
	mStateEndSize = 0;
}

cExpBuffer::cExpBuffer()
{
	mBufferHead = 0;
	mNumTuples = 0;
	mTotalTuples = 0;
}

cExpBuffer::~cExpBuffer()
{
}

void cExpBuffer::Init(const tParams& params)
{
	Clear();
	mParams = params;

	mMemory.resize(params.mNumEntries, CalcEntrySize());
	InitFlagBuffer();
	InitNextBuffer();
	InitSrcStream();
	mLogpBuffer = Eigen::VectorXd::Zero(params.mNumEntries);
}

void cExpBuffer::Reset()
{
	mBufferHead = 0;
	mNumTuples = 0;
	mTotalTuples = 0;
}

void cExpBuffer::Clear()
{
	Reset();
	mMemory.resize(0, 0);
	mFlagBuffer.resize(0);
	mLogpBuffer.resize(0);
}

int cExpBuffer::GetSize() const
{
	return static_cast<int>(mMemory.rows());
}

int cExpBuffer::GetNumTuples() const
{
	return mNumTuples;
}

int cExpBuffer::GetTotalNumTuples() const
{
	return mTotalTuples;
}

int cExpBuffer::GetBufferHead() const
{
	return mBufferHead;
}

int cExpBuffer::AddTuple(const tExpTuple& tuple, int prev_id, int stream_id)
{
	int state_beg_size = GetStateBegSize();
	int action_size = GetActionSize();
	int state_end_size = GetStateEndSize();
	assert(tuple.mStateBeg.size() == state_beg_size);
	assert(tuple.mAction.size() == action_size);
	assert(tuple.mStateEnd.size() == state_end_size);

	int id = gInvalidIdx;
	bool valid_tuple = CheckTuple(tuple);
	if (valid_tuple)
	{
		id = mBufferHead;
		SetTuple(id, tuple);

		int buffer_size = GetSize();
		mNumTuples = std::min(buffer_size, mNumTuples + 1);
		++mTotalTuples;
		mBufferHead = IncBufferHead(mBufferHead);

		bool is_start = tuple.GetFlag(tExpTuple::eFlagStart);
		if (!is_start && prev_id != gInvalidIdx)
		{
			if (mSrcStream[prev_id] == stream_id)
			{
				mNextBuffer[prev_id] = id;
			}
		}
		mNextBuffer[id] = gInvalidIdx;
		mSrcStream[id] = stream_id;
	}
	else
	{
		printf("Bad tuple detected!!!!\n");

		// hack this is all hacks
		const std::string& bad_tuple_file = "output/bad_tuple.txt";
		//if (!cFileUtil::ExistsFile(bad_tuple_file))
		{
			//OutputTuple(tuple, bad_tuple_file);
		}
	}
	return id;
}

tExpTuple cExpBuffer::GetTuple(int t) const
{
	tExpTuple tuple;
	auto curr_row = mMemory.row(t);

	int reward_idx = GetRewardIdx();
	int state_beg_idx = GetStateBegIdx();
	int state_end_idx = GetStateEndIdx();
	int action_idx = GetActionIdx();
	int state_beg_size = GetStateBegSize();
	int action_size = GetActionSize();
	int state_end_size = GetStateEndSize();

	tuple.mID = t;
	tuple.mReward = curr_row[reward_idx];

	tuple.mStateBeg.resize(state_beg_size);
	tuple.mStateEnd.resize(state_end_size);
	tuple.mAction.resize(action_size);

	for (int j = 0; j < state_beg_size; ++j)
	{
		tuple.mStateBeg(j) = curr_row(state_beg_idx + j);
	}

	for (int j = 0; j < action_size; ++j)
	{
		tuple.mAction(j) = curr_row(action_idx + j);
	}

	for (int j = 0; j < state_end_size; ++j)
	{
		tuple.mStateEnd(j) = curr_row(state_end_idx + j);
	}

	tuple.mFlags = GetFlags(t);
	tuple.mActionLogp = mLogpBuffer[t];

	return tuple;
}

int cExpBuffer::GetFlags(int t) const
{
	return mFlagBuffer[t];
}

int cExpBuffer::GetNextTupleID(int t) const
{
	return mNextBuffer[t];
}

double cExpBuffer::GetReward(int t) const
{
	int reward_idx = GetRewardIdx();
	double reward = mMemory(t, reward_idx);
	return reward;
}

int cExpBuffer::GetRandTupleID() const
{
	int t = gInvalidIdx;
	int num_tuples = GetNumTuples();
	if (num_tuples > 0)
	{
		t = cMathUtil::RandInt(0, num_tuples);
	}
	return t;
}

int cExpBuffer::GetLastTupleID(int i) const
{
	// i = 0: latest tuple added
	// i = 1: second latest tuple added
	// etc...
	int t = gInvalidIdx;
	int num_tuples = GetNumTuples();
	if (num_tuples > 0)
	{
		t = mBufferHead - i - 1;
		if (t < 0)
		{
			t = num_tuples + (t + 1) % num_tuples - 1;
		}
	}
	return t;
}

void cExpBuffer::InitFlagBuffer()
{
	mFlagBuffer.resize(mParams.mNumEntries);
	for (int i = 0; i < mParams.mNumEntries; ++i)
	{
		mFlagBuffer[i] = 0;
	}
}

void cExpBuffer::InitNextBuffer()
{
	mNextBuffer.resize(mParams.mNumEntries);
	for (int i = 0; i < mParams.mNumEntries; ++i)
	{
		mNextBuffer[i] = gInvalidIdx;
	}
}

void cExpBuffer::InitSrcStream()
{
	mSrcStream.resize(mParams.mNumEntries);
	for (int i = 0; i < mParams.mNumEntries; ++i)
	{
		mSrcStream[i] = gInvalidIdx;
	}
}

int cExpBuffer::GetStateBegSize() const
{
	return mParams.mStateBegSize;
}

int cExpBuffer::GetActionSize() const
{
	return mParams.mActionSize;
}

int cExpBuffer::GetStateEndSize() const
{
	return mParams.mStateEndSize;
}

int cExpBuffer::CalcEntrySize() const
{
	return 1 + GetStateBegSize() + GetStateEndSize() + GetActionSize();
}

int cExpBuffer::IncBufferHead(int head) const
{
	int buffer_size = GetSize();
	int new_head = (head + 1) % buffer_size;
	return new_head;
}

bool cExpBuffer::CheckTuple(const tExpTuple& tuple) const
{
	// hack hack hack
	const double val_threshold = 50;
	//const double val_threshold = std::numeric_limits<double>::infinity();

	if (!std::isfinite(tuple.mReward))
	{
		return false;
	}

	for (int i = 0; i < static_cast<int>(tuple.mStateBeg.size()); ++i)
	{
		double curr_val = tuple.mStateBeg[i];
		if (!std::isfinite(curr_val) || std::abs(curr_val) > val_threshold) // hack hack hack
		{
			return false;
		}
	}

	for (int i = 0; i < static_cast<int>(tuple.mStateEnd.size()); ++i)
	{
		double curr_val = tuple.mStateEnd[i];
		if (!std::isfinite(curr_val) || std::abs(curr_val) > val_threshold)
		{
			return false;
		}
	}

	for (int i = 0; i < static_cast<int>(tuple.mAction.size()); ++i)
	{
		double curr_val = tuple.mAction[i];
		if (!std::isfinite(curr_val) || std::abs(curr_val) > val_threshold)
		{
			return false;
		}
	}

	return true;
}

void cExpBuffer::SetTuple(int t, const tExpTuple& tuple)
{
	auto curr_row = mMemory.row(t);

	int reward_idx = GetRewardIdx();
	int state_beg_idx = GetStateBegIdx();
	int state_end_idx = GetStateEndIdx();
	int action_idx = GetActionIdx();
	int state_beg_size = GetStateBegSize();
	int state_end_size = GetStateEndSize();
	int action_size = GetActionSize();

	curr_row(reward_idx) = static_cast<float>(tuple.mReward);

	for (int j = 0; j < state_beg_size; ++j)
	{
		curr_row(state_beg_idx + j) = static_cast<float>(tuple.mStateBeg(j));
	}

	for (int j = 0; j < action_size; ++j)
	{
		curr_row(action_idx + j) = static_cast<float>(tuple.mAction(j));
	}

	for (int j = 0; j < state_end_size; ++j)
	{
		curr_row(state_end_idx + j) = static_cast<float>(tuple.mStateEnd(j));
	}

	mFlagBuffer[t] = tuple.mFlags;
	mLogpBuffer[t] = tuple.mActionLogp;
}


int cExpBuffer::GetRewardIdx() const
{
	return 0;
}

int cExpBuffer::GetStateBegIdx() const
{
	return GetRewardIdx() + 1;
}

int cExpBuffer::GetStateEndIdx() const
{
	return GetActionIdx() + GetActionSize();
}

int cExpBuffer::GetActionIdx() const
{
	return GetStateBegIdx() + GetStateBegSize();
}