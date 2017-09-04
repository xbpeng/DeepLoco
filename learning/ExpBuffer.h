#pragma once

#include "learning/ExpTuple.h"

struct cExpBuffer
{
public:
	
	struct tParams
	{
		int mNumEntries;
		int mStateBegSize;
		int mActionSize;
		int mStateEndSize;
		tParams();
	};

	cExpBuffer();
	virtual ~cExpBuffer();

	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual void Clear();
	virtual int GetSize() const;
	virtual int GetNumTuples() const;
	virtual int GetTotalNumTuples() const;
	virtual int GetBufferHead() const;

	virtual int AddTuple(const tExpTuple& tuple, int prev_id, int stream_id);
	virtual tExpTuple GetTuple(int t) const;
	virtual int GetFlags(int t) const;
	virtual int GetNextTupleID(int t) const;
	virtual double GetReward(int t) const;

	virtual int GetRandTupleID() const;
	virtual int GetLastTupleID(int i) const;

protected:

	tParams mParams;
	int mBufferHead;
	int mNumTuples;
	int mTotalTuples;

	Eigen::MatrixXf mMemory;
	std::vector<unsigned int> mFlagBuffer;
	Eigen::VectorXd mLogpBuffer; // needs more precision so keep everything as doubles
	std::vector<int> mNextBuffer; // index of the next tuple in a trajectory
	std::vector<int> mSrcStream;

	virtual void InitFlagBuffer();
	virtual void InitNextBuffer();
	virtual void InitSrcStream();

	virtual int GetStateBegSize() const;
	virtual int GetActionSize() const;
	virtual int GetStateEndSize() const;
	virtual int CalcEntrySize() const;

	virtual int IncBufferHead(int head) const;
	virtual bool CheckTuple(const tExpTuple& tuple) const;
	virtual void SetTuple(int t, const tExpTuple& tuple);

	virtual int GetRewardIdx() const;
	virtual int GetStateBegIdx() const;
	virtual int GetStateEndIdx() const;
	virtual int GetActionIdx() const;
};