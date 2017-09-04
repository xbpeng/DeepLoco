#include "sim/SimCharSoftFall.h"

const double gFallDistCheckPeriod = 5;
const double gFallContactCheckPeriod = 0.1;
const double gMaxFallDist = 0.5;
const double gFallContactDiscount = 0.9;
const double gMaxSumFallContact = 0.25;

cSimCharSoftFall::cSimCharSoftFall()
{
	ResetFallDistCounter();
	mPrevCheckPos.setZero();
	mFailFallDist = false;
	ResetFallContactCounter();
	mSumFallContact = 0;
	mEnableFallDist = true;
	mEnableSoftContact = true;
}

cSimCharSoftFall::~cSimCharSoftFall()
{

}

bool cSimCharSoftFall::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	bool succ = cSimCharacter::Init(world, params);
	ResetFallDistCounter();
	mPrevCheckPos = GetRootPos();
	mFailFallDist = false;
	ResetFallContactCounter();
	mSumFallContact = 0;
	mEnableFallDist = params.mEnableFallDist;
	mEnableSoftContact = params.mEnableSoftContact;
	return succ;
}

void cSimCharSoftFall::Reset()
{
	cSimCharacter::Reset();
	ResetFallDistCounter();
	mPrevCheckPos = GetRootPos();
	mFailFallDist = false;
	ResetFallContactCounter();
	mSumFallContact = 0;
}

void cSimCharSoftFall::Update(double time_step)
{
	cSimCharacter::Update(time_step);
	if (mEnableFallDist)
	{
		UpdateFallDistCheck(time_step);
	}

	if (mEnableSoftContact)
	{
		UpdateFallContactCheck(time_step);
	}
}

bool cSimCharSoftFall::HasFallen() const
{
	bool fall_contact = FailFallContact();
	bool fall_dist = FailFallDist();
	bool fall_misc = FailFallMisc();

	bool has_fallen = fall_contact || fall_dist || fall_misc;
	return has_fallen;
}

void cSimCharSoftFall::EnableDistFall(bool enable)
{
	mEnableFallDist = enable;
}

void cSimCharSoftFall::EnableSoftContact(bool enable)
{
	mEnableSoftContact = enable;
}

void cSimCharSoftFall::ResetFallDistCounter()
{
	mFallDistCounter = gFallDistCheckPeriod;
}

bool cSimCharSoftFall::FailFallDist() const
{
	return mFailFallDist;
}

void cSimCharSoftFall::UpdateFallDistCheck(double time_step)
{
	mFallDistCounter -= time_step;
	if (mFallDistCounter <= 0)
	{
		tVector root_pos = GetRootPos();
		tVector dist = root_pos - mPrevCheckPos;

		if (dist.squaredNorm() < gMaxFallDist * gMaxFallDist)
		{
			mFailFallDist = true;
		}

		mPrevCheckPos = root_pos;
		ResetFallDistCounter();
	}
}

void cSimCharSoftFall::ResetFallContactCounter()
{
	mFallContatCounter = gFallContactCheckPeriod;
}

bool cSimCharSoftFall::FailFallContact() const
{
	if (mEnableSoftContact)
	{
		return mSumFallContact > gMaxSumFallContact;
	}
	else
	{
		return CheckFallContact();
	}
}

bool cSimCharSoftFall::CheckFallContact() const
{
	int num_parts = GetNumBodyParts();
	for (int b = 0; b < num_parts; ++b)
	{
		if (IsValidBodyPart(b) && EnableBodyPartFallContact(b))
		{
			const auto& curr_part = GetBodyPart(b);
			bool has_contact = curr_part->IsInContact();
			if (has_contact)
			{
				return true;
			}
		}
	}
	return false;
}

void cSimCharSoftFall::UpdateFallContactCheck(double time_step)
{
	const double discount = gFallContactDiscount;
	const double norm = (1 + 1 / (1 - discount));

	mFallContatCounter -= time_step;
	if (mFallContatCounter <= 0)
	{
		bool has_contact = CheckFallContact();
		double val = (has_contact) ? 1 : 0;
		mSumFallContact = val/ norm + discount * mSumFallContact;

		ResetFallContactCounter();
	}
}

bool cSimCharSoftFall::FailFallMisc() const
{
	return false;
}