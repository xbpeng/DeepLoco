#include "ScenarioExpCacla.h"
#include "learning/CaclaTrainer.h"

//#define RECORD_ONLY_OFF_POLICY_TUPLE

cScenarioExpCacla::cScenarioExpCacla()
{
}

cScenarioExpCacla::~cScenarioExpCacla()
{
}

std::string cScenarioExpCacla::GetName() const
{
	return "Exploration Cacla";
}

bool cScenarioExpCacla::IsValidTuple(const tExpTuple& tuple) const
{
	bool valid = cScenarioExp::IsValidTuple(tuple);

#if defined(RECORD_ONLY_OFF_POLICY_TUPLE)
	valid &= IsOffPolicyTuple(tuple);
#endif // RECORD_ONLY_OFF_POLICY_TUPLE

	return valid;
}

bool cScenarioExpCacla::IsOffPolicyTuple(const tExpTuple& tuple) const
{
	return tuple.GetFlag(tExpTuple::eFlagOffPolicy);
}

void cScenarioExpCacla::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	cScenarioExp::RecordFlagsBeg(out_tuple);

	const auto nn_ctrl = GetNNController();
	bool off_policy = nn_ctrl->IsOffPolicy();
	out_tuple.SetFlag(off_policy, tExpTuple::eFlagOffPolicy);
}
