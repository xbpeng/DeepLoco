#include "VarCaclaTrainer.h"

cVarCaclaTrainer::cVarCaclaTrainer()
{
}

cVarCaclaTrainer::~cVarCaclaTrainer()
{
}

int cVarCaclaTrainer::GetActionSize() const
{
	int action_size = cCaclaTrainer::GetActionSize();
	action_size /= 2;
	return action_size;
}

void cVarCaclaTrainer::BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	int output_size = GetActorOutputSize();
	int action_size = GetActionSize();
	assert(output_size == tuple.mAction.size() * 2);

	out_y.resize(output_size);

	Eigen::VectorXd action;
	cACTrainer::BuildTupleActorY(tuple, action);

	const auto& actor_net = GetActor();
	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildTupleActorX(tuple, x);
	actor_net->Eval(x, y);

	Eigen::VectorXd mean = y.segment(0, action_size);
	Eigen::VectorXd sig = y.segment(action_size, action_size);
	Eigen::VectorXd dy = action - mean;
	Eigen::VectorXd inv_sig = sig.cwiseInverse();
	Eigen::VectorXd d_sig = inv_sig.cwiseProduct(inv_sig.cwiseProduct(inv_sig));
	d_sig = d_sig.cwiseProduct(dy.cwiseProduct(dy));
	d_sig += -inv_sig;
	d_sig *= 0.1; // hack

	sig += d_sig;

	cMathUtil::Clamp(mActionMin, mActionMax, action);
	out_y.segment(0, action_size) = action;
	out_y.segment(action_size, action_size) = sig;
}