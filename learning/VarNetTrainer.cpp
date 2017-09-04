#include "VarNetTrainer.h"

cVarNetTrainer::cVarNetTrainer()
{
}

cVarNetTrainer::~cVarNetTrainer()
{
}

int cVarNetTrainer::GetActionSize() const
{
	return GetOutputSize() / 2;
}

void cVarNetTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	int output_size = GetOutputSize();
	int action_size = GetActionSize();
	assert(output_size == 2 * action_size);

	auto& net = GetTargetNet(net_id);

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildTupleX(tuple, x);
	net->Eval(x, y);

	out_y.resize(output_size);
	out_y.segment(0, action_size) = tuple.mAction;
	
	auto mean = y.segment(0, action_size);
	auto sig = y.segment(action_size, action_size);
	Eigen::VectorXd dy = out_y.segment(0, action_size) - mean;
	Eigen::VectorXd inv_sig = sig.cwiseInverse();
	Eigen::VectorXd d_sig = inv_sig.cwiseProduct(inv_sig.cwiseProduct(inv_sig));
	d_sig = d_sig.cwiseProduct(dy.cwiseProduct(dy));
	d_sig += -inv_sig;

	out_y.segment(action_size, action_size) = sig + d_sig;
}