#pragma once

#include "learning/NeuralNetTrainer.h"

class cVarNetTrainer : public cNeuralNetTrainer
{
public:
	cVarNetTrainer();
	virtual ~cVarNetTrainer();

	virtual int GetActionSize() const;

protected:

	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
};