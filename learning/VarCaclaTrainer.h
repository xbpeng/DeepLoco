#pragma once

#include "learning/CaclaTrainer.h"

class cVarCaclaTrainer : public cCaclaTrainer
{
public:
	
	cVarCaclaTrainer();
	virtual ~cVarCaclaTrainer();

	virtual int GetActionSize() const;

protected:
	
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);
};