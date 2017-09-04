#pragma once

#include "sim/SimCharSoftFall.h"


class cSimCharGeneral : public cSimCharSoftFall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSimCharGeneral();
	virtual ~cSimCharGeneral();

	virtual bool HasStumbled() const;

protected:
};