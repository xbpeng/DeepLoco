#pragma once

#include "util/MathUtil.h"

class cPerlinNoise
{
public:
	cPerlinNoise();
	virtual ~cPerlinNoise();

	virtual double Eval(tVector p);
	virtual void SetScale(double norm_repeat);

private:

	static const int gPerm[];
	static const int gPermLen;

	int mRepeat;
	virtual double SmoothLerp(double t);
	virtual int Inc(int x);
	virtual double Grad(int hash, double x, double y, double z);
};
