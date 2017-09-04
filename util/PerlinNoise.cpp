/*
Code from https://gist.github.com/Flafla2/f0260a861be0ebdeef76
*/

#include "PerlinNoise.h"

const int cPerlinNoise::gPerm[] = { 151,160,137,91,90,15,					// Hash lookup table as defined by Ken Perlin.  This is a randomly
131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,	// arranged array of all numbers from 0-255 inclusive.
190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};
const int cPerlinNoise::gPermLen = sizeof(cPerlinNoise::gPerm) / sizeof(cPerlinNoise::gPerm[0]);

double cPerlinNoise::SmoothLerp(double t) {
	return t * t * t * (t * (t * 6 - 15) + 10);	// 6t^5 - 15t^4 + 10t^3
}

cPerlinNoise::cPerlinNoise()
{
	mRepeat = gPermLen;
}

cPerlinNoise::~cPerlinNoise()
{
}

int cPerlinNoise::Inc(int x)
{
	return (x + 1) % mRepeat;
}

double cPerlinNoise::Eval(tVector p)
{
	p[0] = std::fmod(p[0], 1);
	p[1] = std::fmod(p[1], 1);
	p[2] = std::fmod(p[2], 1);
	p[0] = (p[0] < 0) ? (1 + p[0]) : p[0];
	p[1] = (p[1] < 0) ? (1 + p[1]) : p[1];
	p[2] = (p[2] < 0) ? (1 + p[2]) : p[2];
	p *= mRepeat;

	int xi = static_cast<int>(p[0]);
	int yi = static_cast<int>(p[1]);
	int zi = static_cast<int>(p[2]);
	double xf = p[0] - xi;
	double yf = p[1] - yi;
	double zf = p[2] - zi;
	double u = SmoothLerp(xf);
	double v = SmoothLerp(yf);
	double w = SmoothLerp(zf);

	int aaa = gPerm[(gPerm[(gPerm[		xi] +		yi)		% gPermLen] +		zi) % gPermLen];
	int aba = gPerm[(gPerm[(gPerm[		xi] +	Inc(yi))	% gPermLen] +		zi) % gPermLen];
	int aab = gPerm[(gPerm[(gPerm[		xi] +		yi)		% gPermLen] +	Inc(zi)) % gPermLen];
	int abb = gPerm[(gPerm[(gPerm[		xi] +	Inc(yi))	% gPermLen] +	Inc(zi)) % gPermLen];
	int baa = gPerm[(gPerm[(gPerm[Inc(xi)] +		yi)		% gPermLen] +		zi) % gPermLen];
	int bba = gPerm[(gPerm[(gPerm[Inc(xi)] +	Inc(yi))	% gPermLen] +		zi) % gPermLen];
	int bab = gPerm[(gPerm[(gPerm[Inc(xi)] +		yi)		% gPermLen] +	Inc(zi)) % gPermLen];
	int bbb = gPerm[(gPerm[(gPerm[Inc(xi)] +	Inc(yi))	% gPermLen] +	Inc(zi)) % gPermLen];

	double x1 = cMathUtil::Lerp(u, Grad(aaa, xf, yf, zf),
									Grad(baa, xf - 1, yf, zf));		
	double x2 = cMathUtil::Lerp(u, Grad(aba, xf, yf - 1, zf),
									Grad(bba, xf - 1, yf - 1, zf));
	double y1 = cMathUtil::Lerp(v, x1, x2);

	double x3 = cMathUtil::Lerp(u, Grad(aab, xf, yf, zf - 1),
									Grad(bab, xf - 1, yf, zf - 1));
	double x4 = cMathUtil::Lerp(u, Grad(abb, xf, yf - 1, zf - 1),
									Grad(bbb, xf - 1, yf - 1, zf - 1));
	double y2 = cMathUtil::Lerp(v, x3, x4);

	double val = (cMathUtil::Lerp(w, y1, y2) + 1) / 2;
	return val;
}

void cPerlinNoise::SetScale(double scale)
{
	mRepeat = static_cast<int>(std::ceil(gPermLen / scale));
	mRepeat = cMathUtil::Clamp(mRepeat, 1, gPermLen);
}

double cPerlinNoise::Grad(int hash, double x, double y, double z) {
	int h = hash & 15;                      // CONVERT LO 4 BITS OF HASH CODE
	double u = h<8 ? x : y,                 // INTO 12 GRADIENT DIRECTIONS.
		v = h<4 ? y : h == 12 || h == 14 ? x : z;
	double val = ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
	return val;
}