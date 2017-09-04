#pragma once
#ifndef __HOSEKSKYMODEL_H
#define __HOSEKSKYMODEL_H

class cHosekSkyModel
{
public:
	cHosekSkyModel();
	~cHosekSkyModel();

	static void CalculateXYZCoefandLM(const double turbidity, 
										const double albedo, 
										const double elevation,
										double coefs[3][9], 
										double radiance[3]);
private:
	static const double PI;

	struct tSkyState
	{
		double mConfig[3][9];
		double mRadiance[3];
	};

	static void XYZSkyStateInit(const double turbidity, 
									const double albedo, 
									const double elevation,
									tSkyState& state);

	static double XYZSkyStateRadiance(tSkyState* state,
										double theta,
										double gamma,
										int channel);

	static void HosekSkyCookConfig(double*       dataset, 
									double  config[9], 
									double        turbidity, 
									double        albedo, 
									double        solar_elevation);

	static double HosekSkyCookRadianceConfig(double*	dataset, 
												double  turbidity, 
												double  albedo, 
												double  solar_elevation);

	static double HosekSkyRadiance(double config[9], 
									double theta, 
									double gamma);
};

#endif //__HOSEKSKYMODEL_H