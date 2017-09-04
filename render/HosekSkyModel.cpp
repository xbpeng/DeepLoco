#include "HosekSkyModel.h"
#include "HosekSkyModelData.h"
#include <cmath>

const double cHosekSkyModel::PI = 3.14159265359;

cHosekSkyModel::cHosekSkyModel()
{
}

cHosekSkyModel::~cHosekSkyModel()
{
}

void cHosekSkyModel::CalculateXYZCoefandLM(const double turbidity, 
											const double albedo, 
											const double elevation,
											double coefs[3][9], 
											double radiance[3])
{
	tSkyState state;
	XYZSkyStateInit(turbidity, albedo, elevation, state);

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 9; ++j)
		{
			coefs[i][j] = state.mConfig[i][j];
		}
		radiance[i] = state.mRadiance[i]; 
	}
}

void cHosekSkyModel::XYZSkyStateInit(const double turbidity, 
										const double albedo, 
										const double elevation,
										tSkyState& state)
{
    for(unsigned int channel = 0; channel < 3; ++channel)
    {
        HosekSkyCookConfig(gDatasetsXYZ[channel], 
							state.mConfig[channel], 
							turbidity, 
							albedo, 
							elevation);
        
        state.mRadiance[ channel ] = HosekSkyCookRadianceConfig(gDatasetsXYZRad[channel],
																	turbidity, 
																	albedo,
																	elevation);
    }
}

double cHosekSkyModel::XYZSkyStateRadiance(tSkyState* state,
											double theta,
											double gamma,
											int channel)
{
	return HosekSkyRadiance(state->mConfig[channel], 
							theta, 
							gamma) * state->mRadiance[channel];
}

void cHosekSkyModel::HosekSkyCookConfig(double*       dataset, 
										double		  config[9], 
										double        turbidity, 
										double        albedo, 
										double        solar_elevation)
{
	double  * elev_matrix;

    int     int_turbidity = turbidity;
    double  turbidity_rem = turbidity - (double)int_turbidity;

    solar_elevation = pow(solar_elevation / (PI / 2.0), (1.0 / 3.0));

    // alb 0 low turb
    elev_matrix = dataset + (9 * 6 * (int_turbidity-1));
    double one_minus_elevation = 1.0 - solar_elevation;
    
    for(unsigned int i = 0; i < 9; ++i)
    {
        //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
        config[i] = 
        (1.0-albedo) * (1.0 - turbidity_rem) 
        * (pow(one_minus_elevation, 5.0) * elev_matrix[i]  + 
           5.0  * pow(one_minus_elevation, 4.0) * solar_elevation * elev_matrix[i+9] +
           10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[i+18] +
           10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[i+27] +
           5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[i+36] +
           pow(solar_elevation, 5.0)  * elev_matrix[i+45]);
    }

    // alb 1 low turb
    elev_matrix = dataset + (9*6*10 + 9*6*(int_turbidity-1));
    for(unsigned int i = 0; i < 9; ++i)
    {
        //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
        config[i] += 
        (albedo) * (1.0 - turbidity_rem)
        * (pow(one_minus_elevation, 5.0) * elev_matrix[i]  + 
           5.0  * pow(one_minus_elevation, 4.0) * solar_elevation * elev_matrix[i+9] +
           10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[i+18] +
           10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[i+27] +
           5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[i+36] +
           pow(solar_elevation, 5.0)  * elev_matrix[i+45]);
    }

    if(int_turbidity == 10)
        return;

    // alb 0 high turb
    elev_matrix = dataset + (9*6*(int_turbidity));
    for(unsigned int i = 0; i < 9; ++i)
    {
        //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
        config[i] += 
        (1.0-albedo) * (turbidity_rem)
        * (pow(one_minus_elevation, 5.0) * elev_matrix[i]  + 
           5.0  * pow(one_minus_elevation, 4.0) * solar_elevation * elev_matrix[i+9] +
           10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[i+18] +
           10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[i+27] +
           5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[i+36] +
           pow(solar_elevation, 5.0)  * elev_matrix[i+45]);
    }

    // alb 1 high turb
    elev_matrix = dataset + (9*6*10 + 9*6*(int_turbidity));
    for(unsigned int i = 0; i < 9; ++i)
    {
        //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
        config[i] += 
        (albedo) * (turbidity_rem)
        * (pow(one_minus_elevation, 5.0) * elev_matrix[i]  + 
           5.0  * pow(one_minus_elevation, 4.0) * solar_elevation * elev_matrix[i+9] +
           10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[i+18] +
           10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[i+27] +
           5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[i+36] +
           pow(solar_elevation, 5.0)  * elev_matrix[i+45]);
    }
}

double cHosekSkyModel::HosekSkyCookRadianceConfig(double*	dataset, 
													double  turbidity, 
													double  albedo, 
													double  solar_elevation)
{
	double* elev_matrix;

    int int_turbidity = turbidity;
    double turbidity_rem = turbidity - (double)int_turbidity;
    double res;
    solar_elevation = pow(solar_elevation / (PI / 2.0), (1.0 / 3.0));

	double one_minus_elevation = 1.0 - solar_elevation;

    // alb 0 low turb
    elev_matrix = dataset + (6*(int_turbidity-1));
    //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
    res = (1.0-albedo) * (1.0 - turbidity_rem) *
        (pow(one_minus_elevation, 5.0) * elev_matrix[0] +
         5.0*pow(one_minus_elevation, 4.0)*solar_elevation * elev_matrix[1] +
         10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[2] +
         10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[3] +
         5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[4] +
         pow(solar_elevation, 5.0) * elev_matrix[5]);

    // alb 1 low turb
    elev_matrix = dataset + (6*10 + 6*(int_turbidity-1));
    //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
    res += (albedo) * (1.0 - turbidity_rem) *
        (pow(one_minus_elevation, 5.0) * elev_matrix[0] +
         5.0*pow(one_minus_elevation, 4.0)*solar_elevation * elev_matrix[1] +
         10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[2] +
         10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[3] +
         5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[4] +
         pow(solar_elevation, 5.0) * elev_matrix[5]);
    if(int_turbidity == 10)
        return res;

    // alb 0 high turb
    elev_matrix = dataset + (6*(int_turbidity));
    //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
    res += (1.0-albedo) * (turbidity_rem) *
        (pow(one_minus_elevation, 5.0) * elev_matrix[0] +
         5.0*pow(one_minus_elevation, 4.0)*solar_elevation * elev_matrix[1] +
         10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[2] +
         10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[3] +
         5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[4] +
         pow(solar_elevation, 5.0) * elev_matrix[5]);

    // alb 1 high turb
    elev_matrix = dataset + (6*10 + 6*(int_turbidity));
    //(1-t).^3* A1 + 3*(1-t).^2.*t * A2 + 3*(1-t) .* t .^ 2 * A3 + t.^3 * A4;
    res += (albedo) * (turbidity_rem) *
        (pow(one_minus_elevation, 5.0) * elev_matrix[0] +
         5.0*pow(one_minus_elevation, 4.0)*solar_elevation * elev_matrix[1] +
         10.0*pow(one_minus_elevation, 3.0)*pow(solar_elevation, 2.0) * elev_matrix[2] +
         10.0*pow(one_minus_elevation, 2.0)*pow(solar_elevation, 3.0) * elev_matrix[3] +
         5.0*(one_minus_elevation)*pow(solar_elevation, 4.0) * elev_matrix[4] +
         pow(solar_elevation, 5.0) * elev_matrix[5]);
    return res;
}

double cHosekSkyModel::HosekSkyRadiance(double config[9], 
											double theta, 
											double gamma)
{
	const double A = config[0];
	const double B = config[1];
	const double C = config[2];
	const double D = config[3];
	const double E = config[4];
	const double F = config[5];
	const double G = config[6];
	const double H = config[7];
	const double I = config[8];

	const double cos_gamma = cos(gamma);
	const double cos_theta = cos(theta);

	const double expM = exp(E * gamma);
    const double rayM = cos_gamma * cos_gamma;
    const double mieM = (1.0 + cos_gamma * cos_gamma) / pow((1.0 + I * I - 2.0* I * cos_gamma), 1.5);
    const double zenith = sqrt(cos_theta);

    return (1.0 + A * exp(B / (cos_theta + 0.01))) *
            (C + D * expM + F * rayM + G * mieM + H * zenith);
}