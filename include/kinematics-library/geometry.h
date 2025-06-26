#ifndef KINEMATICS_LIBRARY_GEOMETRY_H_
#define KINEMATICS_LIBRARY_GEOMETRY_H_

#include <kinematics-library/kinematic_vector.h>

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 * @brief Calculates the LLA point B that is a fixed range and bearing from LLA point A. This function uses an iterative
 * solution to determine outputs using the WGS84 ellipsoidal Earth model. See reference:
 * https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
 *
 * @param lla_a Latitude-longitude-altitude point A [radians-radians-meters].
 * @param range_m Range (i.e., distance) from point A to point B [meters].
 * @param bearing_rad Bearing (i.e., azimuth) from point A to point B relative to true north [radians].
 * @param abs_tol Absolute tolerance used for convergence.
 * @param lla_b Latitude-longitude-altitude point B [radians-radians-meters].
 * @return Function execution status.
 **********************************************************************************************************************/
int vincenty_direct(
    kvector_t const *const lla_a,
    precision_type_t const range_m,
    precision_type_t const bearing_rad,
    precision_type_t const abs_tol,
    kvector_t *const       lla_b);

/***********************************************************************************************************************
 * @brief Calculates range and azimuths between to LLA points. This function uses an iterative solution to determine
 * outputs using the WGS84 ellipsoidal Earth model. See reference: https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
 *
 * @param lla_a Latitude-longitude-altitude point A [radians-radians-meters].
 * @param lla_b Latitude-longitude-altitude point B [radians-radians-meters].
 * @param abs_tol Absolute tolerance used for convergence.
 * @param range_m Range (i.e., distance) from point A to point B [meters].
 * @param bearing_ab_rad Bearing (i.e., azimuth) from point A to point B relative to true north [radians].
 * @param bearing_ba_rad Bearing (i.e., azimuth) from point B to point A relative to true north [radians].
 * @return Function execution status.
 **********************************************************************************************************************/
int vincenty_inverse(
    kvector_t const *const  lla_a,
    kvector_t const *const  lla_b,
    precision_type_t const  abs_tol,
    precision_type_t *const range_m,
    precision_type_t *const bearing_ab_rad,
    precision_type_t *const bearing_ba_rad);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_LIBRARY_GEOMETRY_H_
