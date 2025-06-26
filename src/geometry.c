#include <float.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/geometry.h>
#include <kinematics-library/kinematic_vector.h>
#include <math.h>
#include <stddef.h>
#include <universal-constants/constants.h>

#define VINCENTY_INVERSE_SIN_SIGMA_TOL 1.0e-10
#define VINCENTY_INVERSE_COS2A_TOL     1.0e-10

static inline int _valid_lat_lon(kvector_t const *const lla)
{
    if (fabs(lla->x) > 90.0)
    {
        return 0;
    }

    if (fabs(lla->y) > 180.0)
    {
        return 0;
    }

    return 1;
}

static inline void wrap_to_pi(precision_type_t *const angle)
{
    *angle = atan2(sin(*angle), cos(*angle));
}

int vincenty_direct(
    kvector_t const *const lla_a,
    precision_type_t const range_m,
    precision_type_t const bearing_rad,
    precision_type_t const abs_tol,
    kvector_t *const       lla_b)
{
    if (lla_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (_valid_lat_lon(lla_a) == 0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (range_m < 0.0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (abs_tol < 0.0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (lla_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t f    = 1.0 - EARTH_R_POLAR_M / EARTH_R_EQ_M;
    const precision_type_t phi1 = lla_a->x;

    const precision_type_t u1 = atan((1 - f) * tan(phi1));
    const precision_type_t l1 = lla_a->y;

    const precision_type_t s1    = atan2(tan(u1), cos(bearing_rad));
    const precision_type_t sina  = cos(u1) * sin(bearing_rad);
    const precision_type_t cos2a = 1 - pow(sina, 2.0);
    const precision_type_t u2    = cos2a * (EARTH_R_EQ2_M2 - EARTH_R_POLAR2_M2) / EARTH_R_POLAR2_M2;

    const precision_type_t k1 = (sqrt(1.0 + u2) - 1) / (sqrt(1.0 + u2) + 1.0);
    const precision_type_t a  = (1 + 0.25 * pow(k1, 2.0)) / (1 - k1);
    const precision_type_t b  = k1 * (1.0 - 3. / 8. * pow(k1, 2.0));

    precision_type_t sigma = range_m / (EARTH_R_POLAR_M * a);

    uint16_t         steps = 0;
    precision_type_t twosigm, cos2sm, sins, coss, delta_sigma, sigma_old;
    while (1)
    {
        twosigm = 2 * s1 + sigma;
        cos2sm  = cos(twosigm);
        sins    = sin(sigma);
        coss    = cos(sigma);

        precision_type_t delta_sigma_term_1 = coss * (-1.0 + 2.0 * pow(cos2sm, 2.0));
        precision_type_t delta_sigma_term_2 = (-3.0 + 4.0 * pow(sins, 2.0)) * (-3.0 + 4.0 * pow(cos2sm, 2.0));
        precision_type_t delta_sigma_term_3 = b / 6.0 * cos2sm * delta_sigma_term_2;
        precision_type_t delta_sigma_term_4 = delta_sigma_term_1 - delta_sigma_term_3;
        delta_sigma                         = b * sins * (cos2sm + 0.25 * b * delta_sigma_term_4);
        sigma_old                           = sigma;

        sigma = range_m / (EARTH_R_POLAR_M * a) + delta_sigma;
        steps += 1;
        if ((fabs(sigma_old - sigma) < abs_tol) || (steps > 2000))
        {
            break;
        }
    }

    sins    = sin(sigma);
    coss    = cos(sigma);
    twosigm = 2 * s1 + sigma;
    cos2sm  = cos(twosigm);

    const precision_type_t phi2_term_1 = sin(u1) * coss + cos(u1) * sins * cos(bearing_rad);
    const precision_type_t phi2_term_2 = pow(sin(u1) * sins - cos(u1) * coss * cos(bearing_rad), 2.0);
    const precision_type_t phi2_term_3 = (1 - f) * sqrt(pow(sina, 2.0) + phi2_term_2);
    const precision_type_t phi2        = atan2(phi2_term_1, phi2_term_3);

    const precision_type_t lam = atan2(sins * sin(bearing_rad), cos(u1) * coss - sin(u1) * sins * cos(bearing_rad));
    const precision_type_t c   = f / 16.0 * cos2a * (4.0 + f * (4.0 - 3.0 * cos2a));

    const precision_type_t l_term_1 = (sigma + c * sins * (cos2sm + c * coss * (-1.0 + 2.0 * pow(cos2sm, 2.0))));
    const precision_type_t l        = lam - (1 - c) * f * sina * l_term_1;
    const precision_type_t L2       = l + l1;

    kvector_init(lla_b, phi2, L2, 0.0);

    return KL_NO_ERROR;
}

int vincenty_inverse(
    kvector_t const *const  lla_a,
    kvector_t const *const  lla_b,
    precision_type_t const  abs_tol,
    precision_type_t *const range_m,
    precision_type_t *const bearing_ab_rad,
    precision_type_t *const bearing_ba_rad)
{
    if (lla_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (_valid_lat_lon(lla_a) == 0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (lla_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (_valid_lat_lon(lla_b) == 0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (abs_tol < 0.0)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    if (range_m == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (bearing_ab_rad == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (bearing_ba_rad == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t f = 1.0 - EARTH_R_POLAR_M / EARTH_R_EQ_M;

    const precision_type_t phi1 = lla_a->x;
    const precision_type_t phi2 = lla_b->x;

    const precision_type_t u1     = atan((1.0 - f) * tan(phi1));
    const precision_type_t u1_sin = sin(u1);
    const precision_type_t u1_cos = cos(u1);

    const precision_type_t u2     = atan((1.0 - f) * tan(phi2));
    const precision_type_t u2_sin = sin(u2);
    const precision_type_t u2_cos = cos(u2);

    const precision_type_t l1 = lla_a->y;
    const precision_type_t l2 = lla_b->y;
    const precision_type_t l  = l2 - l1;

    // Early termination if the provided LLA points are coincident.
    if ((fabs(lla_a->x - lla_b->x) < VINCENTY_INVERSE_SIN_SIGMA_TOL) && (fabs(l) < VINCENTY_INVERSE_SIN_SIGMA_TOL))
    {
        *range_m        = 0.0;
        *bearing_ab_rad = 0.0;
        *bearing_ba_rad = 0.0;
        return KL_NO_ERROR;
    }

    uint16_t         steps = 0;
    precision_type_t lam   = l2 - l1;
    precision_type_t cos2a, cos2sm, sigma, sin_sigma, cos_sigma, sina, c, lam_old;
    while (1)
    {
        precision_type_t sin_sigma_term_1 = pow(u2_cos * sin(lam), 2.0);
        precision_type_t sin_sigma_term_2 = pow(u1_cos * u2_sin - u1_sin * u2_cos * cos(lam), 2.0);

        sin_sigma = sqrt(sin_sigma_term_1 + sin_sigma_term_2);
        if (sin_sigma < VINCENTY_INVERSE_SIN_SIGMA_TOL)
        {
            if (fabs(lla_a->x - lla_b->x) < VINCENTY_INVERSE_SIN_SIGMA_TOL)
            {
                // Handles when points that share common latitudes converge to be coincident during iteration.
                *range_m        = 0.0;
                *bearing_ab_rad = 0.0;
                *bearing_ba_rad = 0.0;
                return KL_NO_ERROR;
            }
            else
            {
                // Else the points are anti-podal and a small perturbation is added to avoid divide by zero.
                sin_sigma = VINCENTY_INVERSE_SIN_SIGMA_TOL;
            }
        }

        cos_sigma = u1_sin * u2_sin + u1_cos * u2_cos * cos(lam);
        sigma     = atan2(sin_sigma, cos_sigma);
        sina      = u1_cos * u2_cos * sin(lam) / sin(sigma);

        // Handle case where both points are on the equator, yeilding a divide by zero error.
        cos2a = 1.0 - pow(sina, 2.0);
        if (fabs(cos2a) < VINCENTY_INVERSE_COS2A_TOL)
        {
            cos2sm = 0.0;
        }
        else
        {
            cos2sm = cos_sigma - 2.0 * u1_sin * u2_sin / cos2a;
        }

        c       = f / 16.0 * cos2a * (4.0 + f * (4.0 - 3.0 * cos2a));
        lam_old = lam;

        precision_type_t lam_end_term = cos2sm + c * cos_sigma * (-1.0 + 2.0 * pow(cos2sm, 2.0));

        lam = l + (1 - c) * f * sina * (sigma + c * sin_sigma * lam_end_term);
        steps += 1;
        if ((fabs(lam_old - lam) < abs_tol) || (steps > 200))
        {
            break;
        }
    }

    const precision_type_t g2 = cos2a * (pow(EARTH_R_EQ_M, 2.0) - EARTH_R_POLAR2_M2) / EARTH_R_POLAR2_M2;
    const precision_type_t k1 = (sqrt(1.0 + g2) - 1) / (sqrt(1.0 + g2) + 1.0);
    const precision_type_t a  = (1 + 0.25 * pow(k1, 2.0)) / (1 - k1);
    const precision_type_t b  = k1 * (1.0 - 3. / 8. * pow(k1, 2.0));

    const precision_type_t delta_sigma_term_1 = cos_sigma * (-1.0 + 2.0 * pow(cos2sm, 2.0));
    const precision_type_t delta_sigma_term_2 = (-3.0 + 4.0 * pow(sin_sigma, 2.0)) * (-3.0 + 4.0 * pow(cos2sm, 2.0));
    const precision_type_t delta_sigma_term_3 = (delta_sigma_term_1 - b / 6.0 * cos2sm * delta_sigma_term_2);
    const precision_type_t delta_sigma        = b * sin_sigma * (cos2sm + 0.25 * b * delta_sigma_term_3);

    // For clarity, pi is added to the bearing from B to A so that the angle return is the azimuth from B to A, rather
    // than the forward azimuth from A's perspective. However, to keep both angles in the same domain (+/- pi), the
    // modified angle is wrapped.
    *range_m        = EARTH_R_POLAR_M * a * (sigma - delta_sigma);
    *bearing_ab_rad = atan2(u2_cos * sin(lam), u1_cos * u2_sin - u1_sin * u2_cos * cos(lam));
    *bearing_ba_rad = atan2(u1_cos * sin(lam), -u1_sin * u2_cos + u1_cos * u2_sin * cos(lam)) + PI;
    wrap_to_pi(bearing_ba_rad);

    return KL_NO_ERROR;
}
