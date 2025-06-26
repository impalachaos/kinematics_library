#include <float.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/transforms.h>
#include <math.h>
#include <stddef.h>
#include <universal-constants/constants.h>

int transform_aer_to_ned(const kvector_t *aer, kvector_t *ned)
{
    if (aer == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t azimuth               = aer->x;
    const precision_type_t elevation             = aer->y;
    const precision_type_t range                 = aer->z;
    const precision_type_t north_east_projection = range * cos(elevation);

    ned->x = north_east_projection * cos(azimuth);
    ned->y = north_east_projection * sin(azimuth);
    ned->z = -range * sin(elevation);

    return KL_NO_ERROR;
}

int transform_ecef_to_lla(const kvector_t *ecef, kvector_t *lla)
{
    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t r  = sqrt(ecef->x * ecef->x + ecef->y * ecef->y);
    precision_type_t r2 = r * r;
    precision_type_t z2 = ecef->z * ecef->z;
    precision_type_t f  = 54.0 * EARTH_R_POLAR2_M2 * z2;
    precision_type_t g  = r2 + (1.0 - EARTH_ECC2) * z2 - EARTH_ECC2 * (EARTH_R_EQ2_M2 - EARTH_R_POLAR2_M2);
    precision_type_t c  = EARTH_ECC4 * f * r2 / (g * g * g);
    precision_type_t s  = pow(1.0 + c + sqrt(c * c + 2 * c), 1.0 / 3.0);
    precision_type_t p  = f / (3.0 * pow(s + 1.0 / s + 1.0, 2.0) * g * g);
    precision_type_t q  = sqrt(1.0 + 2.0 * EARTH_ECC4 * p);

    precision_type_t r0_term1 = -p * EARTH_ECC2 * r / (1.0 + q);
    precision_type_t r0_term2 = 0.5 * EARTH_R_EQ2_M2 * (1.0 + 1.0 / q);
    precision_type_t r0_term3 = p * (1.0 - EARTH_ECC2) * z2 / (q * (1.0 + q));
    precision_type_t r0_term4 = 0.5 * p * r2;
    precision_type_t r0       = r0_term1 + sqrt(r0_term2 - r0_term3 - r0_term4);

    precision_type_t u  = sqrt(pow(r - EARTH_ECC2 * r0, 2.0) + z2);
    precision_type_t v  = sqrt(pow(r - EARTH_ECC2 * r0, 2.0) + (1.0 - EARTH_ECC2) * z2);
    precision_type_t z0 = EARTH_R_POLAR2_M2 * ecef->z / (EARTH_R_EQ_M * v);

    precision_type_t alt_m   = u * (1.0 - EARTH_R_POLAR2_M2 / (EARTH_R_EQ_M * v));
    precision_type_t lat_rad = atan((ecef->z + EARTH_ECC2_P * z0) / r);
    precision_type_t lon_rad = atan2(ecef->y, ecef->x);

    lla->x = lat_rad;
    lla->y = lon_rad;
    lla->z = alt_m;

    return KL_NO_ERROR;
}

int transform_ecef_to_ned(const kvector_t *ecef, const kvector_t *lla_ref, kvector_t *ned)
{
    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kvector_t ecef_ref;
    int       error;

    error = transform_lla_to_ecef(lla_ref, &ecef_ref);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    error = kvector_sub(ecef, &ecef_ref, &ecef_ref);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    return transform_ecef_to_ned_local(&ecef_ref, lla_ref, ned);
}

int transform_ecef_to_ned_local(const kvector_t *ecef, const kvector_t *lla_ref, kvector_t *ned)
{
    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t lat_rad = lla_ref->x;
    const precision_type_t lon_rad = lla_ref->y;
    const precision_type_t sin_lat = sin(lat_rad);
    const precision_type_t cos_lat = cos(lat_rad);
    const precision_type_t sin_lon = sin(lon_rad);
    const precision_type_t cos_lon = cos(lon_rad);

    ned->x = -sin_lat * cos_lon * ecef->x - sin_lat * sin_lon * ecef->y + cos_lat * ecef->z;
    ned->y = -sin_lon * ecef->x + cos_lon * ecef->y;
    ned->z = -cos_lat * cos_lon * ecef->x - cos_lat * sin_lon * ecef->y - sin_lat * ecef->z;

    return KL_NO_ERROR;
}

int transform_lla_to_ecef(const kvector_t *lla, kvector_t *ecef)
{
    if (lla == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t lat_rad  = lla->x;
    const precision_type_t lon_rad  = lla->y;
    const precision_type_t alt_m    = lla->z;
    const precision_type_t n        = EARTH_R_EQ_M / sqrt(1.0 - EARTH_ECC2 * pow(sin(lat_rad), 2.0));
    const precision_type_t xy_coeff = (n + alt_m) * cos(lat_rad);

    ecef->x = xy_coeff * cos(lon_rad);
    ecef->y = xy_coeff * sin(lon_rad);
    ecef->z = (n * (1.0 - EARTH_ECC2) + alt_m) * sin(lat_rad);

    return KL_NO_ERROR;
}

int transform_lla_to_ned(const kvector_t *lla, const kvector_t *lla_ref, kvector_t *ned)
{
    if (lla == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kvector_t ecef;
    int       error;

    error = transform_lla_to_ecef(lla, &ecef);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    return transform_ecef_to_ned(&ecef, lla_ref, ned);
}

int transform_ned_to_aer(const kvector_t *ned, kvector_t *aer)
{
    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (aer == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t north_east_projection = sqrt(pow(ned->x, 2.0) + pow(ned->y, 2.0));

    precision_type_t range;
    int              error;
    error = kvector_magnitude(ned, &range);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    aer->x = atan2(ned->y, ned->x);
    aer->y = atan2(-ned->z, north_east_projection);
    aer->z = range;

    return KL_NO_ERROR;
}

int transform_ned_to_ecef(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *ecef)
{
    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kvector_t ecef_local, ecef_ref;
    int       error;

    error = transform_ned_to_ecef_local(ned, lla_ref, &ecef_local);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    error = transform_lla_to_ecef(lla_ref, &ecef_ref);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    return kvector_add(&ecef_local, &ecef_ref, ecef);
}

int transform_ned_to_ecef_local(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *ecef)
{
    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (ecef == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t sin_lat = sin(lla_ref->x);
    const precision_type_t cos_lat = cos(lla_ref->x);
    const precision_type_t sin_lon = sin(lla_ref->y);
    const precision_type_t cos_lon = cos(lla_ref->y);

    ecef->x = -sin_lat * cos_lon * ned->x - sin_lon * ned->y - cos_lat * cos_lon * ned->z;
    ecef->y = -sin_lat * sin_lon * ned->x + cos_lon * ned->y - cos_lat * sin_lon * ned->z;
    ecef->z = cos_lat * ned->x - sin_lat * ned->z;

    return KL_NO_ERROR;
}

int transform_ned_to_lla(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *lla)
{
    if (ned == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla_ref == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (lla == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kvector_t ecef;
    int       error;

    error = transform_ned_to_ecef(ned, lla_ref, &ecef);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    return transform_ecef_to_lla(&ecef, lla);
}
