#include <float.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <math.h>
#include <stddef.h>

#define DBL_TINY 1.0E-15

int kvector_add(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result)
{
    if (kv_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv_a->x + kv_b->x;
    result->y = kv_a->y + kv_b->y;
    result->z = kv_a->z + kv_b->z;

    return KL_NO_ERROR;
}

int kvector_addf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x + scalar;
    result->y = kv->y + scalar;
    result->z = kv->z + scalar;

    return KL_NO_ERROR;
}

int kvector_angle_between(const kvector_t *kv_a, const kvector_t *kv_b, precision_type_t *result)
{
    if (kv_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t dot;
    precision_type_t mag_a;
    precision_type_t mag_b;
    int              error;

    error = kvector_dot(kv_a, kv_b, &dot);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    error = kvector_magnitude(kv_a, &mag_a);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    error = kvector_magnitude(kv_b, &mag_b);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    (*result) = acos(dot / ((mag_a + DBL_TINY) * (mag_b + DBL_TINY)));

    return KL_NO_ERROR;
}

int kvector_azimuth_angle(const kvector_t *kv, precision_type_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    (*result) = atan2(kv->y, kv->x);

    return KL_NO_ERROR;
}

int kvector_cross(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result)
{
    if (kv_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = (kv_a->y * kv_b->z) - (kv_a->z * kv_b->y);
    result->y = (kv_a->z * kv_b->x) - (kv_a->x * kv_b->z);
    result->z = (kv_a->x * kv_b->y) - (kv_a->y * kv_b->x);

    return KL_NO_ERROR;
}

int kvector_compare(const kvector_t *kv_a, const kvector_t *kv_b)
{
    if (kv_a == NULL && kv_b == NULL)
    {
        return 0;
    }

    if (kv_a == NULL && kv_b != NULL)
    {
        return 1;
    }

    if (kv_a != NULL && kv_b == NULL)
    {
        return 1;
    }

    if (kv_a->x == kv_b->x && kv_a->y == kv_b->y && kv_a->z == kv_b->z)
    {
        return 0;
    }

    return 1;
}

int kvector_divf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x / scalar;
    result->y = kv->y / scalar;
    result->z = kv->z / scalar;

    return KL_NO_ERROR;
}

int kvector_divf2(const precision_type_t scalar, const kvector_t *kv, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = scalar / kv->x;
    result->y = scalar / kv->y;
    result->z = scalar / kv->z;

    return KL_NO_ERROR;
}

int kvector_dot(const kvector_t *kv_a, const kvector_t *kv_b, precision_type_t *result)
{
    if (kv_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    (*result) = (kv_a->x * kv_b->x) + (kv_a->y * kv_b->y) + (kv_a->z * kv_b->z);

    return KL_NO_ERROR;
}

int kvector_elevation_angle(const kvector_t *kv, precision_type_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t mag;
    int              error;
    error = kvector_magnitude(kv, &mag);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    (*result) = asin(-kv->z / (mag + DBL_TINY));

    return KL_NO_ERROR;
}

int kvector_init(kvector_t *kv, const precision_type_t x, const precision_type_t y, const precision_type_t z)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kv->x = x;
    kv->y = y;
    kv->z = z;

    return KL_NO_ERROR;
}

int kvector_magnitude(const kvector_t *kv, precision_type_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    (*result) = sqrt((kv->x * kv->x) + (kv->y * kv->y) + (kv->z * kv->z));

    return KL_NO_ERROR;
}

int kvector_mulf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x * scalar;
    result->y = kv->y * scalar;
    result->z = kv->z * scalar;

    return KL_NO_ERROR;
}

int kvector_polar_angle(const kvector_t *kv, precision_type_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t mag;
    int              error;
    error = kvector_magnitude(kv, &mag);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    (*result) = acos(kv->z / (mag + DBL_TINY));

    return KL_NO_ERROR;
}

int kvector_sub(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result)
{
    if (kv_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv_a->x - kv_b->x;
    result->y = kv_a->y - kv_b->y;
    result->z = kv_a->z - kv_b->z;

    return KL_NO_ERROR;
}

int kvector_subf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x - scalar;
    result->y = kv->y - scalar;
    result->z = kv->z - scalar;

    return KL_NO_ERROR;
}

int kvector_subf2(const precision_type_t scalar, const kvector_t *kv, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = scalar - kv->x;
    result->y = scalar - kv->y;
    result->z = scalar - kv->z;

    return KL_NO_ERROR;
}

int kvector_unit(const kvector_t *kv, kvector_t *result)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t mag;
    int              error;
    error = kvector_magnitude(kv, &mag);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    if (mag > 0.0)
    {
        result->x = kv->x / mag;
        result->y = kv->y / mag;
        result->z = kv->z / mag;
    }

    return KL_NO_ERROR;
}

int kvector_zero(kvector_t *kv)
{
    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kv->x = 0.0;
    kv->y = 0.0;
    kv->z = 0.0;

    return KL_NO_ERROR;
}
