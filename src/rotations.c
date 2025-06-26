#include <float.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/rotations.h>
#include <math.h>
#include <stddef.h>

int dcm_compare(const dcm_t *dcm_a, const dcm_t *dcm_b)
{
    if (dcm_a == NULL && dcm_b == NULL)
    {
        return 0;
    }

    if (dcm_a == NULL && dcm_b != NULL)
    {
        return 1;
    }

    if (dcm_a != NULL && dcm_b == NULL)
    {
        return 1;
    }

    if (dcm_a->e00 == dcm_b->e00 && dcm_a->e01 == dcm_b->e01 && dcm_a->e02 == dcm_b->e02 && dcm_a->e10 == dcm_b->e10 &&
        dcm_a->e11 == dcm_b->e11 && dcm_a->e12 == dcm_b->e12 && dcm_a->e20 == dcm_b->e20 && dcm_a->e21 == dcm_b->e21 &&
        dcm_a->e22 == dcm_b->e22)
    {
        return 0;
    }
    return 1;
}

int dcm_init(
    dcm_t                 *dcm,
    const precision_type_t a1_rad,
    const precision_type_t a2_rad,
    const precision_type_t a3_rad,
    const int              seq)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (seq < ROTATION_SEQ_BODY_XYX || seq > ROTATION_SEQ_BODY_ZYZ)
    {
        return KL_ERROR_INVALID_ARGUMENT;
    }

    // The choice to explicitly define functions for every rotation sequence is deliberate. It is computationally more
    // efficient to to only do the exact calculations necessary to calculate the direction cosine matrices. The obvious
    // downside to this approach is that there are a lot of rotation sequence combinations.

    switch (seq)
    {
        case ROTATION_SEQ_BODY_XYX:
            dcm->e00 = cos(a2_rad);
            dcm->e01 = sin(a1_rad) * sin(a2_rad);
            dcm->e02 = -cos(a1_rad) * sin(a2_rad);
            dcm->e10 = sin(a3_rad) * sin(a2_rad);
            dcm->e11 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            dcm->e12 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * cos(a2_rad) * sin(a3_rad);
            dcm->e20 = cos(a3_rad) * sin(a2_rad);
            dcm->e21 = -cos(a1_rad) * sin(a3_rad) - cos(a3_rad) * cos(a2_rad) * sin(a1_rad);
            dcm->e22 = cos(a1_rad) * cos(a3_rad) * cos(a2_rad) - sin(a1_rad) * sin(a3_rad);
            break;

        case ROTATION_SEQ_BODY_XYZ:
            dcm->e00 = cos(a3_rad) * cos(a2_rad);
            dcm->e01 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a1_rad) * sin(a2_rad);
            dcm->e02 = sin(a1_rad) * sin(a3_rad) - cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            dcm->e10 = -cos(a2_rad) * sin(a3_rad);
            dcm->e11 = cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad) * sin(a2_rad);
            dcm->e12 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a3_rad) * sin(a2_rad);
            dcm->e20 = sin(a2_rad);
            dcm->e21 = -cos(a2_rad) * sin(a1_rad);
            dcm->e22 = cos(a1_rad) * cos(a2_rad);
            break;

        case ROTATION_SEQ_BODY_XZX:
            dcm->e00 = cos(a2_rad);
            dcm->e01 = cos(a1_rad) * sin(a2_rad);
            dcm->e02 = sin(a2_rad) * sin(a1_rad);
            dcm->e10 = -cos(a3_rad) * sin(a2_rad);
            dcm->e11 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            dcm->e12 = cos(a1_rad) * sin(a3_rad) + cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            dcm->e20 = sin(a2_rad) * sin(a3_rad);
            dcm->e21 = -cos(a3_rad) * sin(a1_rad) - cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            dcm->e22 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            break;

        case ROTATION_SEQ_BODY_XZY:
            dcm->e00 = cos(a2_rad) * cos(a3_rad);
            dcm->e01 = sin(a1_rad) * sin(a3_rad) + cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            dcm->e02 = cos(a3_rad) * sin(a1_rad) * sin(a2_rad) - cos(a1_rad) * sin(a3_rad);
            dcm->e10 = -sin(a2_rad);
            dcm->e11 = cos(a1_rad) * cos(a2_rad);
            dcm->e12 = cos(a2_rad) * sin(a1_rad);
            dcm->e20 = cos(a2_rad) * sin(a3_rad);
            dcm->e21 = cos(a1_rad) * sin(a2_rad) * sin(a3_rad) - cos(a3_rad) * sin(a1_rad);
            dcm->e22 = cos(a1_rad) * cos(a3_rad) + sin(a1_rad) * sin(a2_rad) * sin(a3_rad);
            break;

        case ROTATION_SEQ_BODY_YXY:
            dcm->e00 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            dcm->e01 = sin(a2_rad) * sin(a3_rad);
            dcm->e02 = -cos(a3_rad) * sin(a1_rad) - cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            dcm->e10 = sin(a2_rad) * sin(a1_rad);
            dcm->e11 = cos(a2_rad);
            dcm->e12 = cos(a1_rad) * sin(a2_rad);
            dcm->e20 = cos(a1_rad) * sin(a3_rad) + cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            dcm->e21 = -cos(a3_rad) * sin(a2_rad);
            dcm->e22 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            break;

        case ROTATION_SEQ_BODY_YXZ:
            dcm->e00 = cos(a3_rad) * cos(a1_rad) + sin(a2_rad) * sin(a3_rad) * sin(a1_rad);
            dcm->e01 = cos(a2_rad) * sin(a3_rad);
            dcm->e02 = cos(a1_rad) * sin(a2_rad) * sin(a3_rad) - cos(a3_rad) * sin(a1_rad);
            dcm->e10 = cos(a3_rad) * sin(a2_rad) * sin(a1_rad) - cos(a1_rad) * sin(a3_rad);
            dcm->e11 = cos(a2_rad) * cos(a3_rad);
            dcm->e12 = sin(a3_rad) * sin(a1_rad) + cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            dcm->e20 = cos(a2_rad) * sin(a1_rad);
            dcm->e21 = -sin(a2_rad);
            dcm->e22 = cos(a2_rad) * cos(a1_rad);
            break;

        case ROTATION_SEQ_BODY_YZX:
            dcm->e00 = cos(a2_rad) * cos(a1_rad);
            dcm->e01 = sin(a2_rad);
            dcm->e02 = -cos(a2_rad) * sin(a1_rad);
            dcm->e10 = sin(a3_rad) * sin(a1_rad) - cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            dcm->e11 = cos(a3_rad) * cos(a2_rad);
            dcm->e12 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a2_rad) * sin(a1_rad);
            dcm->e20 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a3_rad) * sin(a2_rad);
            dcm->e21 = -cos(a2_rad) * sin(a3_rad);
            dcm->e22 = cos(a3_rad) * cos(a1_rad) - sin(a3_rad) * sin(a2_rad) * sin(a1_rad);
            break;

        case ROTATION_SEQ_BODY_YZY:
            dcm->e00 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            dcm->e01 = cos(a3_rad) * sin(a2_rad);
            dcm->e02 = -cos(a1_rad) * sin(a3_rad) - cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            dcm->e10 = -cos(a1_rad) * sin(a2_rad);
            dcm->e11 = cos(a2_rad);
            dcm->e12 = sin(a2_rad) * sin(a1_rad);
            dcm->e20 = cos(a3_rad) * sin(a1_rad) + cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            dcm->e21 = sin(a2_rad) * sin(a3_rad);
            dcm->e22 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            break;

        case ROTATION_SEQ_BODY_ZXY:
            dcm->e00 = cos(a1_rad) * cos(a3_rad) - sin(a2_rad) * sin(a1_rad) * sin(a3_rad);
            dcm->e01 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a2_rad) * sin(a3_rad);
            dcm->e02 = -cos(a2_rad) * sin(a3_rad);
            dcm->e10 = -cos(a2_rad) * sin(a1_rad);
            dcm->e11 = cos(a2_rad) * cos(a1_rad);
            dcm->e12 = sin(a2_rad);
            dcm->e20 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a2_rad) * sin(a1_rad);
            dcm->e21 = sin(a1_rad) * sin(a3_rad) - cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            dcm->e22 = cos(a2_rad) * cos(a3_rad);
            break;

        case ROTATION_SEQ_BODY_ZXZ:
            dcm->e00 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            dcm->e01 = cos(a3_rad) * sin(a1_rad) + cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            dcm->e02 = sin(a2_rad) * sin(a3_rad);
            dcm->e10 = -cos(a1_rad) * sin(a3_rad) - cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            dcm->e11 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            dcm->e12 = cos(a3_rad) * sin(a2_rad);
            dcm->e20 = sin(a2_rad) * sin(a1_rad);
            dcm->e21 = -cos(a1_rad) * sin(a2_rad);
            dcm->e22 = cos(a2_rad);
            break;

        case ROTATION_SEQ_BODY_ZYX:
            dcm->e00 = cos(a1_rad) * cos(a2_rad);
            dcm->e01 = cos(a2_rad) * sin(a1_rad);
            dcm->e02 = -sin(a2_rad);
            dcm->e10 = cos(a1_rad) * sin(a3_rad) * sin(a2_rad) - cos(a3_rad) * sin(a1_rad);
            dcm->e11 = cos(a3_rad) * cos(a1_rad) + sin(a3_rad) * sin(a1_rad) * sin(a2_rad);
            dcm->e12 = cos(a2_rad) * sin(a3_rad);
            dcm->e20 = sin(a3_rad) * sin(a1_rad) + cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            dcm->e21 = cos(a3_rad) * sin(a1_rad) * sin(a2_rad) - cos(a1_rad) * sin(a3_rad);
            dcm->e22 = cos(a3_rad) * cos(a2_rad);
            break;

        case ROTATION_SEQ_BODY_ZYZ:
            dcm->e00 = cos(a1_rad) * cos(a3_rad) * cos(a2_rad) - sin(a1_rad) * sin(a3_rad);
            dcm->e01 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * cos(a2_rad) * sin(a1_rad);
            dcm->e02 = -cos(a3_rad) * sin(a2_rad);
            dcm->e10 = -cos(a3_rad) * sin(a1_rad) - cos(a1_rad) * cos(a2_rad) * sin(a3_rad);
            dcm->e11 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            dcm->e12 = sin(a3_rad) * sin(a2_rad);
            dcm->e20 = cos(a1_rad) * sin(a2_rad);
            dcm->e21 = sin(a1_rad) * sin(a2_rad);
            dcm->e22 = cos(a2_rad);
            break;
    }

    return KL_NO_ERROR;
}

int dcm_init_identity(dcm_t *dcm)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    dcm->e00 = 1.0;
    dcm->e01 = 0.0;
    dcm->e02 = 0.0;
    dcm->e10 = 0.0;
    dcm->e11 = 1.0;
    dcm->e12 = 0.0;
    dcm->e20 = 0.0;
    dcm->e21 = 0.0;
    dcm->e22 = 1.0;

    return KL_NO_ERROR;
}

int dcm_init_quaternion(dcm_t *dcm, const quaternion_t *q)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t a_x_a = (q->a * q->a);
    const precision_type_t a_x_b = (q->a * q->b);
    const precision_type_t a_x_c = (q->a * q->c);
    const precision_type_t a_x_d = (q->a * q->d);
    const precision_type_t b_x_c = (q->b * q->c);
    const precision_type_t b_x_d = (q->b * q->d);
    const precision_type_t c_x_d = (q->c * q->d);

    dcm->e00 = 2.0 * a_x_a - 1.0 + 2.0 * q->b * q->b;
    dcm->e01 = 2.0 * b_x_c + 2.0 * a_x_d;
    dcm->e02 = 2.0 * b_x_d - 2.0 * a_x_c;
    dcm->e10 = 2.0 * b_x_c - 2.0 * a_x_d;
    dcm->e11 = 2.0 * a_x_a - 1.0 + 2.0 * q->c * q->c;
    dcm->e12 = 2.0 * c_x_d + 2.0 * a_x_b;
    dcm->e20 = 2.0 * b_x_d + 2.0 * a_x_c;
    dcm->e21 = 2.0 * c_x_d - 2.0 * a_x_b;
    dcm->e22 = 2.0 * a_x_a - 1.0 + 2.0 * q->d * q->d;

    return KL_NO_ERROR;
}

int dcm_mul(const dcm_t *dcm_a, const dcm_t *dcm_b, dcm_t *result)
{
    if (dcm_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (dcm_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->e00 = dcm_a->e00 * dcm_b->e00 + dcm_a->e01 * dcm_b->e10 + dcm_a->e02 * dcm_b->e20;
    result->e01 = dcm_a->e00 * dcm_b->e01 + dcm_a->e01 * dcm_b->e11 + dcm_a->e02 * dcm_b->e21;
    result->e02 = dcm_a->e00 * dcm_b->e02 + dcm_a->e01 * dcm_b->e12 + dcm_a->e02 * dcm_b->e22;
    result->e10 = dcm_a->e10 * dcm_b->e00 + dcm_a->e11 * dcm_b->e10 + dcm_a->e12 * dcm_b->e20;
    result->e11 = dcm_a->e10 * dcm_b->e01 + dcm_a->e11 * dcm_b->e11 + dcm_a->e12 * dcm_b->e21;
    result->e12 = dcm_a->e10 * dcm_b->e02 + dcm_a->e11 * dcm_b->e12 + dcm_a->e12 * dcm_b->e22;
    result->e20 = dcm_a->e20 * dcm_b->e00 + dcm_a->e21 * dcm_b->e10 + dcm_a->e22 * dcm_b->e20;
    result->e21 = dcm_a->e20 * dcm_b->e01 + dcm_a->e21 * dcm_b->e11 + dcm_a->e22 * dcm_b->e21;
    result->e22 = dcm_a->e20 * dcm_b->e02 + dcm_a->e21 * dcm_b->e12 + dcm_a->e22 * dcm_b->e22;

    return KL_NO_ERROR;
}

int dcm_rotate(const dcm_t *dcm, const kvector_t *kv, kvector_t *result)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x * dcm->e00 + kv->y * dcm->e10 + kv->z * dcm->e20;
    result->y = kv->x * dcm->e01 + kv->y * dcm->e11 + kv->z * dcm->e21;
    result->z = kv->x * dcm->e02 + kv->y * dcm->e12 + kv->z * dcm->e22;

    return KL_NO_ERROR;
}

int dcm_transform(const dcm_t *dcm, const kvector_t *kv, kvector_t *result)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->x = kv->x * dcm->e00 + kv->y * dcm->e01 + kv->z * dcm->e02;
    result->y = kv->x * dcm->e10 + kv->y * dcm->e11 + kv->z * dcm->e12;
    result->z = kv->x * dcm->e20 + kv->y * dcm->e21 + kv->z * dcm->e22;

    return KL_NO_ERROR;
}

int dcm_transpose(const dcm_t *dcm, dcm_t *result)
{
    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    precision_type_t tmp;
    result->e00 = dcm->e00;
    result->e11 = dcm->e11;
    result->e22 = dcm->e22;

    tmp         = dcm->e01;
    result->e01 = dcm->e10;
    result->e10 = tmp;

    tmp         = dcm->e02;
    result->e02 = dcm->e20;
    result->e20 = tmp;

    tmp         = dcm->e12;
    result->e12 = dcm->e21;
    result->e21 = tmp;

    return KL_NO_ERROR;
}

int quaternion_angle(const quaternion_t *q, precision_type_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    (*result) = 2.0 * acos(q->a);

    return KL_NO_ERROR;
}

int quaternion_axis(const quaternion_t *q, kvector_t *kv)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    kv->x = q->b;
    kv->y = q->c;
    kv->z = q->d;

    return kvector_unit(kv, kv);
}

int quaternion_compare(const quaternion_t *q_a, const quaternion_t *q_b)
{
    if (q_a == NULL && q_b == NULL)
    {
        return 0;
    }

    if (q_a == NULL && q_b != NULL)
    {
        return 1;
    }

    if (q_a != NULL && q_b == NULL)
    {
        return 1;
    }

    if (q_a->a == q_b->a && q_a->b == q_b->b && q_a->c == q_b->c && q_a->d == q_b->d)
    {
        return 0;
    }

    return 1;
}

int quaternion_conjugate(const quaternion_t *q, quaternion_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->a = q->a;
    result->b = -q->b;
    result->c = -q->c;
    result->d = -q->d;

    return KL_NO_ERROR;
}

int quaternion_init(
    quaternion_t          *q,
    const precision_type_t a1_rad,
    const precision_type_t a2_rad,
    const precision_type_t a3_rad,
    const int              seq)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    dcm_t dcm;
    int   error;

    error = dcm_init(&dcm, a1_rad, a2_rad, a3_rad, seq);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    return quaternion_init_dcm(q, &dcm);
}

int quaternion_init_dcm(quaternion_t *q, const dcm_t *dcm)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (dcm == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    const precision_type_t trace = dcm->e00 + dcm->e11 + dcm->e22;

    // This logic is necessary to ensure that the conversion will never encounter a divide by zero error. Essentially,
    // the conversion can be done using any of the DCM diagonal components. However, nothing stops one or two of the
    // diagonal values from being zero. But, at least one component must be non-zero (property of DCM). Therefore, the
    // max diagonal diagonal is used for direction cosine matrices with negative traces.
    if (trace > 0.0)
    {
        precision_type_t s = 2.0 * sqrt(1.0 + trace);
        q->a               = 0.25 * s;
        q->b               = (dcm->e12 - dcm->e21) / s;
        q->c               = (dcm->e20 - dcm->e02) / s;
        q->d               = (dcm->e01 - dcm->e10) / s;
    }
    else if (dcm->e00 > dcm->e11 && dcm->e00 > dcm->e22)
    {
        precision_type_t s = 2.0 * sqrt(1.0 + dcm->e00 - dcm->e11 - dcm->e22);
        q->a               = (dcm->e12 - dcm->e21) / s;
        q->b               = 0.25 * s;
        q->c               = (dcm->e01 + dcm->e10) / s;
        q->d               = (dcm->e02 + dcm->e20) / s;
    }
    else if (dcm->e11 > dcm->e22)
    {
        precision_type_t s = 2.0 * sqrt(1.0 + dcm->e11 - dcm->e00 - dcm->e22);
        q->a               = (dcm->e20 - dcm->e02) / s;
        q->b               = (dcm->e01 + dcm->e10) / s;
        q->c               = 0.25 * s;
        q->d               = (dcm->e12 + dcm->e21) / s;
    }
    else
    {
        precision_type_t s = 2.0 * sqrt(1.0 + dcm->e22 - dcm->e00 - dcm->e11);
        q->a               = (dcm->e01 - dcm->e10) / s;
        q->b               = (dcm->e02 + dcm->e20) / s;
        q->c               = (dcm->e12 + dcm->e21) / s;
        q->d               = 0.25 * s;
    }

    return KL_NO_ERROR;
}

int quaternion_init_identity(quaternion_t *q)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    q->a = 1.0;
    q->b = 0.0;
    q->c = 0.0;
    q->d = 0.0;

    return KL_NO_ERROR;
}

int quaternion_inverse(const quaternion_t *q, quaternion_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    int error;

    error = quaternion_conjugate(q, result);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    precision_type_t square;
    error = quaternion_square(result, &square);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    result->a /= square;
    result->b /= square;
    result->c /= square;
    result->d /= square;

    return KL_NO_ERROR;
}

int quaternion_mul(const quaternion_t *q_a, const quaternion_t *q_b, quaternion_t *result)
{
    if (q_a == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (q_b == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    result->a = q_a->a * q_b->a - q_a->b * q_b->b - q_a->c * q_b->c - q_a->d * q_b->d;
    result->b = q_a->a * q_b->b + q_a->b * q_b->a + q_a->c * q_b->d - q_a->d * q_b->c;
    result->c = q_a->a * q_b->c + q_a->c * q_b->a + q_a->d * q_b->b - q_a->b * q_b->d;
    result->d = q_a->a * q_b->d + q_a->d * q_b->a + q_a->b * q_b->c - q_a->c * q_b->b;

    return KL_NO_ERROR;
}

int quaternion_norm(const quaternion_t *q, precision_type_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    int              error;
    precision_type_t square;
    error = quaternion_square(q, &square);
    if (error != KL_NO_ERROR)
    {
        return error;
    }

    (*result) = sqrt(square);

    return KL_NO_ERROR;
}

int quaternion_rotate(const quaternion_t *q, const kvector_t *kv, kvector_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    // This function performs an efficient coordinate transform using an expanded form of the traditional equation used
    // to perform coordinate transforms with quaternions (q_kv' = q * q_kv * q_conj). See the following references:
    // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    // https://www.mathworks.com/help/nav/ref/quaternion.rotatepoint.html

    const precision_type_t x     = kv->x;
    const precision_type_t y     = kv->y;
    const precision_type_t z     = kv->z;
    const precision_type_t alpha = 2.0 * q->b * x + 2.0 * q->c * y + 2.0 * q->d * z;
    const precision_type_t beta  = q->a * q->a - q->b * q->b - q->c * q->c - q->d * q->d;
    const precision_type_t a_x_2 = 2.0 * q->a;

    result->x = alpha * q->b + beta * x + a_x_2 * (q->c * z - q->d * y);
    result->y = alpha * q->c + beta * y + a_x_2 * (q->d * x - q->b * z);
    result->z = alpha * q->d + beta * z + a_x_2 * (q->b * y - q->c * x);

    return KL_NO_ERROR;
}

int quaternion_square(const quaternion_t *q, precision_type_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    (*result) = q->a * q->a + q->b * q->b + q->c * q->c + q->d * q->d;

    return KL_NO_ERROR;
}

int quaternion_transform(const quaternion_t *q, const kvector_t *kv, kvector_t *result)
{
    if (q == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (kv == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    if (result == NULL)
    {
        return KL_ERROR_NULL_ARGUMENT;
    }

    // This function performs an efficient coordinate transform using an expanded form of the traditional equation used
    // to perform coordinate transforms with quaternions (q_kv' = q_conj * q_kv * q). See the following references:
    // https://www.mathworks.com/help/nav/ref/quaternion.rotateframe.html

    const precision_type_t x     = kv->x;
    const precision_type_t y     = kv->y;
    const precision_type_t z     = kv->z;
    const precision_type_t alpha = 2.0 * q->b * x + 2.0 * q->c * y + 2.0 * q->d * z;
    const precision_type_t beta  = q->a * q->a - q->b * q->b - q->c * q->c - q->d * q->d;
    const precision_type_t a_x_2 = 2.0 * q->a;

    result->x = alpha * q->b + beta * x - a_x_2 * (q->c * z - q->d * y);
    result->y = alpha * q->c + beta * y - a_x_2 * (q->d * x - q->b * z);
    result->z = alpha * q->d + beta * z - a_x_2 * (q->b * y - q->c * x);

    return KL_NO_ERROR;
}
