/***********************************************************************************************************************
 * Kinematic rotations library.
 **********************************************************************************************************************/

#include <cmath>
#include <kinematics-library/rotations.hpp>

DCM::DCM() : e00(1.0), e01(0.0), e02(0.0), e10(0.0), e11(1.0), e12(0.0), e20(0.0), e21(0.0), e22(1.0) {}

DCM::DCM(
    const precision_type_t &a1_rad,
    const precision_type_t &a2_rad,
    const precision_type_t &a3_rad,
    const RotationSequence &seq)
{
    this->set(a1_rad, a2_rad, a3_rad, seq);
}

DCM::DCM(const Quaternion &q)
{
    this->set(q);
}

precision_type_t DCM::get_e00() const
{
    return e00;
}

precision_type_t DCM::get_e01() const
{
    return e01;
}

precision_type_t DCM::get_e02() const
{
    return e02;
}

precision_type_t DCM::get_e10() const
{
    return e10;
}

precision_type_t DCM::get_e11() const
{
    return e11;
}

precision_type_t DCM::get_e12() const
{
    return e12;
}

precision_type_t DCM::get_e20() const
{
    return e20;
}

precision_type_t DCM::get_e21() const
{
    return e21;
}

precision_type_t DCM::get_e22() const
{
    return e22;
}

DCM DCM::operator*(const DCM &rhs) const
{
    DCM result;

    result.e00 = e00 * rhs.e00 + e01 * rhs.e10 + e02 * rhs.e20;
    result.e01 = e00 * rhs.e01 + e01 * rhs.e11 + e02 * rhs.e21;
    result.e02 = e00 * rhs.e02 + e01 * rhs.e12 + e02 * rhs.e22;
    result.e10 = e10 * rhs.e00 + e11 * rhs.e10 + e12 * rhs.e20;
    result.e11 = e10 * rhs.e01 + e11 * rhs.e11 + e12 * rhs.e21;
    result.e12 = e10 * rhs.e02 + e11 * rhs.e12 + e12 * rhs.e22;
    result.e20 = e20 * rhs.e00 + e21 * rhs.e10 + e22 * rhs.e20;
    result.e21 = e20 * rhs.e01 + e21 * rhs.e11 + e22 * rhs.e21;
    result.e22 = e20 * rhs.e02 + e21 * rhs.e12 + e22 * rhs.e22;

    return result;
}

KinematicVector DCM::operator*(const KinematicVector &rhs) const
{
    KinematicVector result;
    this->transform(rhs, result);
    return result;
}

bool DCM::operator==(const DCM &rhs) const
{
    return (
        e00 == rhs.e00 && e01 == rhs.e01 && e02 == rhs.e02 && e10 == rhs.e10 && e11 == rhs.e11 && e12 == rhs.e12 &&
        e20 == rhs.e20 && e21 == rhs.e21 && e22 == rhs.e22);
}

bool DCM::operator!=(const DCM &rhs) const
{
    return !(this->operator==(rhs));
}

void DCM::rotate(const KinematicVector &kv, KinematicVector &result) const
{
    const precision_type_t x(kv.get_x()), y(kv.get_y()), z(kv.get_z());

    result.set_x(x * e00 + y * e10 + z * e20);
    result.set_y(x * e01 + y * e11 + z * e21);
    result.set_z(x * e02 + y * e12 + z * e22);
}

void DCM::rotate(KinematicVector &kv) const
{
    this->rotate(kv, kv);
}

void DCM::set(
    const precision_type_t &a1_rad,
    const precision_type_t &a2_rad,
    const precision_type_t &a3_rad,
    const RotationSequence &seq)
{
    // The choice to explicitly define functions for every rotation sequence is deliberate. It is computationally more
    // efficient to to only do the exact calculations necessary to calculate the direction cosine matrices. The obvious
    // downside to this approach is that there are a lot of rotation sequence combinations.

    switch (seq)
    {
        case RotationSequence::BODY_XYX:
            e00 = cos(a2_rad);
            e01 = sin(a1_rad) * sin(a2_rad);
            e02 = -cos(a1_rad) * sin(a2_rad);
            e10 = sin(a3_rad) * sin(a2_rad);
            e11 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            e12 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * cos(a2_rad) * sin(a3_rad);
            e20 = cos(a3_rad) * sin(a2_rad);
            e21 = -cos(a1_rad) * sin(a3_rad) - cos(a3_rad) * cos(a2_rad) * sin(a1_rad);
            e22 = cos(a1_rad) * cos(a3_rad) * cos(a2_rad) - sin(a1_rad) * sin(a3_rad);
            break;

        case RotationSequence::BODY_XYZ:
            e00 = cos(a3_rad) * cos(a2_rad);
            e01 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a1_rad) * sin(a2_rad);
            e02 = sin(a1_rad) * sin(a3_rad) - cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            e10 = -cos(a2_rad) * sin(a3_rad);
            e11 = cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad) * sin(a2_rad);
            e12 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a3_rad) * sin(a2_rad);
            e20 = sin(a2_rad);
            e21 = -cos(a2_rad) * sin(a1_rad);
            e22 = cos(a1_rad) * cos(a2_rad);
            break;

        case RotationSequence::BODY_XZX:
            e00 = cos(a2_rad);
            e01 = cos(a1_rad) * sin(a2_rad);
            e02 = sin(a2_rad) * sin(a1_rad);
            e10 = -cos(a3_rad) * sin(a2_rad);
            e11 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            e12 = cos(a1_rad) * sin(a3_rad) + cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            e20 = sin(a2_rad) * sin(a3_rad);
            e21 = -cos(a3_rad) * sin(a1_rad) - cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            e22 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            break;

        case RotationSequence::BODY_XZY:
            e00 = cos(a2_rad) * cos(a3_rad);
            e01 = sin(a1_rad) * sin(a3_rad) + cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            e02 = cos(a3_rad) * sin(a1_rad) * sin(a2_rad) - cos(a1_rad) * sin(a3_rad);
            e10 = -sin(a2_rad);
            e11 = cos(a1_rad) * cos(a2_rad);
            e12 = cos(a2_rad) * sin(a1_rad);
            e20 = cos(a2_rad) * sin(a3_rad);
            e21 = cos(a1_rad) * sin(a2_rad) * sin(a3_rad) - cos(a3_rad) * sin(a1_rad);
            e22 = cos(a1_rad) * cos(a3_rad) + sin(a1_rad) * sin(a2_rad) * sin(a3_rad);
            break;

        case RotationSequence::BODY_YXY:
            e00 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            e01 = sin(a2_rad) * sin(a3_rad);
            e02 = -cos(a3_rad) * sin(a1_rad) - cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            e10 = sin(a2_rad) * sin(a1_rad);
            e11 = cos(a2_rad);
            e12 = cos(a1_rad) * sin(a2_rad);
            e20 = cos(a1_rad) * sin(a3_rad) + cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            e21 = -cos(a3_rad) * sin(a2_rad);
            e22 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            break;

        case RotationSequence::BODY_YXZ:
            e00 = cos(a3_rad) * cos(a1_rad) + sin(a2_rad) * sin(a3_rad) * sin(a1_rad);
            e01 = cos(a2_rad) * sin(a3_rad);
            e02 = cos(a1_rad) * sin(a2_rad) * sin(a3_rad) - cos(a3_rad) * sin(a1_rad);
            e10 = cos(a3_rad) * sin(a2_rad) * sin(a1_rad) - cos(a1_rad) * sin(a3_rad);
            e11 = cos(a2_rad) * cos(a3_rad);
            e12 = sin(a3_rad) * sin(a1_rad) + cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            e20 = cos(a2_rad) * sin(a1_rad);
            e21 = -sin(a2_rad);
            e22 = cos(a2_rad) * cos(a1_rad);
            break;

        case RotationSequence::BODY_YZX:
            e00 = cos(a2_rad) * cos(a1_rad);
            e01 = sin(a2_rad);
            e02 = -cos(a2_rad) * sin(a1_rad);
            e10 = sin(a3_rad) * sin(a1_rad) - cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            e11 = cos(a3_rad) * cos(a2_rad);
            e12 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a2_rad) * sin(a1_rad);
            e20 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a3_rad) * sin(a2_rad);
            e21 = -cos(a2_rad) * sin(a3_rad);
            e22 = cos(a3_rad) * cos(a1_rad) - sin(a3_rad) * sin(a2_rad) * sin(a1_rad);
            break;

        case RotationSequence::BODY_YZY:
            e00 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            e01 = cos(a3_rad) * sin(a2_rad);
            e02 = -cos(a1_rad) * sin(a3_rad) - cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            e10 = -cos(a1_rad) * sin(a2_rad);
            e11 = cos(a2_rad);
            e12 = sin(a2_rad) * sin(a1_rad);
            e20 = cos(a3_rad) * sin(a1_rad) + cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            e21 = sin(a2_rad) * sin(a3_rad);
            e22 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            break;

        case RotationSequence::BODY_ZXY:
            e00 = cos(a1_rad) * cos(a3_rad) - sin(a2_rad) * sin(a1_rad) * sin(a3_rad);
            e01 = cos(a3_rad) * sin(a1_rad) + cos(a1_rad) * sin(a2_rad) * sin(a3_rad);
            e02 = -cos(a2_rad) * sin(a3_rad);
            e10 = -cos(a2_rad) * sin(a1_rad);
            e11 = cos(a2_rad) * cos(a1_rad);
            e12 = sin(a2_rad);
            e20 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * sin(a2_rad) * sin(a1_rad);
            e21 = sin(a1_rad) * sin(a3_rad) - cos(a1_rad) * cos(a3_rad) * sin(a2_rad);
            e22 = cos(a2_rad) * cos(a3_rad);
            break;

        case RotationSequence::BODY_ZXZ:
            e00 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            e01 = cos(a3_rad) * sin(a1_rad) + cos(a2_rad) * cos(a1_rad) * sin(a3_rad);
            e02 = sin(a2_rad) * sin(a3_rad);
            e10 = -cos(a1_rad) * sin(a3_rad) - cos(a2_rad) * cos(a3_rad) * sin(a1_rad);
            e11 = cos(a2_rad) * cos(a1_rad) * cos(a3_rad) - sin(a1_rad) * sin(a3_rad);
            e12 = cos(a3_rad) * sin(a2_rad);
            e20 = sin(a2_rad) * sin(a1_rad);
            e21 = -cos(a1_rad) * sin(a2_rad);
            e22 = cos(a2_rad);
            break;

        case RotationSequence::BODY_ZYX:
            e00 = cos(a1_rad) * cos(a2_rad);
            e01 = cos(a2_rad) * sin(a1_rad);
            e02 = -sin(a2_rad);
            e10 = cos(a1_rad) * sin(a3_rad) * sin(a2_rad) - cos(a3_rad) * sin(a1_rad);
            e11 = cos(a3_rad) * cos(a1_rad) + sin(a3_rad) * sin(a1_rad) * sin(a2_rad);
            e12 = cos(a2_rad) * sin(a3_rad);
            e20 = sin(a3_rad) * sin(a1_rad) + cos(a3_rad) * cos(a1_rad) * sin(a2_rad);
            e21 = cos(a3_rad) * sin(a1_rad) * sin(a2_rad) - cos(a1_rad) * sin(a3_rad);
            e22 = cos(a3_rad) * cos(a2_rad);
            break;

        case RotationSequence::BODY_ZYZ:
            e00 = cos(a1_rad) * cos(a3_rad) * cos(a2_rad) - sin(a1_rad) * sin(a3_rad);
            e01 = cos(a1_rad) * sin(a3_rad) + cos(a3_rad) * cos(a2_rad) * sin(a1_rad);
            e02 = -cos(a3_rad) * sin(a2_rad);
            e10 = -cos(a3_rad) * sin(a1_rad) - cos(a1_rad) * cos(a2_rad) * sin(a3_rad);
            e11 = cos(a1_rad) * cos(a3_rad) - cos(a2_rad) * sin(a1_rad) * sin(a3_rad);
            e12 = sin(a3_rad) * sin(a2_rad);
            e20 = cos(a1_rad) * sin(a2_rad);
            e21 = sin(a1_rad) * sin(a2_rad);
            e22 = cos(a2_rad);
            break;
    }
}

void DCM::set(const Quaternion &q)
{
    const precision_type_t a(q.get_a()), b(q.get_b()), c(q.get_c()), d(q.get_d());
    const precision_type_t a_x_a(a * a), a_x_b(a * b), a_x_c(a * c), a_x_d(a * d);
    const precision_type_t b_x_c(b * c), b_x_d(b * d);
    const precision_type_t c_x_d(c * d);

    e00 = 2.0 * a_x_a - 1.0 + 2.0 * b * b;
    e01 = 2.0 * b_x_c + 2.0 * a_x_d;
    e02 = 2.0 * b_x_d - 2.0 * a_x_c;
    e10 = 2.0 * b_x_c - 2.0 * a_x_d;
    e11 = 2.0 * a_x_a - 1.0 + 2.0 * c * c;
    e12 = 2.0 * c_x_d + 2.0 * a_x_b;
    e20 = 2.0 * b_x_d + 2.0 * a_x_c;
    e21 = 2.0 * c_x_d - 2.0 * a_x_b;
    e22 = 2.0 * a_x_a - 1.0 + 2.0 * d * d;
}

void DCM::transform(const KinematicVector &kv, KinematicVector &result) const
{
    const precision_type_t x(kv.get_x()), y(kv.get_y()), z(kv.get_z());

    result.set_x(x * e00 + y * e01 + z * e02);
    result.set_y(x * e10 + y * e11 + z * e12);
    result.set_z(x * e20 + y * e21 + z * e22);
}

void DCM::transform(KinematicVector &kv) const
{
    this->transform(kv, kv);
}

void DCM::transpose(DCM &dcm) const
{
    dcm.e00 = e00;
    dcm.e01 = e10;
    dcm.e02 = e20;
    dcm.e10 = e01;
    dcm.e11 = e11;
    dcm.e12 = e21;
    dcm.e20 = e02;
    dcm.e21 = e12;
    dcm.e22 = e22;
}

DCM DCM::transpose() const
{
    DCM dcm;
    this->transpose(dcm);
    return dcm;
}

KinematicVector operator*(const KinematicVector &lhs, const DCM &rhs)
{
    KinematicVector result;
    rhs.rotate(lhs, result);
    return result;
}

Quaternion::Quaternion() : a(1.0), b(0.0), c(0.0), d(0.0) {}

Quaternion::Quaternion(
    const precision_type_t &a1_rad,
    const precision_type_t &a2_rad,
    const precision_type_t &a3_rad,
    const RotationSequence &seq)
{
    this->set(a1_rad, a2_rad, a3_rad, seq);
}

Quaternion::Quaternion(const DCM &dcm)
{
    this->set(dcm);
}

precision_type_t Quaternion::angle() const
{
    return 2.0 * acos(a);
}

void Quaternion::axis(KinematicVector &kv) const
{
    kv.set(b, c, d);
    kv.to_unit();
}

void Quaternion::conjugate(Quaternion &q) const
{
    q.a = a;
    q.b = -b;
    q.c = -c;
    q.d = -d;
}

Quaternion Quaternion::conjugate() const
{
    Quaternion q;
    this->conjugate(q);
    return q;
}

precision_type_t Quaternion::get_a() const
{
    return a;
}

precision_type_t Quaternion::get_b() const
{
    return b;
}

precision_type_t Quaternion::get_c() const
{
    return c;
}

precision_type_t Quaternion::get_d() const
{
    return d;
}

void Quaternion::inverse(Quaternion &q) const
{
    this->conjugate(q);
    precision_type_t q_square(q.square());

    q.a /= q_square;
    q.b /= q_square;
    q.c /= q_square;
    q.d /= q_square;
}

Quaternion Quaternion::inverse() const
{
    Quaternion q;
    this->inverse(q);
    return q;
}

precision_type_t Quaternion::norm() const
{
    return sqrt(this->square());
}

Quaternion Quaternion::operator*(const Quaternion &rhs) const
{
    Quaternion q;

    q.a = a * rhs.a - b * rhs.b - c * rhs.c - d * rhs.d;
    q.b = a * rhs.b + b * rhs.a + c * rhs.d - d * rhs.c;
    q.c = a * rhs.c + c * rhs.a + d * rhs.b - b * rhs.d;
    q.d = a * rhs.d + d * rhs.a + b * rhs.c - c * rhs.b;

    return q;
}

bool Quaternion::operator==(const Quaternion &rhs) const
{
    return (a == rhs.a && b == rhs.b && c == rhs.c && d == rhs.d);
}

bool Quaternion::operator!=(const Quaternion &rhs) const
{
    return !(this->operator==(rhs));
}

void Quaternion::rotate(const KinematicVector &kv, KinematicVector &result) const
{
    // This function performs an efficient coordinate transform using an expanded form of the traditional equation used
    // to perform coordinate transforms with quaternions (q_kv' = q * q_kv * q_conj). See the following references:
    // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    // https://www.mathworks.com/help/nav/ref/quaternion.rotatepoint.html

    const precision_type_t x(kv.get_x()), y(kv.get_y()), z(kv.get_z());
    const precision_type_t alpha(2.0 * b * x + 2.0 * c * y + 2.0 * d * z);
    const precision_type_t beta(a * a - b * b - c * c - d * d);
    const precision_type_t a_x_2(2.0 * a);

    result.set_x(alpha * b + beta * x + a_x_2 * (c * z - d * y));
    result.set_y(alpha * c + beta * y + a_x_2 * (d * x - b * z));
    result.set_z(alpha * d + beta * z + a_x_2 * (b * y - c * x));
}

void Quaternion::rotate(KinematicVector &kv) const
{
    this->rotate(kv, kv);
}

void Quaternion::set(
    const precision_type_t &a1_rad,
    const precision_type_t &a2_rad,
    const precision_type_t &a3_rad,
    const RotationSequence &seq)
{
    // It's significantly easier to use a DCM to set the quaternion behind the scenes than re-write equations for all
    // rotation sequences. This of course comes with increased computational cost but is deemed "acceptable".
    static thread_local DCM dcm;
    dcm.set(a1_rad, a2_rad, a3_rad, seq);
    this->set(dcm);
}

void Quaternion::set(const DCM &dcm)
{
    const precision_type_t c00(dcm.get_e00()), c11(dcm.get_e11()), c22(dcm.get_e22());
    const precision_type_t trace(c00 + c11 + c22);

    // This logic is necessary to ensure that the conversion will never encounter a divide by zero error. Essentially,
    // the conversion can be done using any of the DCM diagonal components. However, nothing stops one or two of the
    // diagonal values from being zero. But, at least one component must be non-zero (property of DCM). Therefore, the
    // max diagonal diagonal is used for direction cosine matrices with negative traces.
    if (trace > 0.0)
    {
        precision_type_t s = 2.0 * std::sqrt(1.0 + trace);
        a                  = 0.25 * s;
        b                  = (dcm.get_e12() - dcm.get_e21()) / s;
        c                  = (dcm.get_e20() - dcm.get_e02()) / s;
        d                  = (dcm.get_e01() - dcm.get_e10()) / s;
    }
    else if (c00 > c11 && c00 > c22)
    {
        precision_type_t s = 2.0 * std::sqrt(1.0 + c00 - c11 - c22);
        a                  = (dcm.get_e12() - dcm.get_e21()) / s;
        b                  = 0.25 * s;
        c                  = (dcm.get_e01() + dcm.get_e10()) / s;
        d                  = (dcm.get_e02() + dcm.get_e20()) / s;
    }
    else if (c11 > c22)
    {
        precision_type_t s = 2.0 * std::sqrt(1.0 + c11 - c00 - c22);
        a                  = (dcm.get_e20() - dcm.get_e02()) / s;
        b                  = (dcm.get_e01() + dcm.get_e10()) / s;
        c                  = 0.25 * s;
        d                  = (dcm.get_e12() + dcm.get_e21()) / s;
    }
    else
    {
        precision_type_t s = 2.0 * std::sqrt(1.0 + c22 - c00 - c11);
        a                  = (dcm.get_e01() - dcm.get_e10()) / s;
        b                  = (dcm.get_e02() + dcm.get_e20()) / s;
        c                  = (dcm.get_e12() + dcm.get_e21()) / s;
        d                  = 0.25 * s;
    }
}

precision_type_t Quaternion::square() const
{
    return a * a + b * b + c * c + d * d;
}

void Quaternion::transform(const KinematicVector &kv, KinematicVector &result) const
{
    // This function performs an efficient coordinate transform using an expanded form of the traditional equation used
    // to perform coordinate transforms with quaternions (q_kv' = q_conj * q_kv * q). See the following references:
    // https://www.mathworks.com/help/nav/ref/quaternion.rotateframe.html

    const precision_type_t x(kv.get_x()), y(kv.get_y()), z(kv.get_z());
    const precision_type_t alpha(2.0 * b * x + 2.0 * c * y + 2.0 * d * z);
    const precision_type_t beta(a * a - b * b - c * c - d * d);
    const precision_type_t a_x_2(2.0 * a);

    result.set_x(alpha * b + beta * x - a_x_2 * (c * z - d * y));
    result.set_y(alpha * c + beta * y - a_x_2 * (d * x - b * z));
    result.set_z(alpha * d + beta * z - a_x_2 * (b * y - c * x));
}

void Quaternion::transform(KinematicVector &kv) const
{
    this->transform(kv, kv);
}
