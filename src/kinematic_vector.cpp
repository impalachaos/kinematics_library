/***********************************************************************************************************************
 * Kinematic vector library.
 **********************************************************************************************************************/

#define DBL_TINY 1.0E-15

#include <cmath>
#include <kinematics-library/kinematic_vector.hpp>

KinematicVector::KinematicVector() : x(0.0), y(0.0), z(0.0) {}

KinematicVector::KinematicVector(const precision_type_t &x_, const precision_type_t &y_, const precision_type_t &z_)
    : x(x_),
      y(y_),
      z(z_)
{}

KinematicVector::KinematicVector(const KinematicVector &kv) : x(kv.x), y(kv.y), z(kv.z) {}

precision_type_t KinematicVector::angle_between(const KinematicVector &kv) const
{
    return acos(this->dot(kv) / ((this->magnitude() + DBL_TINY) * (kv.magnitude() + DBL_TINY)));
}

precision_type_t KinematicVector::azimuth_angle() const
{
    return atan2(y, x);
}

KinematicVector KinematicVector::cross(const KinematicVector &kv) const
{
    return KinematicVector((y * kv.z) - (z * kv.y), (z * kv.x) - (x * kv.z), (x * kv.y) - (y * kv.x));
}

precision_type_t KinematicVector::dot(const KinematicVector &kv) const
{
    return (x * kv.x) + (y * kv.y) + (z * kv.z);
}

precision_type_t KinematicVector::elevation_angle() const
{
    return asin(-z / (this->magnitude() + DBL_TINY));
}

precision_type_t KinematicVector::get_x() const
{
    return x;
}

precision_type_t KinematicVector::get_y() const
{
    return y;
}

precision_type_t KinematicVector::get_z() const
{
    return z;
}

precision_type_t KinematicVector::magnitude() const
{
    return sqrt((x * x) + (y * y) + (z * z));
}

KinematicVector KinematicVector::operator+(const precision_type_t &scalar) const
{
    return KinematicVector(x + scalar, y + scalar, z + scalar);
}

KinematicVector KinematicVector::operator+=(const KinematicVector &kv)
{
    x += kv.x;
    y += kv.y;
    z += kv.z;

    return *this;
}

KinematicVector KinematicVector::operator+(const KinematicVector &kv) const
{
    return KinematicVector(x + kv.x, y + kv.y, z + kv.z);
}

KinematicVector KinematicVector::operator-(const precision_type_t &scalar) const
{
    return KinematicVector(x - scalar, y - scalar, z - scalar);
}

KinematicVector KinematicVector::operator-(const KinematicVector &kv) const
{
    return KinematicVector(x - kv.x, y - kv.y, z - kv.z);
}

KinematicVector KinematicVector::operator-=(const KinematicVector &kv)
{
    x -= kv.x;
    y -= kv.y;
    z -= kv.z;

    return *this;
}

KinematicVector KinematicVector::operator*(const precision_type_t &scalar) const
{
    return KinematicVector(x * scalar, y * scalar, z * scalar);
}

KinematicVector KinematicVector::operator/(const precision_type_t &scalar) const
{
    return KinematicVector(x / scalar, y / scalar, z / scalar);
}

bool KinematicVector::operator==(const KinematicVector &kv) const
{
    return (x == kv.x) && (y == kv.y) && (z == kv.z);
}

bool KinematicVector::operator!=(const KinematicVector &kv) const
{
    return !(operator==(kv));
}

precision_type_t KinematicVector::polar_angle() const
{
    return acos(z / (this->magnitude() + DBL_TINY));
}

void KinematicVector::set(const KinematicVector &kv)
{
    x = kv.x;
    y = kv.y;
    z = kv.z;
}

void KinematicVector::set(const precision_type_t &x_, const precision_type_t &y_, const precision_type_t &z_)
{
    x = x_;
    y = y_;
    z = z_;
}

void KinematicVector::set_x(const precision_type_t &x_)
{
    x = x_;
}

void KinematicVector::set_y(const precision_type_t &y_)
{
    y = y_;
}

void KinematicVector::set_z(const precision_type_t &z_)
{
    z = z_;
}

void KinematicVector::to_unit()
{
    precision_type_t mag = this->magnitude();

    if (mag > 0.0)
    {
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

KinematicVector KinematicVector::unit() const
{
    precision_type_t mag = this->magnitude();
    KinematicVector  uv;

    if (mag > 0.0)
    {
        uv.set(x / mag, y / mag, z / mag);
    }

    return uv;
}

void KinematicVector::zero()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

KinematicVector operator+(const precision_type_t &scalar, const KinematicVector &kv)
{
    return kv + scalar;
}

KinematicVector operator-(const precision_type_t &scalar, const KinematicVector &kv)
{
    return KinematicVector(scalar - kv.get_x(), scalar - kv.get_y(), scalar - kv.get_z());
}

KinematicVector operator*(const precision_type_t &scalar, const KinematicVector &kv)
{
    return kv * scalar;
}

KinematicVector operator/(const precision_type_t &scalar, const KinematicVector &kv)
{
    return KinematicVector(scalar / kv.get_x(), scalar / kv.get_y(), scalar / kv.get_z());
}
