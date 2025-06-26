/***********************************************************************************************************************
 * Kinematic vector library.
 **********************************************************************************************************************/

#pragma once

#include <kinematics-library/kinematic_vector.h>

/***********************************************************************************************************************
 * @brief A vector designed for kinematic applications. At the core, a kinematic vector is a vector of length 3 which
 * has arbitrary x, y, and z components.
 **********************************************************************************************************************/
class KinematicVector
{
public:
    /*******************************************************************************************************************
     * @brief Creates a kinematic vector of all zeros.
     ******************************************************************************************************************/
    KinematicVector();

    /*******************************************************************************************************************
     * @brief Creates a kinematic vector with specified components.
     *
     * @param x Vector x-component (i.e., element 0)
     * @param y Vector y-component (i.e., element 1)
     * @param z Vector z-component (i.e., element 2)
     ******************************************************************************************************************/
    KinematicVector(const precision_type_t &x, const precision_type_t &y, const precision_type_t &z);

    /*******************************************************************************************************************
     * @brief Creates a kinematic vector from another kinematic vector.
     *
     * @param kv Kinematic vector to copy
     ******************************************************************************************************************/
    KinematicVector(const KinematicVector &kv);

    /*******************************************************************************************************************
     * @brief Calculates the angle between two kinematic vectors. For vectors with 0 magnitude, small magnitude is
     * assigned instead. No protections for arccos ![-1, 1]
     *
     * @param kv Kinematic vector
     * @return Angle between vectors [0, +pi] [radians]
     ******************************************************************************************************************/
    precision_type_t angle_between(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the azimuth angle (i.e., the angle between the x-axis and the xy projection of the kinematic
     * vector). Note that convention implies a positive rotation about the z-axis, such that vectors with xy-projections
     * in quadrant 1 and 2 (positive y-values) have positive azimuth angles, while vectors with xy-projections in
     * quadrant 3 and 4  (negative y-values) have negative azimuth angles.
     *
     * @return Azimuth angle in the interval [-pi, +pi] [radians]
     ******************************************************************************************************************/
    precision_type_t azimuth_angle() const;

    /*******************************************************************************************************************
     * @brief Calculates the cross product of two kinematic vectors. This method calculates the cross product such that
     * `kv_a.cross(kv_b)` is equivalent to `kv_a` cross `kv_b`.
     *
     * @param kv Kinematic vector
     * @return Result of cross product
     ******************************************************************************************************************/
    KinematicVector cross(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the dot product of two kinematic vectors.
     *
     * @param kv Kinematic vector
     * @return Result of dot product
     ******************************************************************************************************************/
    precision_type_t dot(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the elevation angle (i.e., the angle between the xy projection of the kinematic vector and the
     * kinematic vector). Note that convention implies that vectors above the xy-plane (i.e., positive z-component) have
     * negative elevation angles and vectors below the xy-plane (i.e., negative z-component) have positive elevation
     * angles.
     *
     * @return Elevation angle in the interval [-pi/2, +pi/2] [radians]
     ******************************************************************************************************************/
    precision_type_t elevation_angle() const;

    /*******************************************************************************************************************
     * @brief Gets the x-component of the kinematic vector.
     *
     * @return Vector x-component (i.e., element 0)
     ******************************************************************************************************************/
    precision_type_t get_x() const;

    /*******************************************************************************************************************
     * @brief Gets the y-component of the kinematic vector.
     *
     * @return Vector y-component (i.e., element 1)
     ******************************************************************************************************************/
    precision_type_t get_y() const;

    /*******************************************************************************************************************
     * @brief Gets the z-component of the kinematic vector.
     *
     * @return Vector z-component (i.e., element 2)
     ******************************************************************************************************************/
    precision_type_t get_z() const;

    /*******************************************************************************************************************
     * @brief Calculates the magnitude of the kinematic vector.
     *
     * @return Magnitude of the vector
     ******************************************************************************************************************/
    precision_type_t magnitude() const;

    /*******************************************************************************************************************
     * @brief Vector-scalar addition.
     *
     * @param scalar Scalar addend
     * @return Result of the vector-scalar addition
     ******************************************************************************************************************/
    KinematicVector operator+(const precision_type_t &scalar) const;

    /*******************************************************************************************************************
     * @brief Vector-vector addition.
     *
     * @param kv Vector addend
     * @return Result of the vector-vector addition
     ******************************************************************************************************************/
    KinematicVector operator+(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Vector-vector in-place addition.
     *
     * @param kv Vector addend
     * @return Result of the vector-vector addition
     ******************************************************************************************************************/
    KinematicVector operator+=(const KinematicVector &kv);

    /*******************************************************************************************************************
     * @brief Vector-scalar subtraction.
     *
     * @param scalar Scalar subtrahend
     * @return Result of the vector-scalar subraction
     ******************************************************************************************************************/
    KinematicVector operator-(const precision_type_t &scalar) const;

    /*******************************************************************************************************************
     * @brief Vector-vector subtraction.
     *
     * @param kv Vector subtrahend
     * @return Result of the vector-vector subtraction
     ******************************************************************************************************************/
    KinematicVector operator-(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Vector-vector in-place subtraction.
     *
     * @param kv Vector subtrahend
     * @return Result of the vector-vector subtraction
     ******************************************************************************************************************/
    KinematicVector operator-=(const KinematicVector &kv);

    /*******************************************************************************************************************
     * @brief Vector-scalar multiplication.
     *
     * @param scalar Scalar multiplicand
     * @return Result of the vector-scalar multiplication
     ******************************************************************************************************************/
    KinematicVector operator*(const precision_type_t &scalar) const;

    /*******************************************************************************************************************
     * @brief Vector-scalar division.
     *
     * @param scalar Scalar divisor
     * @return Result of the vector-scalar division
     ******************************************************************************************************************/
    KinematicVector operator/(const precision_type_t &scalar) const;

    /*******************************************************************************************************************
     * @brief Vector equality operator.
     *
     * @param kv Kinematic vector to compare
     * @return Indicates if vectors are equal
     ******************************************************************************************************************/
    bool operator==(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Vector inequality operator.
     *
     * @param kv Kinematic vector to compare
     * @return Indicates if vectors are not equal
     ******************************************************************************************************************/
    bool operator!=(const KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the polar angle (i.e., angle between the z-axis and the kinematic vector). Note that convention
     * implies that a vector aligned with the z-axis has a polar angle of 0.0 degrees, a vector in the xy-plane has a
     * polar angle of 90.0 degrees, and a vector aligned with the negative z-axis has a polar angle of 180.0 degrees.
     *
     * @return Polar angle in the interval [0, pi] [radians]
     ******************************************************************************************************************/
    precision_type_t polar_angle() const;

    /*******************************************************************************************************************
     * @brief Sets the kinematic vector to the specified kinematic vector.
     *
     * @param kv Kinematic vector
     ******************************************************************************************************************/
    void set(const KinematicVector &kv);

    /*******************************************************************************************************************
     * @brief Sets the kinematic vector to the specified x, y, and z values.
     *
     * @param x Vector x-component (i.e., element 0)
     * @param y Vector y-component (i.e., element 1)
     * @param z Vector z-component (i.e., element 2)
     ******************************************************************************************************************/
    void set(const precision_type_t &x, const precision_type_t &y, const precision_type_t &z);

    /*******************************************************************************************************************
     * @brief Sets the x-component of the kinematic vector.
     *
     * @param x Vector x-component (i.e., element 0)
     ******************************************************************************************************************/
    void set_x(const precision_type_t &x);

    /*******************************************************************************************************************
     * @brief Sets the y-component of the kinematic vector.
     *
     * @param y Vector y-component (i.e., element 1)
     ******************************************************************************************************************/
    void set_y(const precision_type_t &y);

    /*******************************************************************************************************************
     * @brief Sets the z-component of the kinematic vector.
     *
     * @param z Vector z-component (i.e., element 2)
     ******************************************************************************************************************/
    void set_z(const precision_type_t &z);

    /*******************************************************************************************************************
     * @brief Converts the kinematic vector to a unit vector.
     ******************************************************************************************************************/
    void to_unit();

    /*******************************************************************************************************************
     * @brief Calculates the unit vector pointing in the direction of the kinematic vector.
     *
     * @return Unit vector corresponding to the kinematic vector
     ******************************************************************************************************************/
    KinematicVector unit() const;

    /*******************************************************************************************************************
     * @brief Sets all vector components to zero.
     ******************************************************************************************************************/
    void zero();

private:
    precision_type_t x; // Vector x-component
    precision_type_t y; // Vector y-component
    precision_type_t z; // Vector z-component
};

/***********************************************************************************************************************
 * @brief Scalar-vector addition.
 *
 * @param scalar Scalar addend
 * @param kv Kinematic vector addend
 * @return Result of the scalar-vector addition
 **********************************************************************************************************************/
KinematicVector operator+(const precision_type_t &scalar, const KinematicVector &kv);

/***********************************************************************************************************************
 * @brief Scalar-vector subtraction.
 *
 * @param scalar Scalar minuend
 * @param kv Kinematic vector subtrahend
 * @return Result of the scalar-vector subraction
 **********************************************************************************************************************/
KinematicVector operator-(const precision_type_t &scalar, const KinematicVector &kv);

/***********************************************************************************************************************
 * @brief Scalar-vector multiplication.
 *
 * @param scalar Scalar multiplicand
 * @param kv Kinematic vector multiplicand
 * @return Result of the scalar-vector multiplication
 **********************************************************************************************************************/
KinematicVector operator*(const precision_type_t &scalar, const KinematicVector &kv);

/***********************************************************************************************************************
 * @brief Scalar-vector division.
 *
 * @param scalar Scalar dividend
 * @param kv Kinematic vector divisor
 * @return Result of the scalar-vector division
 **********************************************************************************************************************/
KinematicVector operator/(const precision_type_t &scalar, const KinematicVector &kv);
