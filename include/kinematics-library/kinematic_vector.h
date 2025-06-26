#ifndef KINEMATICS_LIBRARY_KINEMATIC_VECTOR_H_
#define KINEMATICS_LIBRARY_KINEMATIC_VECTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USE_PRECISION_TYPE_T
#define USE_PRECISION_TYPE_T
#ifdef MIN_MEMORY_FOOTPRINT
typedef float precision_type_t;
#else
typedef double precision_type_t;
#endif
#endif

/***********************************************************************************************************************
 * @brief A vector designed for kinematic applications. At the core, a kinematic vector is a vector of length 3 which
 * has arbitrary x, y, and z components.
 **********************************************************************************************************************/
typedef struct kvector_t
{
    precision_type_t x; ///< x component of a kinematic vector
    precision_type_t y; ///< y component of a kinematic vector
    precision_type_t z; ///< z component of a kinematic vector
} kvector_t;

/***********************************************************************************************************************
 * @brief Vector-vector addition.
 *
 * @param kv_a First kinematic vector addend
 * @param kv_b Second kinematic vector addend
 * @param result Result of the vector-vector addition
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_add(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result);

/***********************************************************************************************************************
 * @brief Vector-scalar addition.
 *
 * @param kv Kinematic vector addend
 * @param scalar Scalar addend
 * @param result Result of the vector-scalar addition
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_addf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the angle between two kinematic vectors. For vectors with 0 magnitude, small magnitude is
 * assigned instead. No protections for arccos ![-1, 1]
 *
 * @param kv_a Kinematic vector one
 * @param kv_b Kinematic vector two
 * @param result Angle between vectors [0, +pi] [radians]
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_angle_between(const kvector_t *kv_a, const kvector_t *kv_b, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Calculates the azimuth angle (i.e., the angle between the x-axis and the xy projection of the kinematic
 * vector). Note that convention implies a positive rotation about the z-axis, such that vectors with xy-projections
 * in quadrant 1 and 2 (positive y-values) have positive azimuth angles, while vectors with xy-projections in
 * quadrant 3 and 4  (negative y-values) have negative azimuth angles.
 *
 * @param kv Kinematic vector
 * @param result Azimuth angle in the interval [-pi, +pi] [radians]
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_azimuth_angle(const kvector_t *kv, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Calculates the cross product of two kinematic vectors.
 *
 * @param kv_a Kinematic vector one
 * @param kv_b Kinematic vector two
 * @param result Result of cross product
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_cross(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result);

/***********************************************************************************************************************
 * @brief Compares the two kinematic vectors and returns 0 if they are equal, 1 otherwise.
 *
 * @param kv_a Kinematic vector one
 * @param kv_b Kinematic vector two
 * @return 0 if the vectors are equal, 1 otherwise.
 **********************************************************************************************************************/
int kvector_compare(const kvector_t *kv_a, const kvector_t *kv_b);

/***********************************************************************************************************************
 * @brief Vector-scalar division.
 *
 * @param kv Kinematic vector
 * @param scalar Scalar divisor
 * @param result Vector to store the result of the vector-scalar division
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_divf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result);

/***********************************************************************************************************************
 * @brief Scalar-vector division.
 *
 * @param scalar Scalar dividend
 * @param kv Kinematic vector divisor
 * @param result Vector to store the result of the scalar-vector division
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_divf2(const precision_type_t scalar, const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the dot product of two kinematic vectors.
 *
 * @param kv_a Kinematic vector one
 * @param kv_b Kinematic vector two
 * @param result Result of dot product
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_dot(const kvector_t *kv_a, const kvector_t *kv_b, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Calculates the elevation angle (i.e., the angle between the xy projection of the kinematic vector and the
 * kinematic vector). Note that convention implies that vectors above the xy-plane (i.e., positive z-component) have
 * negative elevation angles and vectors below the xy-plane (i.e., negative z-component) have positive elevation
 * angles.
 *
 * @param kv Kinematic vector
 * @param result Elevation angle in the interval [-pi/2, +pi/2] [radians]
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_elevation_angle(const kvector_t *kv, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Initializes the kinematic vector to the specified x, y, and z values.
 *
 * @param x Vector x-component (i.e., element 0)
 * @param y Vector y-component (i.e., element 1)
 * @param z Vector z-component (i.e., element 2)
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_init(kvector_t *kv, const precision_type_t x, const precision_type_t y, const precision_type_t z);

/***********************************************************************************************************************
 * @brief Calculates the magnitude of the kinematic vector.
 *
 * @param kv Kinematic vector
 * @param result Magnitude of the vector
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_magnitude(const kvector_t *kv, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Vector-scalar multiplication.
 *
 * @param kv Kinematic vector multiplicand
 * @param scalar Scalar multiplicand
 * @param result Result of the vector-scalar multiplication
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_mulf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the polar angle (i.e., angle between the z-axis and the kinematic vector). Note that convention
 * implies that a vector aligned with the z-axis has a polar angle of 0.0 degrees, a vector in the xy-plane has a
 * polar angle of 90.0 degrees, and a vector aligned with the negative z-axis has a polar angle of 180.0 degrees.
 *
 * @param kv Kinematic vector
 * @param result Polar angle in the interval [0, pi] [radians]
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_polar_angle(const kvector_t *kv, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Vector-vector subtraction.
 *
 * @param kv_a First vector subtrahend
 * @param kv_b Second vector subtrahend
 * @param result Result of the vector-vector subtraction
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_sub(const kvector_t *kv_a, const kvector_t *kv_b, kvector_t *result);

/***********************************************************************************************************************
 * @brief Vector-scalar subtraction.
 *
 * @param kv Vector subtrahend
 * @param scalar Scalar subtrahend
 * @param result Result of the vector-scalar subtraction
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_subf(const kvector_t *kv, const precision_type_t scalar, kvector_t *result);

/***********************************************************************************************************************
 * @brief Scalar-vector subtraction.
 *
 * @param scalar Scalar subtrahend
 * @param kv Vector subtrahend
 * @param result Result of the scalar-vector subtraction
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_subf2(const precision_type_t scalar, const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the unit vector pointing in the direction of the kinematic vector.
 *
 * @param kv Kinematic vector
 * @param result Unit vector corresponding to the kinematic vector
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_unit(const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Sets all vector components to zero.
 *
 * @param kv Kinematic vector
 * @return Function execution status
 **********************************************************************************************************************/
int kvector_zero(kvector_t *kv);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_LIBRARY_KINEMATIC_VECTOR_H_
