#ifndef KINEMATICS_LIBRARY_ROTATIONS_H_
#define KINEMATICS_LIBRARY_ROTATIONS_H_

#include <kinematics-library/kinematic_vector.h>

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

#define ROTATION_SEQ_BODY_XYX (int)0  // Body x-y-x rotation (i.e., roll-pitch-roll or 1-2-1)
#define ROTATION_SEQ_BODY_XYZ (int)1  // Body x-y-z rotation (i.e., roll-pitch-yaw or 1-2-3)
#define ROTATION_SEQ_BODY_XZX (int)2  // Body x-z-x rotation (i.e., roll-yaw-roll or 1-3-1)
#define ROTATION_SEQ_BODY_XZY (int)3  // Body x-z-y rotation (i.e., roll-yaw-pitch or 1-3-2)
#define ROTATION_SEQ_BODY_YXY (int)4  // Body y-x-y rotation (i.e., pitch-roll-pitch or 2-1-2)
#define ROTATION_SEQ_BODY_YXZ (int)5  // Body y-x-z rotation (i.e., pitch-roll-yaw or 2-1-3)
#define ROTATION_SEQ_BODY_YZX (int)6  // Body y-z-x rotation (i.e., pitch-yaw-roll or 2-3-1)
#define ROTATION_SEQ_BODY_YZY (int)7  // Body y-z-y rotation (i.e., pitch-yaw-pitch or 2-3-2)
#define ROTATION_SEQ_BODY_ZXY (int)8  // Body z-x-y rotation (i.e., yaw-roll-pitch or 3-1-2)
#define ROTATION_SEQ_BODY_ZXZ (int)9  // Body z-x-z rotation (i.e., yaw-roll-yaw or 3-1-3)
#define ROTATION_SEQ_BODY_ZYX (int)10 // Body z-y-x rotation (i.e., yaw-pitch-roll or 3-2-1)
#define ROTATION_SEQ_BODY_ZYZ (int)11 // Body z-y-z rotation (i.e., yaw-pitch-yaw or 3-2-3)

/***********************************************************************************************************************
 * @brief quaternion_t. A quaternion contains a scalar and vector component to represent a rotation. This library uses
 * the convention where a is the scalar component and b, c, and d are the vector component. The convention takes the
 * form of a + bi + cj + dk, where a, b, c, and d are real numbers, and i, j, and k are basis elements.
 **********************************************************************************************************************/
typedef struct quaternion_t
{
    precision_type_t a; ///< Quaternion scalar component
    precision_type_t b; ///< Quaternion component corresponding to i basis element
    precision_type_t c; ///< Quaternion component corresponding to j basis element
    precision_type_t d; ///< Quaternion component corresponding to k basis element
} quaternion_t;

/***********************************************************************************************************************
 * @brief Direction cosine matrix (DCM). A direction cosine matrix is a 3 x 3 matrix for coordinate frame transforms
 * and vector rotations in kinematic applications. This library uses the a standard right-handed convention with
 * pre-multiplication of kinematic vectors (i.e., DCM * v) for coordinate frame transformations and post-multiplication
 * (i.e., v * DCM) for vector rotations.
 **********************************************************************************************************************/
typedef struct dcm_t
{
    precision_type_t e00; ///< Matrix component 00 (row 0, column 0)
    precision_type_t e01; ///< Matrix component 01 (row 0, column 1)
    precision_type_t e02; ///< Matrix component 02 (row 0, column 2)
    precision_type_t e10; ///< Matrix component 10 (row 1, column 0)
    precision_type_t e11; ///< Matrix component 11 (row 1, column 1)
    precision_type_t e12; ///< Matrix component 12 (row 1, column 2)
    precision_type_t e20; ///< Matrix component 20 (row 2, column 0)
    precision_type_t e21; ///< Matrix component 21 (row 2, column 1)
    precision_type_t e22; ///< Matrix component 22 (row 2, column 2)
} dcm_t;

/***********************************************************************************************************************
 * @brief Compares the two direction cosine matricies and returns 0 if they are equal, 1 otherwise.
 *
 * @param dcm_a First direction cosine matrix
 * @param dcm_b Second direction cosine matrix
 * @return 0 if the matricies are equal, 1 otherwise
 **********************************************************************************************************************/
int dcm_compare(const dcm_t *dcm_a, const dcm_t *dcm_b);

/***********************************************************************************************************************
 * @brief Initializes a direction cosine matrix using the specified rotation sequence.
 *
 * @param dcm Direction cosine matrix
 * @param a1_rad First rotation angle [radians]
 * @param a2_rad Second rotation angle [radians]
 * @param a3_rad Third rotation angle [radians]
 * @param seq Rotation sequence
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_init(
    dcm_t                 *dcm,
    const precision_type_t a1_rad,
    const precision_type_t a2_rad,
    const precision_type_t a3_rad,
    const int              seq);

/***********************************************************************************************************************
 * @brief Initializes a direction cosine matrix as an identity matrix.
 *
 * @param dcm Direction cosine matrix
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_init_identity(dcm_t *dcm);

/***********************************************************************************************************************
 * @brief Initializes a direction cosine matrix from a quaternion.
 *
 * @param dcm Direction cosine matrix
 * @param q Quaternion
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_init_quaternion(dcm_t *dcm, const quaternion_t *q);

/***********************************************************************************************************************
 * @brief Matrix-matrix multiplication. For coordinate transforms, if matrix one goes from frame ABC to UVW and
 * matrix two goes from frame UVW to XYZ, then the result goes from frame to ABC to XYZ. For vector rotations, the
 * result is equivalent to applying the first rotation followed by then the second rotation.
 *
 * @param dcm_a Direction cosine matrix multiplicand
 * @param dcm_b Direction cosine matrix multiplicand
 * @param result Result of the matrix-matrix multiplication
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_mul(const dcm_t *dcm_a, const dcm_t *dcm_b, dcm_t *result);

/***********************************************************************************************************************
 * @brief Rotates a vector within a coordinate frame, storing the calculation in result. This function performs an
 * active vector rotation, such that the original vector is rotated within a single coordinate frame, according to
 * the rotation specified by the direction cosine matrix.
 *
 * @param dcm Direction cosine matrix
 * @param kv Vector to rotate
 * @param result Rotated vector in the same coordinate frame as the original vector
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_rotate(const dcm_t *dcm, const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Performs a coordinate transformation to a vector, storing the calculation in result. This function
 * performs a passive vector rotation, such that the original vector is represented in a new coordinate frame,
 * according to the rotation specified by the direction cosine matrix.
 *
 * @param dcm Direction cosine matrix
 * @param kv Vector to perform coordinate transformation
 * @param result Vector representation in new coordinate frame
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_transform(const dcm_t *dcm, const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the transpose the direction cosine matrix, storing the calculation in dcm. This is equivalent
 * to taking the inverse.
 *
 * @param dcm Direction cosine matrix
 * @param result Transpose of the direction cosine matrix
 * @return Function execution status
 **********************************************************************************************************************/
int dcm_transpose(const dcm_t *dcm, dcm_t *result);

/***********************************************************************************************************************
 * @brief Calculates the rotation angle associated with the quaternion.
 *
 * @param q Quaternion
 * @param result Quaternion rotation angle [radians]
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_angle(const quaternion_t *q, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Calculates the rotation axis associated with the quaternion.
 *
 * @param q Quaternion
 * @param kv Quaternion rotation axis as a kinematic unit vector
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_axis(const quaternion_t *q, kvector_t *kv);

/***********************************************************************************************************************
 * @brief Compares the two quaternions and returns 0 if they are equal, 1 otherwise.
 *
 * @param q_a First quaternion
 * @param q_b Second quaternion
 * @return 0 if the quaternions are equal, 1 otherwise
 **********************************************************************************************************************/
int quaternion_compare(const quaternion_t *q_a, const quaternion_t *q_b);

/***********************************************************************************************************************
 * @brief Calculates the conjugate the quaternion, storing the calculation in result.
 *
 * @param q Quaternion
 * @param result Conjugate of the quaternion
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_conjugate(const quaternion_t *q, quaternion_t *result);

/***********************************************************************************************************************
 * @brief Initializes a quaternion using the specified rotation sequence.
 *
 * @param q Quaternion
 * @param a1_rad First rotation angle [radians]
 * @param a2_rad Second rotation angle [radians]
 * @param a3_rad Third rotation angle [radians]
 * @param seq Rotation sequence
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_init(
    quaternion_t          *q,
    const precision_type_t a1_rad,
    const precision_type_t a2_rad,
    const precision_type_t a3_rad,
    const int              seq);

/***********************************************************************************************************************
 * @brief Initializes a quaternion from a direction cosine matrix.
 *
 * @param q Quaternion
 * @param dcm Direction cosine matrix
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_init_dcm(quaternion_t *q, const dcm_t *dcm);

/***********************************************************************************************************************
 * @brief Initializes an identity quaternion.
 *
 * @param q Quaternion
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_init_identity(quaternion_t *q);

/***********************************************************************************************************************
 * @brief Calculates the inverse the quaternion.
 *
 * @param q Quaternion
 * @param result Inverse of the quaternion
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_inverse(const quaternion_t *q, quaternion_t *result);

/***********************************************************************************************************************
 * @brief Quaternion-quaternion multiplication.
 *
 * @param q_a Quaternion multiplicand
 * @param q_b Quaternion multiplicand
 * @param result Result of the quaternion-quaternion multiplication
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_mul(const quaternion_t *q_a, const quaternion_t *q_b, quaternion_t *result);

/***********************************************************************************************************************
 * @brief Calculates the norm of the quaternion.
 *
 * @param q Quaternion
 * @param result Quaternion norm
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_norm(const quaternion_t *q, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Rotates a vector within a coordinate frame, storing the calculation in result. This function performs an
 * active vector rotation, such that the original vector is rotated within a single coordinate frame, according to
 * the rotation specified by the quaternion.
 *
 * @param q Quaternion
 * @param kv Vector to rotate
 * @param result Rotated vector in the same coordinate frame as the original vector
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_rotate(const quaternion_t *q, const kvector_t *kv, kvector_t *result);

/***********************************************************************************************************************
 * @brief Calculates the squared norm of the quaternion.
 *
 * @param q Quaternion
 * @param result Quaternion norm squared
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_square(const quaternion_t *q, precision_type_t *result);

/***********************************************************************************************************************
 * @brief Performs a coordinate transformation to a vector, storing the calculation in result. This function
 * performs a passive vector rotation, such that the original vector is represented in a new coordinate frame,
 * according to the rotation specified by the quaternion.
 *
 * @param q Quaternion
 * @param kv Vector to perform coordinate transformation
 * @param result Vector representation in new coordinate frame
 * @return Function execution status
 **********************************************************************************************************************/
int quaternion_transform(const quaternion_t *q, const kvector_t *kv, kvector_t *result);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_LIBRARY_ROTATIONS_H_
