/***********************************************************************************************************************
 * Kinematic rotations library.
 **********************************************************************************************************************/

#pragma once

#include <kinematics-library/kinematic_vector.hpp>

/***********************************************************************************************************************
 * @brief Rotation sequence for creating direction cosine matrices and quaternions.
 **********************************************************************************************************************/
enum class RotationSequence
{
    BODY_XYX = 0,  // Body x-y-x rotation (i.e., roll-pitch-roll or 1-2-1)
    BODY_XYZ = 1,  // Body x-y-z rotation (i.e., roll-pitch-yaw or 1-2-3)
    BODY_XZX = 2,  // Body x-z-x rotation (i.e., roll-yaw-roll or 1-3-1)
    BODY_XZY = 3,  // Body x-z-y rotation (i.e., roll-yaw-pitch or 1-3-2)
    BODY_YXY = 4,  // Body y-x-y rotation (i.e., pitch-roll-pitch or 2-1-2)
    BODY_YXZ = 5,  // Body y-x-z rotation (i.e., pitch-roll-yaw or 2-1-3)
    BODY_YZX = 6,  // Body y-z-x rotation (i.e., pitch-yaw-roll or 2-3-1)
    BODY_YZY = 7,  // Body y-z-y rotation (i.e., pitch-yaw-pitch or 2-3-2)
    BODY_ZXY = 8,  // Body z-x-y rotation (i.e., yaw-roll-pitch or 3-1-2)
    BODY_ZXZ = 9,  // Body z-x-z rotation (i.e., yaw-roll-yaw or 3-1-3)
    BODY_ZYX = 10, // Body z-y-x rotation (i.e., yaw-pitch-roll or 3-2-1)
    BODY_ZYZ = 11  // Body z-y-z rotation (i.e., yaw-pitch-yaw or 3-2-3)
};

/***********************************************************************************************************************
 * @brief Quaternion. A quaternion contains a scalar and vector component to represent a rotation. This library uses the
 * convention where a is the scalar component and b, c, and d are the vector component. The convention takes the form
 * of a + bi + cj + dk, where a, b, c, and d are real numbers, and i, j, and k are basis elements.
 **********************************************************************************************************************/
class Quaternion;

// Note: The above is a forward declaration of Quaternion so that it may be used to construct a direction cosine matrix

/***********************************************************************************************************************
 * @brief Direction cosine matrix (DCM). A direction cosine matrix is a 3 x 3 matrix for coordinate frame transforms
 * and vector rotations in kinematic applications. This library uses the a standard right-handed convention with
 * pre-multiplication of kinematic vectors (i.e., DCM * v) for coordinate frame transformations and post-multiplication
 * (i.e., v * DCM) for vector rotations.
 **********************************************************************************************************************/
class DCM
{
public:
    /*******************************************************************************************************************
     * @brief Creates an indentify direction cosine matrix.
     ******************************************************************************************************************/
    DCM();

    /*******************************************************************************************************************
     * @brief Creates a direction cosine matrix using the specified rotation sequence.
     *
     * @param a1_rad: First rotation angle [radians]
     * @param a2_rad: Second rotation angle [radians]
     * @param a3_rad: Third rotation angle [radians]
     * @param seq Rotation sequence
     ******************************************************************************************************************/
    DCM(const precision_type_t &a1_rad,
        const precision_type_t &a2_rad,
        const precision_type_t &a3_rad,
        const RotationSequence &seq);

    /*******************************************************************************************************************
     * @brief Creates a direction cosine matrix from a quaternion.
     *
     * @param q Quaternion
     ******************************************************************************************************************/
    DCM(const Quaternion &q);

    /*******************************************************************************************************************
     * @brief Gets element 00 (i.e., row 0, column 0) of the direction cosine matrix.
     *
     * @return Element 00
     ******************************************************************************************************************/
    precision_type_t get_e00() const;

    /*******************************************************************************************************************
     * @brief Gets element 01 (i.e., row 0, column 1) of the direction cosine matrix.
     *
     * @return Element 01
     ******************************************************************************************************************/
    precision_type_t get_e01() const;

    /*******************************************************************************************************************
     * @brief Gets element 02 (i.e., row 0, column 2) of the direction cosine matrix.
     *
     * @return Element 02
     ******************************************************************************************************************/
    precision_type_t get_e02() const;

    /*******************************************************************************************************************
     * @brief Gets element 10 (i.e., row 1, column 0) of the direction cosine matrix.
     *
     * @return Element 10
     ******************************************************************************************************************/
    precision_type_t get_e10() const;

    /*******************************************************************************************************************
     * @brief Gets element 11 (i.e., row 1, column 1) of the direction cosine matrix.
     *
     * @return Element 11
     ******************************************************************************************************************/
    precision_type_t get_e11() const;

    /*******************************************************************************************************************
     * @brief Gets element 12 (i.e., row 1, column 2) of the direction cosine matrix.
     *
     * @return Element 12
     ******************************************************************************************************************/
    precision_type_t get_e12() const;

    /*******************************************************************************************************************
     * @brief Gets element 20 (i.e., row 2, column 0) of the direction cosine matrix.
     *
     * @return Element 20
     ******************************************************************************************************************/
    precision_type_t get_e20() const;

    /*******************************************************************************************************************
     * @brief Gets element 21 (i.e., row 2, column 1) of the direction cosine matrix.
     *
     * @return Element 21
     ******************************************************************************************************************/
    precision_type_t get_e21() const;

    /*******************************************************************************************************************
     * @brief Gets element 22 (i.e., row 2, column 2) of the direction cosine matrix.
     *
     * @return Element 22
     ******************************************************************************************************************/
    precision_type_t get_e22() const;

    /*******************************************************************************************************************
     * @brief Matrix-matrix multiplication. For coordinate transforms, if matrix one goes from frame ABC to UVW and
     * matrix two goes from frame UVW to XYZ, then the result goes from frame to ABC to XYZ. For vector rotations, the
     * result is equivalent to applying the first rotation followed by then the second rotation.
     *
     * @param rhs Direction cosine matrix multiplicand
     * @return Result of the matrix-matrix multiplication
     ******************************************************************************************************************/
    DCM operator*(const DCM &rhs) const;

    /*******************************************************************************************************************
     * @brief Performs a coordinate transformation to a vector. This function performs a passive vector rotation, such
     * that the original vector is represented in a new coordinate frame, according to the rotation specified by the
     * direction cosine matrix.
     *
     * @param rhs Kinematic vector multiplicand
     * @return Transformed kinematic vector
     ******************************************************************************************************************/
    KinematicVector operator*(const KinematicVector &rhs) const;

    /*******************************************************************************************************************
     * @brief Direction cosine matrix equality operator.
     *
     * @param rhs Direction cosine matrix to compare
     * @return Indicates if the direction cosine matrices are equal
     ******************************************************************************************************************/
    bool operator==(const DCM &rhs) const;

    /*******************************************************************************************************************
     * @brief Direction cosine matrix inequality operator.
     *
     * @param rhs Direction cosine matrix to compare
     * @return Indicates if the direction cosine matrices are not equal
     ******************************************************************************************************************/
    bool operator!=(const DCM &rhs) const;

    /*******************************************************************************************************************
     * @brief Rotates a vector within a coordinate frame, storing the calculation in result. This function performs an
     * active vector rotation, such that the original vector is rotated within a single coordinate frame, according to
     * the rotation specified by the direction cosine matrix.
     *
     * @param kv Vector to rotate
     * @param result Rotated vector in the same coordinate frame as the original vector
     ******************************************************************************************************************/
    void rotate(const KinematicVector &kv, KinematicVector &result) const;

    /*******************************************************************************************************************
     * @brief Rotates a vector within a coordinate frame, as an in-place operation. This function performs an active
     * vector rotation, such that the original vector is rotated within a single coordinate frame, according to the
     * rotation specified by the direction cosine matrix.
     *
     * @param kv Vector to rotate in-place
     ******************************************************************************************************************/
    void rotate(KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Sets the direction cosine matrix to the specified rotation sequence.
     *
     * @param a1_rad: First rotation angle [radians]
     * @param a2_rad: Second rotation angle [radians]
     * @param a3_rad: Third rotation angle [radians]
     * @param seq Rotation sequence
     ******************************************************************************************************************/
    void
    set(const precision_type_t &a1_rad,
        const precision_type_t &a2_rad,
        const precision_type_t &a3_rad,
        const RotationSequence &seq);

    /*******************************************************************************************************************
     * @brief Sets the direction cosine matrix from a quaternion.
     *
     * @param q Quaternion
     ******************************************************************************************************************/
    void set(const Quaternion &q);

    /*******************************************************************************************************************
     * @brief Performs a coordinate transformation to a vector, storing the calculation in result. This function
     * performs a passive vector rotation, such that the original vector is represented in a new coordinate frame,
     * according to the rotation specified by the direction cosine matrix.
     *
     * @param kv Vector to perform coordinate transformation
     * @param result: Vector representation in new coordinate frame
     ******************************************************************************************************************/
    void transform(const KinematicVector &kv, KinematicVector &result) const;

    /*******************************************************************************************************************
     * @brief Performs a coordinate transformation to a vector, as an in-place operation. This function performs a
     * passive vector rotation, such that the original vector is represented in a new coordinate frame, according to the
     * rotation specified by the direction cosine matrix.
     *
     * @param kv Vector to perform coordinate transformation in-place
     ******************************************************************************************************************/
    void transform(KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the transpose the direction cosine matrix, storing the calculation in dcm. This is equivalent
     * to taking the inverse.
     *
     * @param dcm Transpose of the direction cosine matrix
     ******************************************************************************************************************/
    void transpose(DCM &dcm) const;

    /*******************************************************************************************************************
     * @brief Calculates the transpose the direction cosine matrix. This is equivalent to taking the inverse.
     *
     * @return Transpose of the direction cosine matrix
     ******************************************************************************************************************/
    DCM transpose() const;

private:
    precision_type_t e00; // Matrix component 00 (row 0, column 0)
    precision_type_t e01; // Matrix component 01 (row 0, column 1)
    precision_type_t e02; // Matrix component 02 (row 0, column 2)
    precision_type_t e10; // Matrix component 10 (row 1, column 0)
    precision_type_t e11; // Matrix component 11 (row 1, column 1)
    precision_type_t e12; // Matrix component 12 (row 1, column 2)
    precision_type_t e20; // Matrix component 20 (row 2, column 0)
    precision_type_t e21; // Matrix component 21 (row 2, column 1)
    precision_type_t e22; // Matrix component 22 (row 2, column 2)
};

/***********************************************************************************************************************
 * @brief Rotates a vector within a coordinate frame. This function performs an active vector rotation, such that the
 * original vector is rotated within a single coordinate frame, according to the rotation specified by the direction
 * cosine matrix.
 *
 * @param lhs Kinematic vector multiplicand
 * @param rhs Direction cosine matrix multiplicand
 * @return Rotated kinematic vector
 **********************************************************************************************************************/
KinematicVector operator*(const KinematicVector &lhs, const DCM &rhs);

class Quaternion
{
public:
    /*******************************************************************************************************************
     * @brief Creates an identity quaternion.
     ******************************************************************************************************************/
    Quaternion();

    /*******************************************************************************************************************
     * @brief Creates a quaternion using the specified rotation sequence.
     *
     * @param a1_rad: First rotation angle [radians]
     * @param a2_rad: Second rotation angle [radians]
     * @param a3_rad: Third rotation angle [radians]
     * @param seq Rotation sequence
     ******************************************************************************************************************/
    Quaternion(
        const precision_type_t &a1_rad,
        const precision_type_t &a2_rad,
        const precision_type_t &a3_rad,
        const RotationSequence &seq);

    /*******************************************************************************************************************
     * @brief Creates a quaternion from a direction cosine matrix.
     *
     * @param dcm Direction cosine matrix
     ******************************************************************************************************************/
    Quaternion(const DCM &dcm);

    /*******************************************************************************************************************
     * @brief Calculates the rotation angle associated with the quaternion.
     *
     * @return Quaternion rotation angle [radians]
     ******************************************************************************************************************/
    precision_type_t angle() const;

    /*******************************************************************************************************************
     * @brief Calculates the rotation axis associated with the quaternion.
     *
     * @param kv Quaternion rotation axis as a kinematic unit vector
     ******************************************************************************************************************/
    void axis(KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Calculates the conjugate the quaternion, storing the calculation in q.
     *
     * @param q Conjugate of the quaternion
     ******************************************************************************************************************/
    void conjugate(Quaternion &q) const;

    /*******************************************************************************************************************
     * @brief Calculates the conjugate the quaternion.
     *
     * @return Conjugate of the quaternion
     ******************************************************************************************************************/
    Quaternion conjugate() const;

    /*******************************************************************************************************************
     * @brief Gets the a (i.e., scalar) component of the quaternion.
     *
     * @return Quaternion a component
     ******************************************************************************************************************/
    precision_type_t get_a() const;

    /*******************************************************************************************************************
     * @brief Gets the b (i.e., component corresponding to the i basis element) component of the quaternion.
     *
     * @return Quaternion b component
     ******************************************************************************************************************/
    precision_type_t get_b() const;

    /*******************************************************************************************************************
     * @brief Gets the c (i.e., component corresponding to the i basis element) component of the quaternion.
     *
     * @return Quaternion c component
     ******************************************************************************************************************/
    precision_type_t get_c() const;

    /*******************************************************************************************************************
     * @brief Gets the d (i.e., component corresponding to the i basis element) component of the quaternion.
     *
     * @return Quaternion d component
     ******************************************************************************************************************/
    precision_type_t get_d() const;

    /*******************************************************************************************************************
     * @brief Calculates the inverse the quaternion, storing the calculation in q.
     *
     * @param q Inverse of the quaternion
     ******************************************************************************************************************/
    void inverse(Quaternion &q) const;

    /*******************************************************************************************************************
     * @brief Calculates the inverse the quaternion.
     *
     * @return Inverse of the quaternion
     ******************************************************************************************************************/
    Quaternion inverse() const;

    /*******************************************************************************************************************
     * @brief Calculates the norm of the quaternion.
     *
     * @return Quaternion norm
     ******************************************************************************************************************/
    precision_type_t norm() const;

    /*******************************************************************************************************************
     * @brief Quaternion-quaternion multiplication.
     *
     * @param rhs Quaternion multiplicand
     * @return Result of the quaternion-quaternion multiplication
     ******************************************************************************************************************/
    Quaternion operator*(const Quaternion &rhs) const;

    /*******************************************************************************************************************
     * @brief Quaternion equality operator.
     *
     * @param rhs Quaternion to compare
     * @return Indicates if the quaternions are equal
     ******************************************************************************************************************/
    bool operator==(const Quaternion &rhs) const;

    /*******************************************************************************************************************
     * @brief Quaternion inequality operator.
     *
     * @param rhs Quaternion to compare
     * @return Indicates if the quaternions are not equal
     ******************************************************************************************************************/
    bool operator!=(const Quaternion &rhs) const;

    /*******************************************************************************************************************
     * @brief Rotates a vector within a coordinate frame, storing the calculation in result. This function performs an
     * active vector rotation, such that the original vector is rotated within a single coordinate frame, according to
     * the rotation specified by the quaternion.
     *
     * @param kv Vector to rotate
     * @param result Rotated vector in the same coordinate frame as the original vector
     ******************************************************************************************************************/
    void rotate(const KinematicVector &kv, KinematicVector &result) const;

    /*******************************************************************************************************************
     * @brief Rotates a vector within a coordinate frame, as an in-place operation. This function performs an active
     * vector rotation, such that the original vector is rotated within a single coordinate frame, according to the
     * rotation specified by the quaternion.
     *
     * @param kv Vector to rotate in-place
     ******************************************************************************************************************/
    void rotate(KinematicVector &kv) const;

    /*******************************************************************************************************************
     * @brief Sets the quaternion using the specified rotation sequence.
     *
     * @param a1_rad: First rotation angle [radians]
     * @param a2_rad: Second rotation angle [radians]
     * @param a3_rad: Third rotation angle [radians]
     * @param seq Rotation sequence
     ******************************************************************************************************************/
    void
    set(const precision_type_t &a1_rad,
        const precision_type_t &a2_rad,
        const precision_type_t &a3_rad,
        const RotationSequence &seq);

    /*******************************************************************************************************************
     * @brief Sets the quaternion from a direction cosine matrix.
     *
     * @param dcm Direction cosine matrix
     ******************************************************************************************************************/
    void set(const DCM &dcm);

    /*******************************************************************************************************************
     * @brief Calculates the squared norm of the quaternion.
     *
     * @return Quaternion norm squared
     ******************************************************************************************************************/
    precision_type_t square() const;

    /*******************************************************************************************************************
     * @brief Performs a coordinate transformation to a vector, storing the calculation in result. This function
     * performs a passive vector rotation, such that the original vector is represented in a new coordinate frame,
     * according to the rotation specified by the quaternion.
     *
     * @param kv Vector to perform coordinate transformation
     * @param result: Vector representation in new coordinate frame
     ******************************************************************************************************************/
    void transform(const KinematicVector &kv, KinematicVector &result) const;

    /*******************************************************************************************************************
     * @brief Performs a coordinate transformation to a vector, as an in-place operation. This function performs a
     * passive vector rotation, such that the original vector is represented in a new coordinate frame, according to the
     * rotation specified by the quaternion.
     *
     * @param kv Vector to perform coordinate transformation in-place
     ******************************************************************************************************************/
    void transform(KinematicVector &kv) const;

private:
    precision_type_t a; // Quaternion scalar component
    precision_type_t b; // Quaternion component corresponding to i basis element
    precision_type_t c; // Quaternion component corresponding to j basis element
    precision_type_t d; // Quaternion component corresponding to k basis element
};
