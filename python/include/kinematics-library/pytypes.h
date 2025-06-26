#pragma once

#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/rotations.h>
#include <kinematics-library/transforms.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>

#ifndef KINEMATIC_VECTOR_ONLY
extern PyTypeObject KinematicVectorType;
#endif // KINEMATIC_VECTOR_ONLY

typedef struct kvector_py_t
{
    PyObject_HEAD kvector_t vector;
} kvector_py_t;

#ifndef KINEMATIC_VECTOR_ONLY
extern PyTypeObject DCMType;

typedef struct dcm_py_t
{
    PyObject_HEAD dcm_t dcm;
} dcm_py_t;

extern PyTypeObject QuaternionType;

typedef struct quaternion_py_t
{
    PyObject_HEAD quaternion_t quaternion;
} quaternion_py_t;

extern PyTypeObject *RotationSequenceType;

#endif // KINEMATIC_VECTOR_ONLY
