#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *Quaternion_angle(quaternion_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = quaternion_angle(&self->quaternion, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_Format(PyExc_RuntimeError, "failed to calculate angle with the following error: %d", error);
        return NULL;
    }

    return PyFloat_FromDouble(result);
}
