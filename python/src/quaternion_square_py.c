#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *Quaternion_square(quaternion_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = quaternion_square(&self->quaternion, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_Format(PyExc_RuntimeError, "failed to calculate normal with the following error: %d", error);
        return NULL;
    }

    return PyFloat_FromDouble(result);
}
