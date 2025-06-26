#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *Quaternion_conjugate(quaternion_py_t *self, PyObject *args, PyObject *kwargs)
{
    quaternion_py_t *result = (quaternion_py_t *)PyObject_CallObject((PyObject *)&QuaternionType, NULL);
    int              error  = quaternion_conjugate(&self->quaternion, &result->quaternion);
    if (error != KL_NO_ERROR)
    {
        Py_XDECREF(result);
        PyErr_Format(PyExc_RuntimeError, "failed to conjugate quaternion with the following error: %d", error);
        return NULL;
    }

    return (PyObject *)result;
}
