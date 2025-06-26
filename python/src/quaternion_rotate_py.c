#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *Quaternion_rotate(quaternion_py_t *self, PyObject *args, PyObject *kwargs)
{
    PyObject *object = NULL;

    if (!PyArg_ParseTuple(args, "O!", &KinematicVectorType, &object))
    {
        return NULL;
    }

    kvector_py_t *arg    = (kvector_py_t *)object;
    kvector_py_t *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
    int           error  = quaternion_rotate(&self->quaternion, &arg->vector, &result->vector);
    if (error != KL_NO_ERROR)
    {
        Py_XDECREF(result);
        PyErr_Format(PyExc_RuntimeError, "failed to rotate vector with the following error: %d", error);
        return NULL;
    }

    return (PyObject *)result;
}
