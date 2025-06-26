#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *DCM_transpose(dcm_py_t *self, PyObject *args, PyObject *kwargs)
{
    dcm_py_t *result = (dcm_py_t *)PyObject_CallObject((PyObject *)&DCMType, NULL);
    int       error  = dcm_transpose(&self->dcm, &result->dcm);
    if (error != KL_NO_ERROR)
    {
        Py_XDECREF(result);
        PyErr_Format(PyExc_RuntimeError, "failed to transpose DCM with the following error: %d", error);
        return NULL;
    }

    return (PyObject *)result;
}
