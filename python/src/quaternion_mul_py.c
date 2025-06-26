#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *Quaternion_mul(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &QuaternionType))
    {
        quaternion_py_t *self = (quaternion_py_t *)lhs;
        if (PyObject_TypeCheck(rhs, &QuaternionType))
        {
            quaternion_py_t *val    = (quaternion_py_t *)rhs;
            quaternion_py_t *result = (quaternion_py_t *)PyObject_CallObject((PyObject *)&QuaternionType, NULL);
            int              error  = quaternion_mul(&self->quaternion, &val->quaternion, &result->quaternion);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
    }

    Py_RETURN_NOTIMPLEMENTED;
}

PyObject *Quaternion_mul_inplace(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &QuaternionType))
    {
        quaternion_py_t *self = (quaternion_py_t *)lhs;
        if (PyObject_TypeCheck(rhs, &QuaternionType))
        {
            quaternion_py_t *val = (quaternion_py_t *)rhs;
            quaternion_t     result;
            int              error = quaternion_mul(&self->quaternion, &val->quaternion, &result);
            if (error != KL_NO_ERROR)
            {
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }

            self->quaternion.a = result.a;
            self->quaternion.b = result.b;
            self->quaternion.c = result.c;
            self->quaternion.d = result.d;

            Py_XINCREF(lhs);
            return lhs;
        }
    }

    Py_RETURN_NOTIMPLEMENTED;
}
