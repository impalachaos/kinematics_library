#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *DCM_mul(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &DCMType))
    {
        dcm_py_t *self = (dcm_py_t *)lhs;
        if (PyObject_TypeCheck(rhs, &DCMType))
        {
            dcm_py_t *val    = (dcm_py_t *)rhs;
            dcm_py_t *result = (dcm_py_t *)PyObject_CallObject((PyObject *)&DCMType, NULL);
            int       error  = dcm_mul(&self->dcm, &val->dcm, &result->dcm);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
        else if (PyObject_TypeCheck(rhs, &KinematicVectorType))
        {
            kvector_py_t *val    = (kvector_py_t *)rhs;
            kvector_py_t *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int           error  = dcm_transform(&self->dcm, &val->vector, &result->vector);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
    }
    else if (PyObject_TypeCheck(rhs, &DCMType))
    {
        dcm_py_t *self = (dcm_py_t *)rhs;
        if (PyObject_TypeCheck(lhs, &DCMType))
        {
            dcm_py_t *val    = (dcm_py_t *)lhs;
            dcm_py_t *result = (dcm_py_t *)PyObject_CallObject((PyObject *)&DCMType, NULL);
            int       error  = dcm_mul(&val->dcm, &self->dcm, &result->dcm);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
        else if (PyObject_TypeCheck(lhs, &KinematicVectorType))
        {
            kvector_py_t *val    = (kvector_py_t *)lhs;
            kvector_py_t *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int           error  = dcm_rotate(&self->dcm, &val->vector, &result->vector);
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

PyObject *DCM_mul_inplace(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &DCMType))
    {
        dcm_py_t *self = (dcm_py_t *)lhs;
        if (PyObject_TypeCheck(rhs, &DCMType))
        {
            dcm_py_t *val = (dcm_py_t *)rhs;
            dcm_t     result;
            int       error = dcm_mul(&self->dcm, &val->dcm, &result);
            if (error != KL_NO_ERROR)
            {
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }

            self->dcm.e00 = result.e00;
            self->dcm.e01 = result.e01;
            self->dcm.e02 = result.e02;
            self->dcm.e10 = result.e10;
            self->dcm.e11 = result.e11;
            self->dcm.e12 = result.e12;
            self->dcm.e20 = result.e20;
            self->dcm.e21 = result.e21;
            self->dcm.e22 = result.e22;

            Py_XINCREF(lhs);
            return lhs;
        }
    }

    Py_RETURN_NOTIMPLEMENTED;
}
