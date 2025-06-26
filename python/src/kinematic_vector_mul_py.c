#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyObject *KinematicVector_mul(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &KinematicVectorType))
    {
        kvector_py_t *self = (kvector_py_t *)lhs;
        if (PyFloat_Check(rhs))
        {
            precision_type_t val    = (precision_type_t)PyFloat_AsDouble(rhs);
            kvector_py_t    *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int              error  = kvector_mulf(&self->vector, val, &result->vector);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
        else if (PyLong_Check(rhs))
        {
            precision_type_t val    = (precision_type_t)PyLong_AsDouble(rhs);
            kvector_py_t    *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int              error  = kvector_mulf(&self->vector, val, &result->vector);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
    }
    else if (PyObject_TypeCheck(rhs, &KinematicVectorType))
    {
        kvector_py_t *self = (kvector_py_t *)rhs;
        if (PyFloat_Check(lhs))
        {
            precision_type_t val    = (precision_type_t)PyFloat_AsDouble(lhs);
            kvector_py_t    *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int              error  = kvector_mulf(&self->vector, val, &result->vector);
            if (error != KL_NO_ERROR)
            {
                Py_XDECREF(result);
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            return (PyObject *)result;
        }
        else if (PyLong_Check(lhs))
        {
            precision_type_t val    = (precision_type_t)PyLong_AsDouble(lhs);
            kvector_py_t    *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
            int              error  = kvector_mulf(&self->vector, val, &result->vector);
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

PyObject *KinematicVector_mul_inplace(PyObject *lhs, PyObject *rhs)
{
    if (PyObject_TypeCheck(lhs, &KinematicVectorType))
    {
        kvector_py_t *self = (kvector_py_t *)lhs;
        if (PyFloat_Check(rhs))
        {
            precision_type_t val   = (precision_type_t)PyFloat_AsDouble(rhs);
            int              error = kvector_mulf(&self->vector, val, &self->vector);
            if (error != KL_NO_ERROR)
            {
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            Py_XINCREF(lhs);
            return lhs;
        }
        else if (PyLong_Check(rhs))
        {
            precision_type_t val   = (precision_type_t)PyLong_AsDouble(rhs);
            int              error = kvector_mulf(&self->vector, val, &self->vector);
            if (error != KL_NO_ERROR)
            {
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
            Py_XINCREF(lhs);
            return lhs;
        }
    }

    Py_RETURN_NOTIMPLEMENTED;
}
