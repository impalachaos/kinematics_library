#include <kinematics-library/errors.h>
#include <kinematics-library/helpers.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/transforms.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

extern PyTypeObject *KinematicVectorTypeRef;

PyObject *aer_to_ned(PyObject *self, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }

    PyObject *object = NULL;
    PyObject *output = NULL;

    if (!PyArg_ParseTuple(args, "O|O", &object, &output))
    {
        return NULL;
    }

    if (PyObject_TypeCheck(object, KinematicVectorTypeRef))
    {
        if (output != NULL)
        {
            if (!PyObject_TypeCheck(output, KinematicVectorTypeRef))
            {
                PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                return NULL;
            }
        }
        kvector_py_t *val    = (kvector_py_t *)object;
        kvector_py_t *result = (kvector_py_t *)output;
        if (result == NULL)
        {
            result = (kvector_py_t *)PyObject_CallObject((PyObject *)KinematicVectorTypeRef, NULL);
        }
        int error = transform_aer_to_ned(&val->vector, &result->vector);
        if (error != KL_NO_ERROR)
        {
            if (output == NULL)
            {
                Py_XDECREF(result);
            }
            PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
            return NULL;
        }
        if (output != NULL)
        {
            Py_XINCREF(output);
        }
        return (PyObject *)result;
    }
    else if (PyArray_Check(object))
    {
        PyArrayObject *pyArray = (PyArrayObject *)object;

        if (!PyArray_CheckValidDims(pyArray, output))
        {
            return NULL;
        }

        PyArrayObject *outArray = PyArray_AllocateOutput(pyArray, (PyArrayObject *)output);

        PyArray_Descr *pyArrayDType = PyArray_DTYPE(pyArray);
        int            numDims      = PyArray_NDIM(pyArray);
        npy_intp      *shape        = PyArray_DIMS(pyArray);

        if (numDims == 1)
        {
            if (shape[0] == 3)
            {
                kvector_t aer;
                kvector_t result;
                aer.x     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 0));
                aer.y     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 1));
                aer.z     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 2));
                int error = transform_aer_to_ned(&aer, &result);
                if (error != KL_NO_ERROR)
                {
                    if (output == NULL)
                    {
                        Py_XDECREF(outArray);
                    }
                    PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                    return NULL;
                }
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR1(outArray, 0), result.x);
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR1(outArray, 1), result.y);
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR1(outArray, 2), result.z);

                if (output != NULL)
                {
                    Py_XINCREF(output);
                }
                return (PyObject *)outArray;
            }
        }
        else if (numDims == 2)
        {
            if (shape[1] == 3)
            {
                kvector_t aer;
                kvector_t result;
                for (int i = 0; i < shape[0]; i++)
                {
                    aer.x     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 0));
                    aer.y     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 1));
                    aer.z     = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 2));
                    int error = transform_aer_to_ned(&aer, &result);
                    if (error != KL_NO_ERROR)
                    {
                        if (output == NULL)
                        {
                            Py_XDECREF(outArray);
                        }
                        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
                        return NULL;
                    }
                    PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 0), result.x);
                    PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 1), result.y);
                    PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 2), result.z);
                }

                if (output != NULL)
                {
                    Py_XINCREF(output);
                }
                return (PyObject *)outArray;
            }
        }
    }

    PyErr_SetString(PyExc_RuntimeError, "invalid argument");
    return NULL;
}
