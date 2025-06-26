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

PyObject *lla_to_ned(PyObject *self, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }

    PyObject *object = NULL;
    PyObject *ref    = NULL;
    PyObject *output = NULL;

    if (!PyArg_ParseTuple(args, "OO|O", &object, &ref, &output))
    {
        return NULL;
    }

    if (PyObject_TypeCheck(object, KinematicVectorTypeRef))
    {
        if (!PyObject_TypeCheck(ref, KinematicVectorTypeRef))
        {
            PyErr_SetString(PyExc_RuntimeError, "ref must be KinematicVector");
            return NULL;
        }
        if (output != NULL)
        {
            if (!PyObject_TypeCheck(output, KinematicVectorTypeRef))
            {
                PyErr_SetString(PyExc_RuntimeError, "out must be KinematicVector");
                return NULL;
            }
        }
        kvector_py_t *refPtr = (kvector_py_t *)ref;
        kvector_py_t *val    = (kvector_py_t *)object;
        kvector_py_t *result = (kvector_py_t *)output;
        if (result == NULL)
        {
            result = (kvector_py_t *)PyObject_CallObject((PyObject *)KinematicVectorTypeRef, NULL);
        }
        int error = transform_lla_to_ned(&val->vector, &refPtr->vector, &result->vector);
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
        kvector_t      refLla;
        PyArrayObject *pyArray = (PyArrayObject *)object;

        if (!PyArray_CheckValidDims(pyArray, output))
        {
            return NULL;
        }

        int wasKVector = 0;

        if (PyObject_TypeCheck(ref, KinematicVectorTypeRef))
        {
            kvector_py_t *refPtr = (kvector_py_t *)ref;
            refLla.x             = refPtr->vector.x;
            refLla.y             = refPtr->vector.y;
            refLla.z             = refPtr->vector.z;
            wasKVector           = 1;
        }
        else if (PyArray_Check(ref))
        {
            if (!PyArray_CheckValidRefDims(pyArray, ref))
            {
                return NULL;
            }
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "ref must be KinematicVector or NumPy array");
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
                kvector_t lla;
                kvector_t result;
                lla.x = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 0));
                lla.y = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 1));
                lla.z = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 2));
                if (wasKVector == 0)
                {
                    PyArray_SetRef((PyArrayObject *)ref, &refLla, -1);
                }
                int error = transform_lla_to_ned(&lla, &refLla, &result);
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
                kvector_t lla;
                kvector_t result;
                for (int i = 0; i < shape[0]; i++)
                {
                    lla.x = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 0));
                    lla.y = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 1));
                    lla.z = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, i, 2));
                    if (wasKVector == 0)
                    {
                        PyArray_SetRef((PyArrayObject *)ref, &refLla, i);
                    }
                    int error = transform_lla_to_ned(&lla, &refLla, &result);
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
