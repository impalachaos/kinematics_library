#include <kinematics-library/errors.h>
#include <kinematics-library/geometry.h>
#include <kinematics-library/helpers.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>
#include <universal-constants/constants.h>

PyObject *vincenty_inverse_py(PyObject *self, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }

    PyObject        *lat_a_deg_obj = NULL;
    PyObject        *lon_a_deg_obj = NULL;
    PyObject        *lat_b_deg_obj = NULL;
    PyObject        *lon_b_deg_obj = NULL;
    precision_type_t abs_tol       = 0.0;
    PyObject        *output        = NULL;

#ifdef MIN_MEMORY_FOOTPRINT
    static char *parseString         = "OOOOf";
    static char *return_tuple_format = "fff";
#else
    static char *parseString         = "OOOOd";
    static char *return_tuple_format = "ddd";
#endif

    if (!PyArg_ParseTuple(args, parseString, &lat_a_deg_obj, &lon_a_deg_obj, &lat_b_deg_obj, &lon_b_deg_obj, &abs_tol))
    {
        return NULL;
    }

    kvector_t lla_a = {0.0, 0.0, 0.0};
    kvector_t lla_b = {0.0, 0.0, 0.0};

    precision_type_t range_m        = 0.0;
    precision_type_t bearing_ab_rad = 0.0;
    precision_type_t bearing_ba_rad = 0.0;

    if (PyArray_Check(lat_a_deg_obj))
    {
        if (!PyArray_Check(lon_a_deg_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_deg must be an array_like");
            return NULL;
        }
        if (!PyArray_Check(lat_b_deg_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "range_m must be an array_like");
            return NULL;
        }
        if (!PyArray_Check(lon_b_deg_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg must be an array_like");
            return NULL;
        }

        npy_intp wantDims[] = {-1};

        PyArrayObject *lat_a_deg_array = (PyArrayObject *)lat_a_deg_obj;
        PyArrayObject *lon_a_deg_array = (PyArrayObject *)lon_a_deg_obj;
        PyArrayObject *lat_b_deg_array = (PyArrayObject *)lat_b_deg_obj;
        PyArrayObject *lon_b_deg_array = (PyArrayObject *)lon_b_deg_obj;

        if (!PyArray_HasDims(lat_a_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lat_a_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(lon_a_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_a_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(lat_b_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lat_b_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(lon_b_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_b_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_a_deg_array, lon_a_deg_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_a_deg_array, lat_b_deg_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_a_deg_array, lon_b_deg_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }

        int       numDims = PyArray_NDIM(lat_a_deg_array);
        npy_intp *shape   = PyArray_DIMS(lat_a_deg_array);

        if (numDims == 1)
        {
            npy_intp outShape[] = {shape[0], 3};

            PyArrayObject *outArray = PyArray_AllocateOutputWithDim((PyArrayObject *)output, 2, &outShape[0]);

            for (int i = 0; i < shape[0]; i++)
            {
                lla_a.x = PyArray_ToPrecisionType(lat_a_deg_array, PyArray_GETPTR1(lat_a_deg_array, i)) * DEG_TO_RAD;
                lla_a.y = PyArray_ToPrecisionType(lon_a_deg_array, PyArray_GETPTR1(lon_a_deg_array, i)) * DEG_TO_RAD;
                lla_b.x = PyArray_ToPrecisionType(lat_b_deg_array, PyArray_GETPTR1(lat_b_deg_array, i)) * DEG_TO_RAD;
                lla_b.y = PyArray_ToPrecisionType(lon_b_deg_array, PyArray_GETPTR1(lon_b_deg_array, i)) * DEG_TO_RAD;

                int error = vincenty_inverse(&lla_a, &lla_b, abs_tol, &range_m, &bearing_ab_rad, &bearing_ba_rad);
                if (error != KL_NO_ERROR)
                {
                    if (output == NULL)
                    {
                        Py_XDECREF(outArray);
                    }
                    PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
                    return NULL;
                }

                PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 0), range_m);
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 1), bearing_ab_rad * RAD_TO_DEG);
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 2), bearing_ba_rad * RAD_TO_DEG);
            }

            if (output != NULL)
            {
                Py_XINCREF(output);
            }
            return (PyObject *)outArray;
        }

        PyErr_SetString(PyExc_RuntimeError, "Invalid argument");
        return NULL;
    }
    else if (PyFloat_Check(lat_a_deg_obj) || PyLong_Check(lat_a_deg_obj))
    {
        if (PyFloat_Check(lat_a_deg_obj))
        {
            lla_a.x = ((precision_type_t)PyFloat_AsDouble(lat_a_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            lla_a.x = ((precision_type_t)PyLong_AsDouble(lat_a_deg_obj)) * DEG_TO_RAD;
        }

        if (PyFloat_Check(lon_a_deg_obj))
        {
            lla_a.y = ((precision_type_t)PyFloat_AsDouble(lon_a_deg_obj)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_a_deg_obj))
        {
            lla_a.y = ((precision_type_t)PyLong_AsDouble(lon_a_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_deg must be a number");
            return NULL;
        }

        if (PyFloat_Check(lat_b_deg_obj))
        {
            lla_b.x = ((precision_type_t)PyFloat_AsDouble(lat_b_deg_obj)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lat_b_deg_obj))
        {
            lla_b.x = ((precision_type_t)PyLong_AsDouble(lat_b_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "range_m must be a number");
            return NULL;
        }

        if (PyFloat_Check(lon_b_deg_obj))
        {
            lla_b.y = ((precision_type_t)PyFloat_AsDouble(lon_b_deg_obj)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_b_deg_obj))
        {
            lla_b.y = ((precision_type_t)PyLong_AsDouble(lon_b_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg must be a number");
            return NULL;
        }

        int error = vincenty_inverse(&lla_a, &lla_b, abs_tol, &range_m, &bearing_ab_rad, &bearing_ba_rad);
        if (error != KL_NO_ERROR)
        {
            PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
            return NULL;
        }

        PyObject *range_bearing_bearing =
            Py_BuildValue(return_tuple_format, range_m, bearing_ab_rad * RAD_TO_DEG, bearing_ba_rad * RAD_TO_DEG);
        if (range_bearing_bearing == NULL)
        {
            PyErr_SetString(PyExc_RuntimeError, "Error creating range-bearing-bearing return tuple");
            return NULL;
        }

        return range_bearing_bearing;
    }

    PyObject *lat_a_deg_iter = PyObject_GetIter(lat_a_deg_obj);
    if (lat_a_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *lon_a_deg_iter = PyObject_GetIter(lon_a_deg_obj);
    if (lon_a_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *lat_b_deg_iter = PyObject_GetIter(lat_b_deg_obj);
    if (lat_b_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *lon_b_deg_iter = PyObject_GetIter(lon_b_deg_obj);
    if (lon_b_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    output = PyList_New(0);

    PyObject *lat_a_deg_next = PyIter_Next(lat_a_deg_iter);
    PyObject *lon_a_deg_next = PyIter_Next(lon_a_deg_iter);
    PyObject *lat_b_deg_next = PyIter_Next(lat_b_deg_iter);
    PyObject *lon_b_deg_next = PyIter_Next(lon_b_deg_iter);

    while (lat_a_deg_next != NULL && lon_a_deg_next != NULL && lat_b_deg_next != NULL && lon_b_deg_next != NULL)
    {
        if (PyFloat_Check(lat_a_deg_next))
        {
            lla_a.x = ((precision_type_t)PyFloat_AsDouble(lat_a_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lat_a_deg_next))
        {
            lla_a.x = ((precision_type_t)PyLong_AsDouble(lat_a_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lat_a_deg value must be a number");
            return NULL;
        }

        if (PyFloat_Check(lon_a_deg_next))
        {
            lla_a.y = ((precision_type_t)PyFloat_AsDouble(lon_a_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_a_deg_next))
        {
            lla_a.y = ((precision_type_t)PyLong_AsDouble(lon_a_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lon_a_deg value must be a number");
            return NULL;
        }

        if (PyFloat_Check(lat_b_deg_next))
        {
            lla_b.x = ((precision_type_t)PyFloat_AsDouble(lat_b_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lat_b_deg_next))
        {
            lla_b.x = ((precision_type_t)PyLong_AsDouble(lat_b_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lat_b_m value must be a number");
            return NULL;
        }

        if (PyFloat_Check(lon_b_deg_next))
        {
            lla_b.y = ((precision_type_t)PyFloat_AsDouble(lon_b_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_b_deg_next))
        {
            lla_b.y = ((precision_type_t)PyLong_AsDouble(lon_b_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lon_b_deg value must be a number");
            return NULL;
        }

        int error = vincenty_inverse(&lla_a, &lla_b, abs_tol, &range_m, &bearing_ab_rad, &bearing_ba_rad);
        if (error != KL_NO_ERROR)
        {
            PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
            return NULL;
        }

        PyObject *range_bearing_bearing =
            Py_BuildValue(return_tuple_format, range_m, bearing_ab_rad * RAD_TO_DEG, bearing_ba_rad * RAD_TO_DEG);
        if (range_bearing_bearing == NULL)
        {
            PyErr_SetString(PyExc_RuntimeError, "Error creating range-bearing-bearing return tuple");
            return NULL;
        }

        if (PyList_Append(output, range_bearing_bearing) != 0)
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "Failed to append range-bearing-bearing tuple");
            return NULL;
        }

        lat_a_deg_next = PyIter_Next(lat_a_deg_iter);
        lon_a_deg_next = PyIter_Next(lon_a_deg_iter);
        lat_b_deg_next = PyIter_Next(lat_b_deg_iter);
        lon_b_deg_next = PyIter_Next(lon_b_deg_iter);
    }

    return output;
}
