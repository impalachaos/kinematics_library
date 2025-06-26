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

PyObject *vincenty_direct_py(PyObject *self, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }

    PyObject        *lat_deg_obj     = NULL;
    PyObject        *lon_deg_obj     = NULL;
    PyObject        *range_m_obj     = NULL;
    PyObject        *bearing_deg_obj = NULL;
    precision_type_t abs_tol         = 0.0;
    PyObject        *output          = NULL;

#ifdef MIN_MEMORY_FOOTPRINT
    static char *parseString         = "OOOOf";
    static char *return_tuple_format = "ff";
#else
    static char *parseString         = "OOOOd";
    static char *return_tuple_format = "dd";
#endif

    if (!PyArg_ParseTuple(args, parseString, &lat_deg_obj, &lon_deg_obj, &range_m_obj, &bearing_deg_obj, &abs_tol))
    {
        return NULL;
    }

    kvector_t        lla_a       = {0.0, 0.0, 0.0};
    kvector_t        lla_b       = {0.0, 0.0, 0.0};
    precision_type_t range_m     = 0.0;
    precision_type_t bearing_deg = 0.0;

    if (PyArray_Check(lat_deg_obj))
    {
        if (!PyArray_Check(lon_deg_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_deg must be an array_like");
            return NULL;
        }
        if (!PyArray_Check(range_m_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "range_m must be an array_like");
            return NULL;
        }
        if (!PyArray_Check(bearing_deg_obj))
        {
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg must be an array_like");
            return NULL;
        }

        npy_intp wantDims[] = {-1};

        PyArrayObject *lat_deg_array     = (PyArrayObject *)lat_deg_obj;
        PyArrayObject *lon_deg_array     = (PyArrayObject *)lon_deg_obj;
        PyArrayObject *range_m_array     = (PyArrayObject *)range_m_obj;
        PyArrayObject *bearing_deg_array = (PyArrayObject *)bearing_deg_obj;

        if (!PyArray_HasDims(lat_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lat_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(lon_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(range_m_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "range_m has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasDims(bearing_deg_array, 1, &wantDims[0]))
        {
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg has an invalid shape");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_deg_array, lon_deg_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_deg_array, range_m_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }
        if (!PyArray_HasSameDims(lat_deg_array, bearing_deg_array))
        {
            PyErr_SetString(PyExc_RuntimeError, "input array lengths do not match");
            return NULL;
        }

        int       numDims = PyArray_NDIM(lat_deg_array);
        npy_intp *shape   = PyArray_DIMS(lat_deg_array);

        if (numDims == 1)
        {
            npy_intp       outShape[] = {shape[0], 2};
            PyArrayObject *outArray   = PyArray_AllocateOutputWithDim((PyArrayObject *)output, 2, &outShape[0]);

            for (int i = 0; i < shape[0]; i++)
            {
                lla_a.x     = PyArray_ToPrecisionType(lat_deg_array, PyArray_GETPTR1(lat_deg_array, i)) * DEG_TO_RAD;
                lla_a.y     = PyArray_ToPrecisionType(lon_deg_array, PyArray_GETPTR1(lon_deg_array, i)) * DEG_TO_RAD;
                range_m     = PyArray_ToPrecisionType(range_m_array, PyArray_GETPTR1(range_m_array, i));
                bearing_deg = PyArray_ToPrecisionType(bearing_deg_array, PyArray_GETPTR1(bearing_deg_array, i));

                int error = vincenty_direct(&lla_a, range_m, bearing_deg * DEG_TO_RAD, abs_tol, &lla_b);
                if (error != KL_NO_ERROR)
                {
                    if (output == NULL)
                    {
                        Py_XDECREF(outArray);
                    }
                    PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
                    return NULL;
                }

                PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 0), lla_b.x * RAD_TO_DEG);
                PyArray_SetPrecisionType(outArray, PyArray_GETPTR2(outArray, i, 1), lla_b.y * RAD_TO_DEG);
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
    else if (PyFloat_Check(lat_deg_obj) || PyLong_Check(lat_deg_obj))
    {
        if (PyFloat_Check(lat_deg_obj))
        {
            lla_a.x = ((precision_type_t)PyFloat_AsDouble(lat_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            lla_a.x = ((precision_type_t)PyLong_AsDouble(lat_deg_obj)) * DEG_TO_RAD;
        }

        if (PyFloat_Check(lon_deg_obj))
        {
            lla_a.y = ((precision_type_t)PyFloat_AsDouble(lon_deg_obj)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_deg_obj))
        {
            lla_a.y = ((precision_type_t)PyLong_AsDouble(lon_deg_obj)) * DEG_TO_RAD;
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "lon_deg must be a number");
            return NULL;
        }

        if (PyFloat_Check(range_m_obj))
        {
            range_m = ((precision_type_t)PyFloat_AsDouble(range_m_obj));
        }
        else if (PyLong_Check(range_m_obj))
        {
            range_m = ((precision_type_t)PyLong_AsDouble(range_m_obj));
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "range_m must be a number");
            return NULL;
        }

        if (PyFloat_Check(bearing_deg_obj))
        {
            bearing_deg = ((precision_type_t)PyFloat_AsDouble(bearing_deg_obj));
        }
        else if (PyLong_Check(bearing_deg_obj))
        {
            bearing_deg = ((precision_type_t)PyLong_AsDouble(bearing_deg_obj));
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg must be a number");
            return NULL;
        }

        int error = vincenty_direct(&lla_a, range_m, bearing_deg * DEG_TO_RAD, abs_tol, &lla_b);
        if (error != KL_NO_ERROR)
        {
            PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
            return NULL;
        }

        PyObject *lat_lon = Py_BuildValue(return_tuple_format, lla_b.x * RAD_TO_DEG, lla_b.y * RAD_TO_DEG);
        if (lat_lon == NULL)
        {
            PyErr_SetString(PyExc_RuntimeError, "Error creating lat-lon return tuple");
            return NULL;
        }
        return lat_lon;
    }

    PyObject *lat_deg_iter = PyObject_GetIter(lat_deg_obj);
    if (lat_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *lon_deg_iter = PyObject_GetIter(lon_deg_obj);
    if (lon_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *range_m_iter = PyObject_GetIter(range_m_obj);
    if (range_m_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    PyObject *bearing_deg_iter = PyObject_GetIter(bearing_deg_obj);
    if (bearing_deg_iter == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "lat_deg is not iterable");
        return NULL;
    }

    output = PyList_New(0);

    PyObject *lat_deg_next     = PyIter_Next(lat_deg_iter);
    PyObject *lon_deg_next     = PyIter_Next(lon_deg_iter);
    PyObject *range_m_next     = PyIter_Next(range_m_iter);
    PyObject *bearing_deg_next = PyIter_Next(bearing_deg_iter);

    while (lat_deg_next != NULL && lon_deg_next != NULL && range_m_next != NULL && bearing_deg_next != NULL)
    {
        if (PyFloat_Check(lat_deg_next))
        {
            lla_a.x = ((precision_type_t)PyFloat_AsDouble(lat_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lat_deg_next))
        {
            lla_a.x = ((precision_type_t)PyLong_AsDouble(lat_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lat_deg value must be a number");
            return NULL;
        }

        if (PyFloat_Check(lon_deg_next))
        {
            lla_a.y = ((precision_type_t)PyFloat_AsDouble(lon_deg_next)) * DEG_TO_RAD;
        }
        else if (PyLong_Check(lon_deg_next))
        {
            lla_a.y = ((precision_type_t)PyLong_AsDouble(lon_deg_next)) * DEG_TO_RAD;
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "lon_deg value must be a number");
            return NULL;
        }

        if (PyFloat_Check(range_m_next))
        {
            range_m = ((precision_type_t)PyFloat_AsDouble(range_m_next));
        }
        else if (PyLong_Check(range_m_next))
        {
            range_m = ((precision_type_t)PyLong_AsDouble(range_m_next));
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "range_m value must be a number");
            return NULL;
        }

        if (PyFloat_Check(bearing_deg_next))
        {
            bearing_deg = ((precision_type_t)PyFloat_AsDouble(bearing_deg_next));
        }
        else if (PyLong_Check(bearing_deg_next))
        {
            bearing_deg = ((precision_type_t)PyLong_AsDouble(bearing_deg_next));
        }
        else
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "bearing_deg value must be a number");
            return NULL;
        }

        int error = vincenty_direct(&lla_a, range_m, bearing_deg * DEG_TO_RAD, abs_tol, &lla_b);
        if (error != KL_NO_ERROR)
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "Null or invalid argument");
            return NULL;
        }

        PyObject *lat_lon = Py_BuildValue(return_tuple_format, lla_b.x * RAD_TO_DEG, lla_b.y * RAD_TO_DEG);
        if (lat_lon == NULL)
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "Error creating lat-lon return tuple");
            return NULL;
        }

        if (PyList_Append(output, lat_lon) != 0)
        {
            Py_XDECREF(output);
            PyErr_SetString(PyExc_RuntimeError, "Failed to append lat-lon tuple");
            return NULL;
        }

        lat_deg_next     = PyIter_Next(lat_deg_iter);
        lon_deg_next     = PyIter_Next(lon_deg_iter);
        range_m_next     = PyIter_Next(range_m_iter);
        bearing_deg_next = PyIter_Next(bearing_deg_iter);
    }

    return output;
}
