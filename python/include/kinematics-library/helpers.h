#pragma once

#include <kinematics-library/kinematic_vector.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>

#ifdef MIN_MEMORY_FOOTPRINT
typedef float precision_type_t;
#else
typedef double precision_type_t;
#endif

static inline precision_type_t PyArray_ToPrecisionType(PyArrayObject *obj, void *data);

static inline PyArrayObject *PyArray_AllocateOutput(PyArrayObject *in, PyArrayObject *out)
{
    if (out != NULL)
    {
        return out;
    }

    int       inDims  = PyArray_NDIM(in);
    npy_intp *inShape = PyArray_DIMS(in);

#ifdef MIN_MEMORY_FOOTPRINT
    return (PyArrayObject *)PyArray_SimpleNew(inDims, inShape, NPY_FLOAT);
#else
    return (PyArrayObject *)PyArray_SimpleNew(inDims, inShape, NPY_DOUBLE);
#endif
}

static inline PyArrayObject *PyArray_AllocateOutputWithDim(PyArrayObject *out, int inDims, const npy_intp *inShape)
{
    if (out != NULL)
    {
        return out;
    }

#ifdef MIN_MEMORY_FOOTPRINT
    return (PyArrayObject *)PyArray_SimpleNew(inDims, inShape, NPY_FLOAT);
#else
    return (PyArrayObject *)PyArray_SimpleNew(inDims, inShape, NPY_DOUBLE);
#endif
}

static inline int PyArray_CheckPrecisionType(PyArrayObject *obj)
{
    PyArray_Descr *pyArrayDType = PyArray_DTYPE(obj);

    switch (pyArrayDType->type_num)
    {
        case NPY_BYTE:
        case NPY_UBYTE:
        case NPY_SHORT:
        case NPY_USHORT:
        case NPY_INT:
        case NPY_UINT:
        case NPY_LONG:
        case NPY_ULONG:
        case NPY_LONGLONG:
        case NPY_ULONGLONG:
        case NPY_FLOAT:
        case NPY_DOUBLE:
            return 1;
    }

    return 0;
}

static inline int PyArray_CheckValidDims(PyArrayObject *in, PyObject *out)
{
    if (!PyArray_CheckPrecisionType(in))
    {
        PyErr_SetString(PyExc_RuntimeError, "NumPy array contains unsupported type");
        return 0;
    }

    PyArray_Descr *inDtype = PyArray_DTYPE(in);
    int            inDims  = PyArray_NDIM(in);
    npy_intp      *inShape = PyArray_DIMS(in);

    if (inDims == 1)
    {
        if (inShape[0] != 3)
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid input shape");
            return 0;
        }
    }
    else if (inDims == 2)
    {
        if (inShape[1] != 3)
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid input shape");
            return 0;
        }
    }
    else
    {
        PyErr_SetString(PyExc_RuntimeError, "invalid input shape");
        return 0;
    }

    if (out == NULL)
    {
        return 1;
    }

    if (!PyArray_Check(out))
    {
        PyErr_SetString(PyExc_RuntimeError, "out must be a NumPy ndarray");
        return 0;
    }

    PyArrayObject *outArray = (PyArrayObject *)out;

    PyArray_Descr *outDtype = PyArray_DTYPE(outArray);
    int            outDims  = PyArray_NDIM(outArray);
    npy_intp      *outShape = PyArray_DIMS(outArray);

#ifdef MIN_MEMORY_FOOTPRINT
    if (outDtype->type_num != NPY_FLOAT)
    {
        PyErr_SetString(PyExc_RuntimeError, "out must be float type");
        return 0;
    }
#else
    if (outDtype->type_num != NPY_DOUBLE)
    {
        PyErr_SetString(PyExc_RuntimeError, "out must be double type");
        return 0;
    }
#endif

    if (inDims != outDims)
    {
        PyErr_SetString(PyExc_RuntimeError, "out shape must match input shape");
        return 0;
    }

    for (int i = 0; i < outDims; i++)
    {
        if (inShape[i] != outShape[i])
        {
            PyErr_SetString(PyExc_RuntimeError, "out shape must match input shape");
            return 0;
        }
    }

    return 1;
}

static inline int PyArray_CheckValidRefDims(PyArrayObject *in, PyObject *ref)
{
    PyArray_Descr *inDtype = PyArray_DTYPE(in);
    int            inDims  = PyArray_NDIM(in);
    npy_intp      *inShape = PyArray_DIMS(in);

    if (ref == NULL)
    {
        return 1;
    }

    if (!PyArray_Check(ref))
    {
        PyErr_SetString(PyExc_RuntimeError, "out must be a NumPy ndarray");
        return 0;
    }

    PyArrayObject *refArray = (PyArrayObject *)ref;

    PyArray_Descr *refDtype = PyArray_DTYPE(refArray);
    int            refDims  = PyArray_NDIM(refArray);
    npy_intp      *refShape = PyArray_DIMS(refArray);

    if (inDims == 1)
    {
        if (refDims == 1)
        {
            if (refShape[0] != 3)
            {
                PyErr_SetString(PyExc_RuntimeError, "ref must match input shape");
                return 0;
            }
        }
        else if (refDims == 2)
        {
            if (refShape[0] != 1 || refShape[1] != 3)
            {
                PyErr_SetString(PyExc_RuntimeError, "ref must match input shape");
                return 0;
            }
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "ref must match input shape");
            return 0;
        }
    }
    else
    {
        if (refDims == 1)
        {
            if (refShape[0] != 3)
            {
                PyErr_SetString(PyExc_RuntimeError, "invalid ref shape");
                return 0;
            }
        }
        else if (refDims == 2)
        {
            if (refShape[1] != 3)
            {
                PyErr_SetString(PyExc_RuntimeError, "invalid ref shape");
                return 0;
            }
            else if (refShape[0] != 1 && refShape[0] != inShape[0])
            {
                PyErr_SetString(PyExc_RuntimeError, "invalid ref shape");
                return 0;
            }
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid ref shape");
            return 0;
        }
    }

    return 1;
}

static inline int PyArray_HasDims(PyArrayObject *in, int wantDims, npy_intp *wantShape)
{
    int       inDims  = PyArray_NDIM(in);
    npy_intp *inShape = PyArray_DIMS(in);

    if (inDims != wantDims)
    {
        return 0;
    }

    for (int i = 0; i < wantDims; i++)
    {
        if (wantShape[i] >= 0 && inShape[i] != wantShape[i])
        {
            return 0;
        }
    }

    return 1;
}

static inline int PyArray_HasSameDims(PyArrayObject *in, PyArrayObject *other)
{
    int       inDims  = PyArray_NDIM(in);
    npy_intp *inShape = PyArray_DIMS(in);

    int       otherDims  = PyArray_NDIM(other);
    npy_intp *otherShape = PyArray_DIMS(other);

    if (inDims != otherDims)
    {
        return 0;
    }

    for (int i = 0; i < otherDims; i++)
    {
        if (inShape[i] != otherShape[i])
        {
            return 0;
        }
    }

    return 1;
}

static inline void PyArray_SetRef(PyArrayObject *ref, kvector_t *refLla, int index)
{
    PyArray_Descr *refDtype = PyArray_DTYPE(ref);
    int            refDims  = PyArray_NDIM(ref);
    npy_intp      *refShape = PyArray_DIMS(ref);
    if (refDims == 1)
    {
        refLla->x = PyArray_ToPrecisionType(ref, PyArray_GETPTR1(ref, 0));
        refLla->y = PyArray_ToPrecisionType(ref, PyArray_GETPTR1(ref, 1));
        refLla->z = PyArray_ToPrecisionType(ref, PyArray_GETPTR1(ref, 2));
    }
    else if (refDims == 2)
    {
        if (refShape[0] == 1)
        {
            refLla->x = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, 0, 0));
            refLla->y = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, 0, 1));
            refLla->z = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, 0, 2));
        }
        else
        {
            refLla->x = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, index, 0));
            refLla->y = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, index, 1));
            refLla->z = PyArray_ToPrecisionType(ref, PyArray_GETPTR2(ref, index, 2));
        }
    }
}

static inline void PyArray_SetPrecisionType(PyArrayObject *obj, void *data, precision_type_t value)
{
    PyArray_Descr *pyArrayDType = PyArray_DTYPE(obj);

    switch (pyArrayDType->type_num)
    {
        case NPY_BYTE: {
            int8_t *val = (int8_t *)data;
            (*val)      = (int8_t)value;
        }
        break;
        case NPY_UBYTE: {
            uint8_t *val = (uint8_t *)data;
            (*val)       = (uint8_t)value;
        }
        break;
        case NPY_SHORT: {
            int16_t *val = (int16_t *)data;
            (*val)       = (int16_t)value;
        }
        break;
        case NPY_USHORT: {
            uint16_t *val = (uint16_t *)data;
            (*val)        = (uint16_t)value;
        }
        break;
        case NPY_INT: {
            int32_t *val = (int32_t *)data;
            (*val)       = (int32_t)value;
        }
        break;
        case NPY_UINT: {
            uint32_t *val = (uint32_t *)data;
            (*val)        = (uint32_t)value;
        }
        break;
        case NPY_LONG: {
            long *val = (long *)data;
            (*val)    = (long)value;
        }
        break;
        case NPY_ULONG: {
            unsigned long *val = (unsigned long *)data;
            (*val)             = (unsigned long)value;
        }
        break;
        case NPY_LONGLONG: {
            long long *val = (long long *)data;
            (*val)         = (long long)value;
        }
        break;
        case NPY_ULONGLONG: {
            unsigned long long *val = (unsigned long long *)data;
            (*val)                  = (unsigned long long)value;
        }
        break;
        case NPY_FLOAT: {
            float *val = (float *)data;
            (*val)     = (float)value;
        }
        break;
        case NPY_DOUBLE: {
            double *val = (double *)data;
            (*val)      = (double)value;
        }
        break;
    }
}

static inline precision_type_t PyArray_ToPrecisionType(PyArrayObject *obj, void *data)
{
    PyArray_Descr *pyArrayDType = PyArray_DTYPE(obj);

    switch (pyArrayDType->type_num)
    {
        case NPY_BYTE: {
            int8_t val = *((int8_t *)data);
            return (precision_type_t)val;
        }
        case NPY_UBYTE: {
            uint8_t val = *((uint8_t *)data);
            return (precision_type_t)val;
        }
        case NPY_SHORT: {
            int16_t val = *((int16_t *)data);
            return (precision_type_t)val;
        }
        case NPY_USHORT: {
            uint16_t val = *((uint16_t *)data);
            return (precision_type_t)val;
        }
        case NPY_INT: {
            int32_t val = *((int32_t *)data);
            return (precision_type_t)val;
        }
        case NPY_UINT: {
            uint32_t val = *((uint32_t *)data);
            return (precision_type_t)val;
        }
        case NPY_LONG: {
            long val = *((long *)data);
            return (precision_type_t)val;
        }
        case NPY_ULONG: {
            unsigned long val = *((unsigned long *)data);
            return (precision_type_t)val;
        }
        case NPY_LONGLONG: {
            long long val = *((long long *)data);
            return (precision_type_t)val;
        }
        case NPY_ULONGLONG: {
            unsigned long long val = *((unsigned long long *)data);
            return (precision_type_t)val;
        }
        case NPY_FLOAT: {
            float val = *((float *)data);
            return (precision_type_t)val;
        }
        case NPY_DOUBLE: {
            double val = *((double *)data);
            return (precision_type_t)val;
        }
    }

    return 0;
}
