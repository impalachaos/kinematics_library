#include <kinematics-library/errors.h>
#include <kinematics-library/helpers.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

static void KinematicVector_dealloc(kvector_py_t *self)
{
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *KinematicVector_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }
    kvector_py_t *self;
    self           = (kvector_py_t *)type->tp_alloc(type, 0);
    self->vector.x = 0;
    self->vector.y = 0;
    self->vector.z = 0;
    return (PyObject *)self;
}

static int KinematicVector_init(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
#ifdef MIN_MEMORY_FOOTPRINT
    static char *parseString = "|Off";
#else
    static char *parseString = "|Odd";
#endif

    PyObject        *object = NULL;
    precision_type_t y      = 0.0;
    precision_type_t z      = 0.0;

    if (!PyArg_ParseTuple(args, parseString, &object, &y, &z))
    {
        return -1;
    }

    if (object == NULL)
    {
        return 0;
    }

    if (PyFloat_Check(object))
    {
        precision_type_t x = (precision_type_t)PyFloat_AsDouble(object);
        self->vector.x     = x;
        self->vector.y     = y;
        self->vector.z     = z;

        return 0;
    }
    else if (PyLong_Check(object))
    {
        precision_type_t x = (precision_type_t)PyLong_AsDouble(object);
        self->vector.x     = x;
        self->vector.y     = y;
        self->vector.z     = z;

        return 0;
    }
    else if (PyArray_Check(object))
    {
        PyArrayObject *pyArray = (PyArrayObject *)object;
        if (!PyArray_CheckPrecisionType(pyArray))
        {
            PyErr_SetString(PyExc_RuntimeError, "NumPy array contains unsupported type");
            return -1;
        }
        PyArray_Descr *pyArrayDType = PyArray_DTYPE(pyArray);
        int            numDims      = PyArray_NDIM(pyArray);
        npy_intp      *shape        = PyArray_DIMS(pyArray);
        if (numDims == 1)
        {
            if (shape[0] == 3)
            {
                self->vector.x = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 0));
                self->vector.y = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 1));
                self->vector.z = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR1(pyArray, 2));
            }
            else
            {
                PyErr_SetString(PyExc_RuntimeError, "invalid NumPy array shape");
                return -1;
            }
        }
        else if (numDims == 2)
        {
            if (shape[0] == 1 && shape[1] == 3)
            {
                self->vector.x = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 0, 0));
                self->vector.y = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 0, 1));
                self->vector.z = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 0, 2));
            }
            else if (shape[0] == 3 && shape[1] == 1)
            {
                self->vector.x = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 0, 0));
                self->vector.y = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 1, 0));
                self->vector.z = PyArray_ToPrecisionType(pyArray, PyArray_GETPTR2(pyArray, 2, 0));
            }
            else
            {
                PyErr_SetString(PyExc_RuntimeError, "invalid NumPy array shape");
                return -1;
            }
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid NumPy array shape");
            return -1;
        }

        return 0;
    }

    PyObject *iterable = PyObject_GetIter(object);
    if (iterable == NULL)
    {
        return -1;
    }

    precision_type_t values[9];
    size_t           size = 0;
    PyObject        *next = PyIter_Next(iterable);
    while (next != NULL && size < 9)
    {
        if (PyFloat_Check(next))
        {
            values[size++] = (precision_type_t)PyFloat_AsDouble(next);
        }
        else if (PyLong_Check(next))
        {
            values[size++] = (precision_type_t)PyLong_AsDouble(next);
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "iterable contained unsupported type");
            return -1;
        }

        next = PyIter_Next(iterable);
    }

    if (size == 3)
    {
        self->vector.x = values[0];
        self->vector.y = values[1];
        self->vector.z = values[2];
    }
    else
    {
        PyErr_SetString(PyExc_RuntimeError, "iterable must have length of 3");
        return -1;
    }

    return 0;
}

extern PyObject *KinematicVector_add(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_add_inplace(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_div(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_div_inplace(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_mul(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_mul_inplace(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_sub(PyObject *lhs, PyObject *rhs);
extern PyObject *KinematicVector_sub_inplace(PyObject *lhs, PyObject *rhs);

static PyObject *KinematicVector_angle_between(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    kvector_py_t *object = NULL;
    if (!PyArg_ParseTuple(args, "O!", &KinematicVectorType, &object))
    {
        PyErr_SetString(PyExc_RuntimeError, "failed to parse arguments");
        return NULL;
    }

    if (object == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "no parameter provided");
        return NULL;
    }

    precision_type_t result;
    int              error = kvector_angle_between(&self->vector, &object->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_azimuth_angle(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = kvector_azimuth_angle(&self->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_cross(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    kvector_py_t *object = NULL;
    if (!PyArg_ParseTuple(args, "O!", &KinematicVectorType, &object))
    {
        PyErr_SetString(PyExc_RuntimeError, "failed to parse arguments");
        return NULL;
    }

    kvector_py_t *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
    int           error  = kvector_cross(&self->vector, &object->vector, &result->vector);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return (PyObject *)result;
}

static PyObject *KinematicVector_dot(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    kvector_py_t *object = NULL;
    if (!PyArg_ParseTuple(args, "O!", &KinematicVectorType, &object))
    {
        PyErr_SetString(PyExc_RuntimeError, "failed to parse arguments");
        return NULL;
    }

    if (object == NULL)
    {
        PyErr_SetString(PyExc_RuntimeError, "no parameter provided");
        return NULL;
    }

    precision_type_t result;
    int              error = kvector_dot(&self->vector, &object->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_elevation_angle(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = kvector_elevation_angle(&self->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_magnitude(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = kvector_magnitude(&self->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_polar_angle(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    precision_type_t result;
    int              error = kvector_polar_angle(&self->vector, &result);
    if (error != KL_NO_ERROR)
    {
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return PyFloat_FromDouble(result);
}

static PyObject *KinematicVector_richcompare(kvector_py_t *self, PyObject *other, int comparison)
{
    switch (comparison)
    {
        case Py_EQ: {
            if (PyObject_TypeCheck(other, &KinematicVectorType))
            {
                kvector_py_t *val = (kvector_py_t *)other;
                if (kvector_compare(&self->vector, &val->vector) == 0)
                {
                    Py_RETURN_TRUE;
                }
                Py_RETURN_FALSE;
            }
        }
        break;
        case Py_NE: {
            if (PyObject_TypeCheck(other, &KinematicVectorType))
            {
                kvector_py_t *val = (kvector_py_t *)other;
                if (kvector_compare(&self->vector, &val->vector) != 0)
                {
                    Py_RETURN_TRUE;
                }
                Py_RETURN_FALSE;
            }
        }
        break;
    }
    Py_RETURN_NOTIMPLEMENTED;
}

static PyObject *KinematicVector_str(kvector_py_t *self)
{
    char temp[100];
    snprintf(
        temp, sizeof(temp), "KinematicVector(x=%.6f, y=%.6f, z=%.6f)", self->vector.x, self->vector.y, self->vector.z);
    return PyUnicode_FromString(temp);
}

static PyObject *KinematicVector_unit(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    kvector_py_t *result = (kvector_py_t *)PyObject_CallObject((PyObject *)&KinematicVectorType, NULL);
    int           error  = kvector_unit(&self->vector, &result->vector);
    if (error != KL_NO_ERROR)
    {
        Py_XDECREF(result);
        PyErr_SetString(PyExc_RuntimeError, "null or invalid argument");
        return NULL;
    }

    return (PyObject *)result;
}

static PyObject *KinematicVector_zero(kvector_py_t *self, PyObject *args, PyObject *kwargs)
{
    self->vector.x = 0.0;
    self->vector.y = 0.0;
    self->vector.z = 0.0;

    Py_RETURN_NONE;
}

PyMemberDef KinematicVector_Members[] = {
    {"x",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(kvector_py_t, vector) + offsetof(kvector_t, x),
     0,
     "The X component of the kinematic vector."},
    {"y",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(kvector_py_t, vector) + offsetof(kvector_t, y),
     0,
     "The Y component of the kinematic vector."},
    {"z",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(kvector_py_t, vector) + offsetof(kvector_t, z),
     0,
     "The Z component of the kinematic vector."},
    {NULL}};

PyMethodDef KinematicVector_Methods[] = {
    {"angle_between", (PyCFunction)KinematicVector_angle_between, METH_VARARGS, NULL},
    {"azimuth_angle", (PyCFunction)KinematicVector_azimuth_angle, METH_VARARGS, NULL},
    {"cross", (PyCFunction)KinematicVector_cross, METH_VARARGS, NULL},
    {"dot", (PyCFunction)KinematicVector_dot, METH_VARARGS, NULL},
    {"elevation_angle", (PyCFunction)KinematicVector_elevation_angle, METH_VARARGS, NULL},
    {"magnitude", (PyCFunction)KinematicVector_magnitude, METH_VARARGS, NULL},
    {"polar_angle", (PyCFunction)KinematicVector_polar_angle, METH_VARARGS, NULL},
    {"unit", (PyCFunction)KinematicVector_unit, METH_VARARGS, NULL},
    {"zero", (PyCFunction)KinematicVector_zero, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}};

PyNumberMethods KinematicVector_NumberMethods = {
    (binaryfunc)KinematicVector_add,         // nb_add
    (binaryfunc)KinematicVector_sub,         // nb_subtract
    (binaryfunc)KinematicVector_mul,         // nb_multiply
    NULL,                                    // nb_remainder
    NULL,                                    // nb_divmod
    NULL,                                    // nb_power
    NULL,                                    // nb_negative
    NULL,                                    // nb_positive
    NULL,                                    // nb_absolute
    NULL,                                    // nb_bool
    NULL,                                    // nb_invert
    NULL,                                    // nb_lshift
    NULL,                                    // nb_rshift
    NULL,                                    // nb_and
    NULL,                                    // nb_xor
    NULL,                                    // nb_or
    NULL,                                    // nb_int
    NULL,                                    // nb_reserved
    NULL,                                    // nb_float
    (binaryfunc)KinematicVector_add_inplace, // nb_inplace_add
    (binaryfunc)KinematicVector_sub_inplace, // nb_inplace_subtract
    (binaryfunc)KinematicVector_mul_inplace, // nb_inplace_multiply
    NULL,                                    // nb_inplace_remainder
    NULL,                                    // nb_inplace_power
    NULL,                                    // nb_inplace_lshift
    NULL,                                    // nb_inplace_rshift
    NULL,                                    // nb_inplace_and
    NULL,                                    // nb_inplace_xor
    NULL,                                    // nb_inplace_or
    NULL,                                    // nb_floor_divide
    (binaryfunc)KinematicVector_div,         // nb_true_divide
    NULL,                                    // nb_inplace_floor_divide
    (binaryfunc)KinematicVector_div_inplace, // nb_inplace_true_divide
    NULL,                                    // nb_index
    NULL,                                    // nb_matrix_multiply
    NULL,                                    // nb_inplace_matrix_multiply
};

extern Py_ssize_t KinematicVector_mapping_size(kvector_py_t *self);
extern PyObject  *KinematicVector_subscript(kvector_py_t *self, PyObject *other);
extern PyObject  *KinematicVector_ass_subscript(kvector_py_t *self, PyObject *key, PyObject *value);

PyMappingMethods KinematicVectorMapping = {
    (lenfunc)KinematicVector_mapping_size,        // mp_length
    (binaryfunc)KinematicVector_subscript,        // mp_subscript
    (objobjargproc)KinematicVector_ass_subscript, // mp_ass_subscript
};

PyTypeObject KinematicVectorType = {
    PyVarObject_HEAD_INIT(NULL, 0) "KinematicVector", // tp_name
    sizeof(kvector_py_t),                             // tp_basicsize
    0,                                                // tp_itemsize
    (destructor)KinematicVector_dealloc,              // tp_dealloc
    0,                                                // tp_vectorcall_offset
    NULL,                                             // tp_getattr
    NULL,                                             // tp_setattr
    NULL,                                             // tp_as_async
    (reprfunc)KinematicVector_str,                    // tp_repr
    &KinematicVector_NumberMethods,                   // tp_as_number
    NULL,                                             // tp_as_sequence
    &KinematicVectorMapping,                          // tp_as_mapping
    NULL,                                             // tp_hash
    NULL,                                             // tp_call
    (reprfunc)KinematicVector_str,                    // tp_str
    NULL,                                             // tp_getattro
    NULL,                                             // tp_setattro
    NULL,                                             // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,         // tp_flags
    NULL,                                             // tp_doc
    NULL,                                             // tp_traverse
    NULL,                                             // tp_clear
    (richcmpfunc)KinematicVector_richcompare,         // tp_richcompare
    0,                                                // tp_weaklistoffset
    NULL,                                             // tp_iter
    NULL,                                             // tp_iternext
    KinematicVector_Methods,                          // tp_methods
    KinematicVector_Members,                          // tp_members
    NULL,                                             // tp_getset
    NULL,                                             // tp_base
    NULL,                                             // tp_dict
    NULL,                                             // tp_descr_get
    NULL,                                             // tp_descr_set
    0,                                                // tp_dictoffset
    (initproc)KinematicVector_init,                   // tp_init
    NULL,                                             // tp_alloc
    (newfunc)KinematicVector_new,                     // tp_new
    NULL,                                             // tp_free
    NULL,                                             // tp_is_gc
    NULL,                                             // tp_bases
    NULL,                                             // tp_mro
    NULL,                                             // tp_cache
    NULL,                                             // tp_subclasses
    NULL,                                             // tp_weaklist
    NULL,                                             // tp_del
    0,                                                // tp_version_tag
    NULL,                                             // tp_finalize
    NULL                                              // tp_vectorcall
};
