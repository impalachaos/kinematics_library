#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

static void Quaternion_dealloc(quaternion_py_t *self)
{
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *Quaternion_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }
    quaternion_py_t *self;
    self               = (quaternion_py_t *)type->tp_alloc(type, 0);
    self->quaternion.a = 1;
    self->quaternion.b = 0;
    self->quaternion.c = 0;
    self->quaternion.d = 0;
    return (PyObject *)self;
}

static int Quaternion_init(quaternion_py_t *self, PyObject *args, PyObject *kwargs)
{
    PyObject        *object    = NULL;
    precision_type_t a2_rad    = 0.0;
    precision_type_t a3_rad    = 0.0;
    PyObject        *seqObject = NULL;

#ifdef MIN_MEMORY_FOOTPRINT
    static char *parseString = "|OffO";
#else
    static char *parseString = "|OddO";
#endif

    if (!PyArg_ParseTuple(args, parseString, &object, &a2_rad, &a3_rad, &seqObject))
    {
        return -1;
    }

    if (object == NULL)
    {
        return 0;
    }

    int sequence = -1;
    if (seqObject != NULL)
    {
        if (PyLong_Check(seqObject))
        {
            sequence = (int)PyLong_AsLong(seqObject);
        }
        else if (PyObject_TypeCheck(seqObject, RotationSequenceType))
        {
            PyObject *seqVal = PyObject_GetAttrString(seqObject, "value");
            if (seqVal == NULL)
            {
                PyErr_SetString(PyExc_RuntimeError, "failed to get RotationSequence value");
                return -1;
            }

            if (PyLong_Check(seqVal))
            {
                sequence = (int)PyLong_AsLong(seqVal);
            }
            else
            {
                PyErr_SetString(PyExc_RuntimeError, "RotationSequence value was not an integer");
                return -1;
            }
        }
        else
        {
            PyErr_SetString(PyExc_RuntimeError, "expected seq to be RotationSequence or integer");
            return -1;
        }
    }

    if (PyObject_TypeCheck(object, &DCMType))
    {
        dcm_py_t *val   = (dcm_py_t *)object;
        int       error = quaternion_init_dcm(&self->quaternion, &val->dcm);
        if (error != KL_NO_ERROR)
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid DCM");
            return -1;
        }
    }
    else if (PyObject_TypeCheck(object, &QuaternionType))
    {
        quaternion_py_t *val = (quaternion_py_t *)object;
        self->quaternion.a   = val->quaternion.a;
        self->quaternion.b   = val->quaternion.b;
        self->quaternion.c   = val->quaternion.c;
        self->quaternion.d   = val->quaternion.d;
    }
    else if (PyFloat_Check(object))
    {
        precision_type_t a1_rad = (precision_type_t)PyFloat_AsDouble(object);
        int              error  = quaternion_init(&self->quaternion, a1_rad, a2_rad, a3_rad, sequence);
        if (error != KL_NO_ERROR)
        {
            PyErr_Format(PyExc_RuntimeError, "failed to initialize quaternion: %d", error);
            return -1;
        }
    }
    else if (PyLong_Check(object))
    {
        precision_type_t a1_rad = (precision_type_t)PyLong_AsDouble(object);
        int              error  = quaternion_init(&self->quaternion, a1_rad, a2_rad, a3_rad, sequence);
        if (error != KL_NO_ERROR)
        {
            PyErr_Format(PyExc_RuntimeError, "failed to initialize quaternion: %d", error);
            return -1;
        }
    }

    return 0;
}

PyObject *Quaternion_angle(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_axis(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_conjugate(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_inverse(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_mul(PyObject *lhs, PyObject *rhs);
PyObject *Quaternion_mul_inplace(PyObject *lhs, PyObject *rhs);
PyObject *Quaternion_norm(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_rotate(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_square(quaternion_py_t *self, PyObject *args, PyObject *kwargs);
PyObject *Quaternion_transform(quaternion_py_t *self, PyObject *args, PyObject *kwargs);

static PyObject *Quaternion_richcompare(quaternion_py_t *self, PyObject *other, int comparison)
{
    switch (comparison)
    {
        case Py_EQ: {
            if (PyObject_TypeCheck(other, &QuaternionType))
            {
                quaternion_py_t *val = (quaternion_py_t *)other;
                if (quaternion_compare(&self->quaternion, &val->quaternion) == 0)
                {
                    Py_RETURN_TRUE;
                }
                Py_RETURN_FALSE;
            }
        }
        break;
        case Py_NE: {
            if (PyObject_TypeCheck(other, &QuaternionType))
            {
                quaternion_py_t *val = (quaternion_py_t *)other;
                if (quaternion_compare(&self->quaternion, &val->quaternion) != 0)
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

static PyObject *Quaternion_str(quaternion_py_t *self)
{
    char temp[256];
    snprintf(
        temp,
        sizeof(temp),
        "Quaternion(a=%.6f, b=%.6f, c=%.6f, d=%.6f)",
        self->quaternion.a,
        self->quaternion.b,
        self->quaternion.c,
        self->quaternion.d);
    return PyUnicode_FromString(temp);
}

PyMemberDef Quaternion_Members[] = {
    {"a",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(quaternion_py_t, quaternion) + offsetof(quaternion_t, a),
     READONLY,
     NULL},
    {"b",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(quaternion_py_t, quaternion) + offsetof(quaternion_t, b),
     READONLY,
     NULL},
    {"c",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(quaternion_py_t, quaternion) + offsetof(quaternion_t, c),
     READONLY,
     NULL},
    {"d",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(quaternion_py_t, quaternion) + offsetof(quaternion_t, d),
     READONLY,
     NULL},
    {NULL}};

PyMethodDef Quaternion_Methods[] = {
    {"angle", (PyCFunction)Quaternion_angle, METH_VARARGS, NULL},
    {"axis", (PyCFunction)Quaternion_axis, METH_VARARGS, NULL},
    {"conjugate", (PyCFunction)Quaternion_conjugate, METH_VARARGS, NULL},
    {"inverse", (PyCFunction)Quaternion_inverse, METH_VARARGS, NULL},
    {"norm", (PyCFunction)Quaternion_norm, METH_VARARGS, NULL},
    {"rotate", (PyCFunction)Quaternion_rotate, METH_VARARGS, NULL},
    {"square", (PyCFunction)Quaternion_square, METH_VARARGS, NULL},
    {"transform", (PyCFunction)Quaternion_transform, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}};

PyNumberMethods Quaternion_NumberMethods = {
    NULL,                               // nb_add
    NULL,                               // nb_subtract
    (binaryfunc)Quaternion_mul,         // nb_multiply
    NULL,                               // nb_remainder
    NULL,                               // nb_divmod
    NULL,                               // nb_power
    NULL,                               // nb_negative
    NULL,                               // nb_positive
    NULL,                               // nb_absolute
    NULL,                               // nb_bool
    NULL,                               // nb_invert
    NULL,                               // nb_lshift
    NULL,                               // nb_rshift
    NULL,                               // nb_and
    NULL,                               // nb_xor
    NULL,                               // nb_or
    NULL,                               // nb_int
    NULL,                               // nb_reserved
    NULL,                               // nb_float
    NULL,                               // nb_inplace_add
    NULL,                               // nb_inplace_subtract
    (binaryfunc)Quaternion_mul_inplace, // nb_inplace_multiply
    NULL,                               // nb_inplace_remainder
    NULL,                               // nb_inplace_power
    NULL,                               // nb_inplace_lshift
    NULL,                               // nb_inplace_rshift
    NULL,                               // nb_inplace_and
    NULL,                               // nb_inplace_xor
    NULL,                               // nb_inplace_or
    NULL,                               // nb_floor_divide
    NULL,                               // nb_true_divide
    NULL,                               // nb_inplace_floor_divide
    NULL,                               // nb_inplace_true_divide
    NULL,                               // nb_index
    NULL,                               // nb_matrix_multiply
    NULL,                               // nb_inplace_matrix_multiply
};

PyTypeObject QuaternionType = {
    PyVarObject_HEAD_INIT(NULL, 0) "Quaternion", // tp_name
    sizeof(quaternion_py_t),                     // tp_basicsize
    0,                                           // tp_itemsize
    (destructor)Quaternion_dealloc,              // tp_dealloc
    0,                                           // tp_vectorcall_offset
    NULL,                                        // tp_getattr
    NULL,                                        // tp_setattr
    NULL,                                        // tp_as_async
    (reprfunc)Quaternion_str,                    // tp_repr
    &Quaternion_NumberMethods,                   // tp_as_number
    NULL,                                        // tp_as_sequence
    NULL,                                        // tp_as_mapping
    NULL,                                        // tp_hash
    NULL,                                        // tp_call
    (reprfunc)Quaternion_str,                    // tp_str
    NULL,                                        // tp_getattro
    NULL,                                        // tp_setattro
    NULL,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,    // tp_flags
    NULL,                                        // tp_doc
    NULL,                                        // tp_traverse
    NULL,                                        // tp_clear
    (richcmpfunc)Quaternion_richcompare,         // tp_richcompare
    0,                                           // tp_weaklistoffset
    NULL,                                        // tp_iter
    NULL,                                        // tp_iternext
    Quaternion_Methods,                          // tp_methods
    Quaternion_Members,                          // tp_members
    NULL,                                        // tp_getset
    NULL,                                        // tp_base
    NULL,                                        // tp_dict
    NULL,                                        // tp_descr_get
    NULL,                                        // tp_descr_set
    0,                                           // tp_dictoffset
    (initproc)Quaternion_init,                   // tp_init
    NULL,                                        // tp_alloc
    (newfunc)Quaternion_new,                     // tp_new
    NULL,                                        // tp_free
    NULL,                                        // tp_is_gc
    NULL,                                        // tp_bases
    NULL,                                        // tp_mro
    NULL,                                        // tp_cache
    NULL,                                        // tp_subclasses
    NULL,                                        // tp_weaklist
    NULL,                                        // tp_del
    0,                                           // tp_version_tag
    NULL,                                        // tp_finalize
    NULL                                         // tp_vectorcall
};
