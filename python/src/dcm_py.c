#include <kinematics-library/errors.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

static void DCM_dealloc(dcm_py_t *self)
{
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *DCM_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    if (PyArray_API == NULL)
    {
        import_array();
    }
    dcm_py_t *self;
    self          = (dcm_py_t *)type->tp_alloc(type, 0);
    self->dcm.e00 = 1;
    self->dcm.e01 = 0;
    self->dcm.e02 = 0;
    self->dcm.e10 = 0;
    self->dcm.e11 = 1;
    self->dcm.e12 = 0;
    self->dcm.e20 = 0;
    self->dcm.e21 = 0;
    self->dcm.e22 = 1;
    return (PyObject *)self;
}

static int DCM_init(dcm_py_t *self, PyObject *args, PyObject *kwargs)
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
        dcm_py_t *val = (dcm_py_t *)object;
        self->dcm.e00 = val->dcm.e00;
        self->dcm.e01 = val->dcm.e01;
        self->dcm.e02 = val->dcm.e02;
        self->dcm.e10 = val->dcm.e10;
        self->dcm.e11 = val->dcm.e11;
        self->dcm.e12 = val->dcm.e12;
        self->dcm.e20 = val->dcm.e20;
        self->dcm.e21 = val->dcm.e21;
        self->dcm.e22 = val->dcm.e22;

        return 0;
    }
    else if (PyObject_TypeCheck(object, &QuaternionType))
    {
        quaternion_py_t *val   = (quaternion_py_t *)object;
        int              error = dcm_init_quaternion(&self->dcm, &val->quaternion);
        if (error != KL_NO_ERROR)
        {
            PyErr_SetString(PyExc_RuntimeError, "invalid quaternion");
            return -1;
        }
    }
    else if (PyFloat_Check(object))
    {
        precision_type_t a1_rad = (precision_type_t)PyFloat_AsDouble(object);
        int              error  = dcm_init(&self->dcm, a1_rad, a2_rad, a3_rad, sequence);
        if (error != KL_NO_ERROR)
        {
            PyErr_Format(PyExc_RuntimeError, "failed to initialize dcm: %d", error);
            return -1;
        }
    }
    else if (PyLong_Check(object))
    {
        precision_type_t a1_rad = (precision_type_t)PyLong_AsDouble(object);
        int              error  = dcm_init(&self->dcm, a1_rad, a2_rad, a3_rad, sequence);
        if (error != KL_NO_ERROR)
        {
            PyErr_Format(PyExc_RuntimeError, "failed to initialize dcm: %d", error);
            return -1;
        }
    }

    return 0;
}

extern PyObject *DCM_mul(PyObject *lhs, PyObject *rhs);
extern PyObject *DCM_mul_inplace(PyObject *lhs, PyObject *rhs);
extern PyObject *DCM_rotate(dcm_py_t *self, PyObject *args, PyObject *kwargs);
extern PyObject *DCM_transform(dcm_py_t *self, PyObject *args, PyObject *kwargs);
extern PyObject *DCM_transpose(dcm_py_t *self, PyObject *args, PyObject *kwargs);

static PyObject *DCM_richcompare(dcm_py_t *self, PyObject *other, int comparison)
{
    switch (comparison)
    {
        case Py_EQ: {
            if (PyObject_TypeCheck(other, &DCMType))
            {
                dcm_py_t *val = (dcm_py_t *)other;
                if (dcm_compare(&self->dcm, &val->dcm) == 0)
                {
                    Py_RETURN_TRUE;
                }
                Py_RETURN_FALSE;
            }
        }
        break;
        case Py_NE: {
            if (PyObject_TypeCheck(other, &DCMType))
            {
                dcm_py_t *val = (dcm_py_t *)other;
                if (dcm_compare(&self->dcm, &val->dcm) != 0)
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

static PyObject *DCM_str(dcm_py_t *self)
{
    char temp[256];
    snprintf(
        temp,
        sizeof(temp),
        "DCM[[%.6f, %.6f, %.6f],\n    [%.6f, %.6f, %.6f],\n    [%.6f, %.6f, %.6f]]",
        self->dcm.e00,
        self->dcm.e01,
        self->dcm.e02,
        self->dcm.e10,
        self->dcm.e11,
        self->dcm.e12,
        self->dcm.e20,
        self->dcm.e21,
        self->dcm.e22);
    return PyUnicode_FromString(temp);
}

PyMemberDef DCM_Members[] = {
    {"e00",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e00),
     READONLY,
     NULL},
    {"e01",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e01),
     READONLY,
     NULL},
    {"e02",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e02),
     READONLY,
     NULL},
    {"e10",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e10),
     READONLY,
     NULL},
    {"e11",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e11),
     READONLY,
     NULL},
    {"e12",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e12),
     READONLY,
     NULL},
    {"e20",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e20),
     READONLY,
     NULL},
    {"e21",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e21),
     READONLY,
     NULL},
    {"e22",
#ifdef MIN_MEMORY_FOOTPRINT
     T_FLOAT,
#else
     T_DOUBLE,
#endif
     offsetof(dcm_py_t, dcm) + offsetof(dcm_t, e22),
     READONLY,
     NULL},
    {NULL}};

PyMethodDef DCM_Methods[] = {
    {"rotate", (PyCFunction)DCM_rotate, METH_VARARGS, NULL},
    {"transform", (PyCFunction)DCM_transform, METH_VARARGS, NULL},
    {"transpose", (PyCFunction)DCM_transpose, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}};

PyNumberMethods DCM_NumberMethods = {
    NULL,                        // nb_add
    NULL,                        // nb_subtract
    (binaryfunc)DCM_mul,         // nb_multiply
    NULL,                        // nb_remainder
    NULL,                        // nb_divmod
    NULL,                        // nb_power
    NULL,                        // nb_negative
    NULL,                        // nb_positive
    NULL,                        // nb_absolute
    NULL,                        // nb_bool
    NULL,                        // nb_invert
    NULL,                        // nb_lshift
    NULL,                        // nb_rshift
    NULL,                        // nb_and
    NULL,                        // nb_xor
    NULL,                        // nb_or
    NULL,                        // nb_int
    NULL,                        // nb_reserved
    NULL,                        // nb_float
    NULL,                        // nb_inplace_add
    NULL,                        // nb_inplace_subtract
    (binaryfunc)DCM_mul_inplace, // nb_inplace_multiply
    NULL,                        // nb_inplace_remainder
    NULL,                        // nb_inplace_power
    NULL,                        // nb_inplace_lshift
    NULL,                        // nb_inplace_rshift
    NULL,                        // nb_inplace_and
    NULL,                        // nb_inplace_xor
    NULL,                        // nb_inplace_or
    NULL,                        // nb_floor_divide
    NULL,                        // nb_true_divide
    NULL,                        // nb_inplace_floor_divide
    NULL,                        // nb_inplace_true_divide
    NULL,                        // nb_index
    NULL,                        // nb_matrix_multiply
    NULL,                        // nb_inplace_matrix_multiply
};

PyTypeObject DCMType = {
    PyVarObject_HEAD_INIT(NULL, 0) "DCM",     // tp_name
    sizeof(dcm_py_t),                         // tp_basicsize
    0,                                        // tp_itemsize
    (destructor)DCM_dealloc,                  // tp_dealloc
    0,                                        // tp_vectorcall_offset
    NULL,                                     // tp_getattr
    NULL,                                     // tp_setattr
    NULL,                                     // tp_as_async
    (reprfunc)DCM_str,                        // tp_repr
    &DCM_NumberMethods,                       // tp_as_number
    NULL,                                     // tp_as_sequence
    NULL,                                     // tp_as_mapping
    NULL,                                     // tp_hash
    NULL,                                     // tp_call
    (reprfunc)DCM_str,                        // tp_str
    NULL,                                     // tp_getattro
    NULL,                                     // tp_setattro
    NULL,                                     // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
    NULL,                                     // tp_doc
    NULL,                                     // tp_traverse
    NULL,                                     // tp_clear
    (richcmpfunc)DCM_richcompare,             // tp_richcompare
    0,                                        // tp_weaklistoffset
    NULL,                                     // tp_iter
    NULL,                                     // tp_iternext
    DCM_Methods,                              // tp_methods
    DCM_Members,                              // tp_members
    NULL,                                     // tp_getset
    NULL,                                     // tp_base
    NULL,                                     // tp_dict
    NULL,                                     // tp_descr_get
    NULL,                                     // tp_descr_set
    0,                                        // tp_dictoffset
    (initproc)DCM_init,                       // tp_init
    NULL,                                     // tp_alloc
    (newfunc)DCM_new,                         // tp_new
    NULL,                                     // tp_free
    NULL,                                     // tp_is_gc
    NULL,                                     // tp_bases
    NULL,                                     // tp_mro
    NULL,                                     // tp_cache
    NULL,                                     // tp_subclasses
    NULL,                                     // tp_weaklist
    NULL,                                     // tp_del
    0,                                        // tp_version_tag
    NULL,                                     // tp_finalize
    NULL                                      // tp_vectorcall
};
