#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

Py_ssize_t KinematicVector_mapping_size(kvector_py_t *self)
{
    return 3;
}

PyObject *KinematicVector_subscript(kvector_py_t *self, PyObject *key)
{
    if (!PyLong_Check(key))
    {
        PyErr_SetString(PyExc_RuntimeError, "index must be a number");
        return NULL;
    }

    size_t index = PyLong_AsSize_t(key);
    switch (index)
    {
        case 0:
            return PyFloat_FromDouble(self->vector.x);
        case 1:
            return PyFloat_FromDouble(self->vector.y);
        case 2:
            return PyFloat_FromDouble(self->vector.z);
    }
    PyErr_SetString(PyExc_IndexError, "index out-of-bounds");
    return NULL;
}

int KinematicVector_ass_subscript(kvector_py_t *self, PyObject *key, PyObject *value)
{
    if (!PyLong_Check(key))
    {
        PyErr_SetString(PyExc_RuntimeError, "index must be a number");
        return -1;
    }

    size_t index = PyLong_AsSize_t(key);
    if (index < 0 || index > 2)
    {
        PyErr_SetString(PyExc_IndexError, "index out-of-bounds");
        return -1;
    }

    precision_type_t val = 0.0;
    if (PyLong_Check(value))
    {
        val = (precision_type_t)PyLong_AsDouble(value);
    }
    else if (PyFloat_Check(value))
    {
        val = (precision_type_t)PyFloat_AsDouble(value);
    }
    else
    {
        PyErr_SetString(PyExc_TypeError, "value must be a number");
        return -1;
    }

    switch (index)
    {
        case 0:
            self->vector.x = val;
            break;
        case 1:
            self->vector.y = val;
            break;
        case 2:
            self->vector.z = val;
            break;
    }

    return 0;
}
