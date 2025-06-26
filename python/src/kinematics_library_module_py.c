#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/rotations.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

PyTypeObject *RotationSequenceType;

extern PyObject *vincenty_direct_py(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *vincenty_inverse_py(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *arr_add_docstring(PyObject *object, PyObject *const *args, Py_ssize_t len_args);

typedef struct enum_int_values
{
    char *name;
    int   value;
    char *docstr;
} enum_int_values;

enum_int_values RotationSequenceValues[] = {
    {"BODY_XYX", ROTATION_SEQ_BODY_XYX, "Body x-y-x rotation (i.e., roll-pitch-roll or 1-2-1)"},
    {"BODY_XYZ", ROTATION_SEQ_BODY_XYZ, NULL},
    {"BODY_XZX", ROTATION_SEQ_BODY_XZX, NULL},
    {"BODY_XZY", ROTATION_SEQ_BODY_XZY, NULL},
    {"BODY_YXY", ROTATION_SEQ_BODY_YXY, NULL},
    {"BODY_YXZ", ROTATION_SEQ_BODY_YXZ, NULL},
    {"BODY_YZX", ROTATION_SEQ_BODY_YZX, NULL},
    {"BODY_YZY", ROTATION_SEQ_BODY_YZY, NULL},
    {"BODY_ZXY", ROTATION_SEQ_BODY_ZXY, NULL},
    {"BODY_ZXZ", ROTATION_SEQ_BODY_ZXZ, NULL},
    {"BODY_ZYX", ROTATION_SEQ_BODY_ZYX, NULL},
    {"BODY_ZYZ", ROTATION_SEQ_BODY_ZYZ, NULL},
    {NULL, 0},
};

PyMethodDef KinematicLibraryMethods[] = {
    {"vincenty_direct", (PyCFunction)vincenty_direct_py, METH_VARARGS, NULL},
    {"vincenty_inverse", (PyCFunction)vincenty_inverse_py, METH_VARARGS, NULL},
    {"add_docstring", (PyCFunction)arr_add_docstring, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}};

struct PyModuleDef kinematics_library_module = {
    PyModuleDef_HEAD_INIT,
    "kinematics_library", /* name of module */
    NULL,                 /* module documentation, may be NULL */
    -1,                   /* size of per-interpreter state of the module,
                             or -1 if the module keeps state in global variables. */
    KinematicLibraryMethods};

PyMODINIT_FUNC PyInit_kinematics_library(void)
{
    PyObject *enum_module = PyImport_ImportModule("enum");
    if (!enum_module)
    {
        return NULL;
    }

    PyObject *enum_class = PyObject_GetAttrString(enum_module, "Enum");
    if (!enum_class)
    {
        Py_DECREF(enum_module);
        return NULL;
    }

    PyObject *class_name = PyUnicode_FromString("RotationSequence");
    if (!enum_class)
    {
        Py_DECREF(enum_module);
        Py_DECREF(enum_class);
        return NULL;
    }

    PyObject *members_dict = PyDict_New();
    if (!members_dict)
    {
        Py_DECREF(class_name);
        Py_DECREF(enum_class);
        Py_DECREF(enum_module);
        return NULL;
    }

    for (int i = 0; RotationSequenceValues[i].name != NULL; i++)
    {
        PyObject *val = PyLong_FromLong(RotationSequenceValues[i].value);
        PyDict_SetItemString(members_dict, RotationSequenceValues[i].name, val);
        Py_DECREF(val);
    }

    PyObject *args = Py_BuildValue("(OO)", class_name, members_dict);
    Py_DECREF(class_name);
    Py_DECREF(members_dict);

    PyObject *rotationSequenceEnum = PyObject_CallObject(enum_class, args);
    Py_DECREF(args);
    Py_DECREF(enum_class);
    Py_DECREF(enum_module);

    if (!rotationSequenceEnum)
    {
        return NULL;
    }

    for (int i = 0; RotationSequenceValues[i].name != NULL; i++)
    {
        if (RotationSequenceValues[i].docstr != NULL)
        {
            PyObject *attrObj = PyObject_GetAttrString(rotationSequenceEnum, RotationSequenceValues[i].name);
            if (attrObj != NULL)
            {
                PyObject *enumDoc = PyUnicode_FromString(RotationSequenceValues[i].docstr);
                if (PyObject_SetAttrString(attrObj, "__doc__", enumDoc) != 0)
                {
                    PyErr_Print();
                    Py_DECREF(attrObj);
                    Py_DECREF(enumDoc);
                    return NULL;
                }
                Py_DECREF(attrObj);
                Py_DECREF(enumDoc);
            }
        }
    }

    // if (PyObject_SetAttrString(rotationSequenceEnum, "__doc__", enumDoc) != 0)
    // {
    //     PyErr_Print();
    //     Py_DECREF(rotationSequenceEnum);
    //     Py_DECREF(enumDoc);
    //     return NULL;
    // }

    RotationSequenceType = (PyTypeObject *)rotationSequenceEnum;

    PyObject *m = NULL;
    if (PyType_Ready((PyTypeObject *)&KinematicVectorType) < 0)
    {
        return NULL;
    }

    if (PyType_Ready((PyTypeObject *)&DCMType) < 0)
    {
        return NULL;
    }

    if (PyType_Ready((PyTypeObject *)&QuaternionType) < 0)
    {
        return NULL;
    }

    m = PyModule_Create(&kinematics_library_module);
    if (m == NULL)
    {
        return NULL;
    }

    Py_INCREF(&KinematicVectorType);
    PyModule_AddObject(m, "KinematicVector", (PyObject *)&KinematicVectorType);
    Py_INCREF(&DCMType);
    PyModule_AddObject(m, "DCM", (PyObject *)&DCMType);
    Py_INCREF(&QuaternionType);
    PyModule_AddObject(m, "Quaternion", (PyObject *)&QuaternionType);
    PyModule_AddObject(m, "RotationSequence", rotationSequenceEnum);
    return m;
}
