#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <kinematics-library/pytypes.h>
#include <kinematics-library/transforms.h>
#include <numpy/ndarrayobject.h>
#include <Python.h>
#include <stddef.h>
#include <structmember.h>

extern PyObject *aer_to_ned(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *ecef_to_lla(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *ecef_to_ned(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *lla_to_ecef(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *lla_to_ned(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *ned_to_aer(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *ned_to_ecef(PyObject *self, PyObject *args, PyObject *kwargs);
extern PyObject *ned_to_lla(PyObject *self, PyObject *args, PyObject *kwargs);
PyTypeObject    *KinematicVectorTypeRef = NULL;

PyMethodDef TransformsMethods[] = {
    {"aer_to_ned", (PyCFunction)aer_to_ned, METH_VARARGS, NULL},
    {"ecef_to_lla", (PyCFunction)ecef_to_lla, METH_VARARGS, NULL},
    {"ecef_to_ned", (PyCFunction)ecef_to_ned, METH_VARARGS, NULL},
    {"lla_to_ecef", (PyCFunction)lla_to_ecef, METH_VARARGS, NULL},
    {"lla_to_ned", (PyCFunction)lla_to_ned, METH_VARARGS, NULL},
    {"ned_to_aer", (PyCFunction)ned_to_aer, METH_VARARGS, NULL},
    {"ned_to_ecef", (PyCFunction)ned_to_ecef, METH_VARARGS, NULL},
    {"ned_to_lla", (PyCFunction)ned_to_lla, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}};

struct PyModuleDef transforms_module = {
    PyModuleDef_HEAD_INIT,
    "kinematics_library.transforms", /* name of module */
    NULL,                            /* module documentation, may be NULL */
    -1,                              /* size of per-interpreter state of the module,
                                        or -1 if the module keeps state in global variables. */
    TransformsMethods};

PyMODINIT_FUNC PyInit_transforms(void)
{
    PyObject *m = NULL;

    PyObject *kl = PyImport_ImportModule("kinematics_library");

    if (kl == NULL)
    {
        return NULL;
    }

    PyObject *kvRef = PyObject_GetAttrString(kl, "KinematicVector");
    Py_DECREF(kl);
    if (kvRef == NULL)
    {
        return NULL;
    }

    KinematicVectorTypeRef = (PyTypeObject *)kvRef;

    m = PyModule_Create(&transforms_module);
    if (m == NULL)
    {
        return NULL;
    }
    return m;
}
