#include <Python.h>
#include <structmember.h>

/*
 * Can only be called if doc is currently NULL.
 * Borrowed from https://github.com/numpy/numpy/_core/src/multiarray/compiled_base.c
 */
PyObject *arr_add_docstring(PyObject *object, PyObject *const *args, Py_ssize_t len_args)
{
    PyObject    *obj;
    PyObject    *str;
    const char  *docstr;
    static char *msg = "already has a different docstring";

    if (!PyArg_ParseTuple(args, "OO", &obj, &str))
    {
        return NULL;
    }
    if (!PyUnicode_Check(str))
    {
        PyErr_SetString(PyExc_TypeError, "argument docstring of add_docstring should be a str");
        return NULL;
    }

    docstr = PyUnicode_AsUTF8(str);
    if (docstr == NULL)
    {
        return NULL;
    }

#define _ADDDOC(doc, name)                                                                                             \
    if (!(doc))                                                                                                        \
    {                                                                                                                  \
        doc = docstr;                                                                                                  \
        Py_INCREF(str); /* hold on to string (leaks reference) */                                                      \
    }                                                                                                                  \
    else if (strcmp(doc, docstr) != 0)                                                                                 \
    {                                                                                                                  \
        PyErr_Format(PyExc_RuntimeError, "%s method %s", name, msg);                                                   \
        return NULL;                                                                                                   \
    }

    if (Py_TYPE(obj) == &PyCFunction_Type)
    {
        PyCFunctionObject *new = (PyCFunctionObject *)obj;
        _ADDDOC(new->m_ml->ml_doc, new->m_ml->ml_name);
    }
    else if (PyObject_TypeCheck(obj, &PyType_Type))
    {
        /*
         * We add it to both `tp_doc` and `__doc__` here.  Note that in theory
         * `tp_doc` extracts the signature line, but we currently do not use
         * it.  It may make sense to only add it as `__doc__` and
         * `__text_signature__` to the dict in the future.
         * The dictionary path is only necessary for heaptypes (currently not
         * used) and metaclasses.
         * If `__doc__` as stored in `tp_dict` is None, we assume this was
         * filled in by `PyType_Ready()` and should also be replaced.
         */
        PyTypeObject *new = (PyTypeObject *)obj;
        _ADDDOC(new->tp_doc, new->tp_name);
        if (new->tp_dict != NULL && PyDict_CheckExact(new->tp_dict) &&
            PyDict_GetItemString(new->tp_dict, "__doc__") == Py_None)
        {
            /* Warning: Modifying `tp_dict` is not generally safe! */
            if (PyDict_SetItemString(new->tp_dict, "__doc__", str) < 0)
            {
                return NULL;
            }
        }
    }
    else if (Py_TYPE(obj) == &PyMemberDescr_Type)
    {
        PyMemberDescrObject *new = (PyMemberDescrObject *)obj;
        _ADDDOC(new->d_member->doc, new->d_member->name);
    }
    else if (Py_TYPE(obj) == &PyGetSetDescr_Type)
    {
        PyGetSetDescrObject *new = (PyGetSetDescrObject *)obj;
        _ADDDOC(new->d_getset->doc, new->d_getset->name);
    }
    else if (Py_TYPE(obj) == &PyMethodDescr_Type)
    {
        PyMethodDescrObject *new = (PyMethodDescrObject *)obj;
        _ADDDOC(new->d_method->ml_doc, new->d_method->ml_name);
    }
    else
    {
        PyObject *doc_attr;

        doc_attr = PyObject_GetAttrString(obj, "__doc__");
        if (doc_attr != NULL && doc_attr != Py_None && (PyUnicode_Compare(doc_attr, str) != 0))
        {
            Py_DECREF(doc_attr);
            if (PyErr_Occurred())
            {
                /* error during PyUnicode_Compare */
                return NULL;
            }
            PyErr_Format(PyExc_RuntimeError, "object %s", msg);
            return NULL;
        }
        Py_XDECREF(doc_attr);

        if (PyObject_SetAttrString(obj, "__doc__", str) < 0)
        {
            PyErr_SetString(PyExc_TypeError, "Cannot set a docstring for that object");
            return NULL;
        }
        Py_RETURN_NONE;
    }

#undef _ADDDOC

    Py_RETURN_NONE;
}
