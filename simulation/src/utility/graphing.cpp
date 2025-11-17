#include "../../include/utility/graphing.h"
#include <Python.h>

namespace utility{

void plot2d(const std::vector<std::vector<float>>& data) {
    Py_Initialize();
    
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.insert(0, '/Users/anthony/Documents/rocket/venv/lib/python3.13/site-packages')");  
    PyRun_SimpleString("sys.path.append('../src/utility/')");
    
    PyObject* pName = PyUnicode_DecodeFSDefault("graphing"); 
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);
    
    PyObject* pFunc = PyObject_GetAttrString(pModule, "plot");
    
    PyObject* pList = PyList_New(data.size());
    for (size_t i = 0; i < data.size(); i++) {
        PyObject* pRow = PyList_New(data[i].size());
        for (size_t j = 0; j < data[i].size(); j++) {
            PyList_SetItem(pRow, j, PyFloat_FromDouble(data[i][j]));
        }
        PyList_SetItem(pList, i, pRow);
    }
    
    PyObject_CallFunctionObjArgs(pFunc, pList, NULL);
    
    Py_DECREF(pList);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();
}


}