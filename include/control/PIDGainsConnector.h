#ifndef PIDGAINSCONNECTOR_H
#define PIDGAINSCONNECTOR_H
#include <Python.h>
#include <iostream>
#include <vector>

void initPython() {

    Py_Initialize();
    
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.insert(0, '/Users/anthony/Documents/rocket/venv/lib/python3.13/site-packages')");  
    PyRun_SimpleString("sys.path.append('../src/control/')");
}

std::tuple<float,float,float> getNextPID() {
    PyObject *pName = PyUnicode_DecodeFSDefault("bayesianOptimization"); //bayesianOptimization.py
    PyObject *pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    PyObject *pFunc = PyObject_GetAttrString(pModule, "getNext");
    PyObject *pResult = PyObject_CallObject(pFunc, nullptr);

    std::vector<double> result; 
    if (PyList_Check(pResult)) {
        for (Py_ssize_t i = 0; i < PyList_Size(pResult); ++i) {
            result.push_back(PyFloat_AsDouble(PyList_GetItem(pResult, i)));
        }
    }

    Py_DECREF(pResult);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
    std::tuple<float,float,float> PID = {result[0],result[1],result[2]};
    return PID;
}

void costToPython(const std::tuple<float, float, float>& pid, double cost) {
    PyObject* pName = PyUnicode_DecodeFSDefault("bayesianOptimization");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        return;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "updateResult");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        PyErr_Print();
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        return;
    }

    float p, i, d;
    std::tie(p, i, d) = pid;

    PyObject* pPidList = PyList_New(3);
    PyList_SetItem(pPidList, 0, PyFloat_FromDouble(static_cast<double>(p)));
    PyList_SetItem(pPidList, 1, PyFloat_FromDouble(static_cast<double>(i)));
    PyList_SetItem(pPidList, 2, PyFloat_FromDouble(static_cast<double>(d)));

    // Pack (pid list, cost)
    PyObject* pArgs = PyTuple_Pack(2, pPidList, PyFloat_FromDouble(cost));
    PyObject_CallObject(pFunc, pArgs);

    // Cleanup
    Py_DECREF(pArgs);
    Py_DECREF(pPidList);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
}


#endif //PIDGAINSCONNECTOR_H