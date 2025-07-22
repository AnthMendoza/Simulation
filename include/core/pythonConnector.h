#ifndef PIDGAINSCONNECTOR_H
#define PIDGAINSCONNECTOR_H
#include <Python.h>
#include <iostream>
#include <vector>
using dataPID = std::tuple<float,float,float>;
using PIDPair = std::pair<dataPID,dataPID>;




void initPython() {
    
    Py_Initialize();
    
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.insert(0, '/Users/anthony/Documents/rocket/venv/lib/python3.13/site-packages')");  
    PyRun_SimpleString("sys.path.append('../src/control/')");
}





dataPID getNextPIDSingle() {
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
    dataPID PID = {result[0],result[1],result[2]};
    return PID;
}

void costToPythonSingle(const dataPID& pid, double cost) {
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

    // Pack arguments (pid list, cost)
    PyObject* pArgs = PyTuple_Pack(2, pPidList, PyFloat_FromDouble(cost));
    PyObject_CallObject(pFunc, pArgs);

    // Cleanup
    Py_DECREF(pArgs);
    Py_DECREF(pPidList);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
}


void callPythonResetFunction(const char* funcName) {
    PyObject* pName = PyUnicode_DecodeFSDefault("bayesianOptimization");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        return;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, funcName);
    if (!pFunc || !PyCallable_Check(pFunc)) {
        PyErr_Print();
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        return;
    }

    PyObject_CallObject(pFunc, nullptr);

    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
}

void resetOptimizer() {
    callPythonResetFunction("reset");
}

void resetOptimizerDuel() {
    callPythonResetFunction("resetDuel");
}




PIDPair getNextDuelPID() {
    PyObject *pName = PyUnicode_DecodeFSDefault("bayesianOptimization");
    PyObject *pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    PyObject *pFunc = PyObject_GetAttrString(pModule, "getNextDuel");
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

    dataPID PID1 = {result[0], result[1], result[2]};
    dataPID PID2 = {result[3], result[4], result[5]};
    PIDPair data = {PID1,PID2};
    return data;
}

void costToPythonDuel(const PIDPair PIDs, double cost) {
    PyObject* pName = PyUnicode_DecodeFSDefault("bayesianOptimization");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        return;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "updateResultDuel");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        PyErr_Print();
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        return;
    }

    float p1, i1, d1, p2, i2, d2;
    std::tie(p1, i1, d1) = PIDs.first;
    std::tie(p2, i2, d2) = PIDs.second;

    PyObject* pPidList = PyList_New(6);
    PyList_SetItem(pPidList, 0, PyFloat_FromDouble(p1));
    PyList_SetItem(pPidList, 1, PyFloat_FromDouble(i1));
    PyList_SetItem(pPidList, 2, PyFloat_FromDouble(d1));
    PyList_SetItem(pPidList, 3, PyFloat_FromDouble(p2));
    PyList_SetItem(pPidList, 4, PyFloat_FromDouble(i2));
    PyList_SetItem(pPidList, 5, PyFloat_FromDouble(d2));

    PyObject* pArgs = PyTuple_Pack(2, pPidList, PyFloat_FromDouble(cost));
    PyObject_CallObject(pFunc, pArgs);

    Py_DECREF(pArgs);
    Py_DECREF(pPidList);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
}


template<typename PIDType>
PIDType getNextPID();
// these empyt <> are a little tricky but they are creating a specific defination for the gneralized typename template.
//Note to self.
template<>
dataPID getNextPID<dataPID>(){
    return getNextPIDSingle();  
}

template<>
PIDPair getNextPID<PIDPair>(){
    return getNextDuelPID();
}

template<typename PIDType>
void costToPython(PIDType PID,double cost){
    static_assert(sizeof(PIDType) == 0, "costToPython is not defined for this type.");
}

template<>
void costToPython<dataPID>(dataPID PID,double cost){
    return costToPythonSingle(PID,cost);  
}

template<>
void costToPython<PIDPair>(PIDPair PID,double cost){
    return costToPythonDuel(PID,cost);
}

template<typename PIDType>
void displayPID(PIDType PID , const char* type) {
    static_assert(sizeof(PIDType) == 0, "displayPID is not defined for this type.");
}
template<typename PIDType>
void displayPID(PIDType PID) {
    static_assert(sizeof(PIDType) == 0, "displayPID is not defined for this type.");
}

template<>
void displayPID<dataPID>(dataPID PID, const char* type){

    std::cout << type << ":\n" <<
    std::get<0>(PID) << ","<< 
    std::get<1>(PID) << ","<< 
    std::get<2>(PID)<<"\n";
}

template<>
void displayPID<PIDPair>(PIDPair PID , const char* type){

    std::cout<< type << ":\n" <<
    std::get<0>(PID.first) << ","<< 
    std::get<1>(PID.first) << ","<< 
    std::get<2>(PID.first)<<"\n";

    std::cout<<
    std::get<0>(PID.second) << ","<< 
    std::get<1>(PID.second) << ","<< 
    std::get<2>(PID.second)<<"\n";
}

template<>
void displayPID<dataPID>(dataPID PID){

    std::cout<< 
    std::get<0>(PID) << ","<< 
    std::get<1>(PID) << ","<< 
    std::get<2>(PID)<<"\n";
}

template<>
void displayPID<PIDPair>(PIDPair PID){

    std::cout<<
    std::get<0>(PID.first) << ","<< 
    std::get<1>(PID.first) << ","<< 
    std::get<2>(PID.first)<<"\n";

    std::cout<<
    std::get<0>(PID.second) << ","<<
    std::get<1>(PID.second) << ","<<
    std::get<2>(PID.second)<<"\n";
}





#endif //PIDGAINSCONNECTOR_H