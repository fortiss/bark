// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "stl_discrete_time_specification.hpp"

namespace bark {
namespace world {
namespace evaluation {

PyObject* StlDiscreteTimeSpecificationWrapper::pModule = NULL;

StlDiscreteTimeSpecificationWrapper::StlDiscreteTimeSpecificationWrapper() {
  PythonInterpreter::GetInstance().Initialize();

  if (pModule == NULL) {
    pModule = PyImport_ImportModule("rtamt");
    if (pModule == NULL) {
      PyErr_Print();
      std::cerr << "Failed to load rtamt Python module" << std::endl;
    }
  }
}

bool StlDiscreteTimeSpecificationWrapper::InitializePyObj() {
  if (pModule) {
    PyObject* pClass = PyObject_GetAttrString(pModule, "StlDiscreteTimeSpecification");

    if (pClass && PyCallable_Check(pClass)) {
      pInstance = PyObject_CallObject(pClass, NULL);

      if (pInstance) {
      //  std::cout << "Successfully initialized StlDiscreteTimeSpecification object!" << std::endl;
       Py_DECREF(pClass);       
       return true;
      }      
    }
    Py_XDECREF(pClass);
  }
    
  return false;
}

void StlDiscreteTimeSpecificationWrapper::DeclareVar(std::string var_name, std::string var_type) {
  PyObject *pArgs = PyTuple_New(2);
  if (!pArgs) {
    return;
  }

  PyObject *pVarName = PyUnicode_FromString(var_name.c_str());
  PyObject *pVarType = PyUnicode_FromString(var_type.c_str());

  if (!pVarName || !pVarType) {
    Py_XDECREF(pVarName);
    Py_XDECREF(pVarType);
    return;
  }

  PyTuple_SetItem(pArgs, 0, pVarName);
  PyTuple_SetItem(pArgs, 1, pVarType);

  PyObject *pMethod = PyObject_GetAttrString(pInstance, "declare_var");
  if (pMethod) {
    PyObject *pResult = PyObject_CallObject(pMethod, pArgs);    
    if (!pResult) {
      PyErr_Print(); 
    }

    Py_XDECREF(pResult);
        
    Py_DECREF(pMethod);
  } else {
    PyErr_Print(); 
  }

  Py_DECREF(pVarName);
  Py_DECREF(pVarType);
}

void StlDiscreteTimeSpecificationWrapper::SetSamplingPeriod(int sampling_period, std::string unit, double tolerance) {
  PyObject *pMethod = NULL;
  PyTypeObject *pType = Py_TYPE(pInstance);

  while (pType != NULL) {
    pMethod = PyObject_GetAttrString((PyObject *)pType, "set_sampling_period");
    if (pMethod != NULL) {
      break;
    }
    pType = pType->tp_base;
  }

  if (pMethod != NULL) {
    PyObject *pSamplingPeriod = PyLong_FromLong(sampling_period);
    PyObject *pUnit = PyUnicode_FromString(unit.c_str());
    PyObject *pTolerance = PyFloat_FromDouble(tolerance);

    if (!pSamplingPeriod || !pUnit || !pTolerance) {
        Py_XDECREF(pSamplingPeriod);
        Py_XDECREF(pUnit);
        Py_XDECREF(pTolerance);
        return;
    }

    PyObject *pArgs = PyTuple_Pack(4, pInstance, pSamplingPeriod, pUnit, pTolerance); 
    if (pArgs != NULL) {
      PyObject *pResult = PyObject_CallObject(pMethod, pArgs);

      if (!pResult) {
        PyErr_Print(); 
      }

      Py_XDECREF(pResult);
    } else {
      
    }

    Py_XDECREF(pArgs);
    Py_XDECREF(pSamplingPeriod);
    Py_XDECREF(pUnit);
    Py_XDECREF(pTolerance);
    Py_XDECREF(pMethod);
  } else {
    std::cout << "Method NOT FOUND in the class hierarchy" << std::endl;
  }
}

void StlDiscreteTimeSpecificationWrapper::SetStringAttribute(std::string attr_name, std::string attr_value) {
  PyObject *pValue = PyUnicode_FromString(attr_value.c_str());

  if (PyObject_SetAttrString(pInstance, attr_name.c_str(), pValue) == -1) {
    PyErr_Print(); 
  }

  Py_XDECREF(pValue);  
}

void StlDiscreteTimeSpecificationWrapper::CallMethodWithNoArg(std::string method_name) {
  PyObject *pMethod = NULL;
  PyTypeObject *pType = Py_TYPE(pInstance);

  while (pType != NULL) {
    pMethod = PyObject_GetAttrString((PyObject *)pType, method_name.c_str());
    if (pMethod != NULL) {
      break;
    }
    pType = pType->tp_base;
  }

  if (pMethod != NULL) {
    PyObject *pArgs = PyTuple_Pack(1, pInstance); 
    if (pArgs != NULL) {
      PyObject *pResult = PyObject_CallObject(pMethod, pArgs);

      if (!pResult) {
        PyErr_Print(); 
      }

      Py_XDECREF(pResult);
    } else {
      
    }

    Py_XDECREF(pArgs);
    Py_XDECREF(pMethod);
  } else {
    std::cout << "Method NOT FOUND in the class hierarchy" << std::endl;
  }
}

double StlDiscreteTimeSpecificationWrapper::UpdateStlSpec(double stl_spec_timestep, 
                                                            std::vector<std::pair<std::string, double>> dataset) const {
  double pyRobustness = std::numeric_limits<double>::infinity();

  PyObject *timestampObj = PyFloat_FromDouble(stl_spec_timestep);
  if (!timestampObj) {
    PyErr_Print();
    return pyRobustness;
  }

  PyObject *datasetList = PyList_New(0);
  if (!datasetList) {
    PyErr_Print();
    Py_DECREF(timestampObj);
    return pyRobustness;
  }

  for (const auto &pair : dataset) {
    PyObject *varNameObj = PyUnicode_FromString(pair.first.c_str());
    PyObject *varValueObj = PyFloat_FromDouble(pair.second);
    PyObject *tuple = PyTuple_Pack(2, varNameObj, varValueObj);

    PyList_Append(datasetList, tuple);

    Py_DECREF(varNameObj);
    Py_DECREF(varValueObj);
    Py_DECREF(tuple);
  }

  PyObject *updateMethod = PyObject_GetAttrString(pInstance, "update");
  if (updateMethod && PyCallable_Check(updateMethod)) {
    PyObject *pArgs = PyTuple_Pack(2, timestampObj, datasetList);
    PyObject *pResult = PyObject_CallObject(updateMethod, pArgs);
    
    double result = PyFloat_AsDouble(pResult);
    if (result == -1.0 && PyErr_Occurred()) {
      PyErr_Print();
    } else {
      pyRobustness = result;
      // std::cout << "ROBUSTNESS PURE: " << pyRobustness << std::endl;
    }

    Py_XDECREF(pResult);
    Py_DECREF(pArgs);
  } else {
    PyErr_Print();
  }

  Py_DECREF(timestampObj);
  Py_DECREF(datasetList);
  Py_XDECREF(updateMethod);  

  return pyRobustness;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
