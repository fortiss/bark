#include "python_interpreter.hpp"

namespace bark {
namespace world {
namespace evaluation {

PythonInterpreter& PythonInterpreter::GetInstance() {
    static PythonInterpreter instance;
    return instance;
}

bool PythonInterpreter::Initialize() {
    if (!initialized_) {
        Py_Initialize();        
        initialized_ = true;
        return true;
    }
    return false;
}

void PythonInterpreter::Finalize() {
    if (initialized_) {        
        Py_Finalize();
        initialized_ = false;
    }
}

PythonInterpreter::PythonInterpreter() : initialized_(false) {  
}

PythonInterpreter::~PythonInterpreter() {
    Finalize();
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark