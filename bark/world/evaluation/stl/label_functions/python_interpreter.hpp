// Copyright (c) 2024 fortiss GmbH
//
// Authors: Esra Acar-Celik and Xiangzhong Liu
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_STL_LABELS_PYTHON_INTERPRETER_HPP_
#define BARK_WORLD_EVALUATION_STL_LABELS_PYTHON_INTERPRETER_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <Python.h>

namespace bark {
namespace world {
namespace evaluation {

class PythonInterpreter {
public:
    static PythonInterpreter& GetInstance();

    bool Initialize();

    void Finalize();

private:
    PythonInterpreter();
    ~PythonInterpreter();

    bool initialized_;
};

#endif  // BARK_WORLD_EVALUATION_STL_LABELS_PYTHON_INTERPRETER_HPP_

}  // namespace evaluation
}  // namespace world
}  // namespace bark