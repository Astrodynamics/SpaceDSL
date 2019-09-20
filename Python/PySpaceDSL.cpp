/************************************************************************
* Copyright (C) 2019 Niu ZhiYong
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Niu ZhiYong
* Date:2019-09-20
* Description:
*   PySpaceDSL.cpp
*
*   Purpose:
*
*         Build Python API by pybind11.
*
*
*   Last modified:
*
*   2019-09-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <SpaceDSL/SpaceDSL.h>

using namespace SpaceDSL;
using namespace Eigen;
namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}


PYBIND11_MODULE(PySpaceDSL, m) {
    m.doc() = R"pbdoc(
        SpaceDSL is a astrodynamics simulation library. This library is Written by C++.
        The purpose is to provide an open framework for astronaut dynamics enthusiasts,
        and more freely to achieve astrodynamics simulation.
        The project is open under the MIT protocol, and it is also for freer purposes.
        The project is built with CMake and can be used on Windows, Linux and Mac OS.
        This library can compiled into static library, dynamic library and Python library.
    )pbdoc";
    m.def("GM_Earth",[](){return GM_Earth;});
    py::class_<CartState>(m, "CartState",
                          R"pbdoc(
                             Cartesian State Elements.
                             Position and velocity in cartesian coordinates.
                         )pbdoc")
            .def(py::init<>())
            .def(py::init<double, double, double, double, double, double>())
            .def(py::init<Vector3d, Vector3d>())
            .def(py::self - py::self)
            .def(-py::self)
            .def(py::self + py::self)
            .def(py::self += py::self)
            .def(py::self -= py::self)
            .def("Pos", &CartState::Pos)
            .def("Vel", &CartState::Vel)
            .def("SetPos", py::overload_cast<const Vector3d&>(&CartState::SetPos))
            .def("SetVel", py::overload_cast<const Vector3d&>(&CartState::SetVel))
            .def("SetPos", py::overload_cast<double, double, double>(&CartState::SetPos))
            .def("SetVel", py::overload_cast<double, double, double>(&CartState::SetVel));

//    py::module SpConst = m.def_submodule("SpConst", "Define The Constants in Library");

//    py::module SpMath = m.def_submodule("SpMath", "Define The Math Function in Library");

//      py::enum_<Type::Value>(sm, "Type")
//        .value("Cat", Type::Cat)
//        .value("Dog", Type::Dog)
//        .export_values();



#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}


