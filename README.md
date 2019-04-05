![logo](https://github.com/Astrodynamics/SpaceDSL/blob/master/doc/logo.png)

# SpaceDSL(Testing)[![Build Status](https://travis-ci.org/Astrodynamics/SpaceDSL.svg?branch=master)](https://travis-ci.org/Astrodynamics/SpaceDSL.svg?branch=master)&nbsp;

**SpaceDSL** is a astrodynamics simulation library. This library is Written by C++.
The purpose is to provide an open framework for astronaut dynamics enthusiasts, and more freely to achieve astrodynamics simulation. 
The project is open under the `MIT` protocol, and it is also for freer purposes. 
The project is built with CMake and can be used on `Windows`, `Linux` and `Mac OS`. 
This library can compiled into `static library` or `dynamic library`. Of course, it can also be used directly.

## Core features

**SpaceDSL** contain all basic functions of astrodynamics simulation, including:
- Astronomical Time System: `Support EOP web Service`, including the definition and conversion of various astronomical time. such as UTC, UT1, TT...
- Reference Coordinate System: including the transformation equations of commonly used spatial coordinate system. such as ECI, TOD, ECEF, VVLH...
- JPL Ephemeris: Repackage the JPL ephemeris reading module, providing the latest `DE436` file, including data from `1950` to `2050`.
- Earth Gravity Model: Using the spherical harmonic function to build the earth's high-precision gravitational model, with `EGM2008` data(80X80).
- Other Perturbation Models: Atmospheric Drag(`NRLMSISE2000`), Solar Radiation Pressure, Third Body Gravity.
- Two Body Orbit Prediction, J2 Orbit Prediction And High Precision Orbit Prediction For Spacecraft.
- Various Ground Target Models and Access Analysis.
- Sensor Support(`Simple Conic` and `Rectangular`).
- Common Physical Constants
- Multi Thread Parallel Support: Similar to Qt API, including Thread and Thread Pool.
- Support Exception Handling Based on STL.
- Provide Nonlinear Optimization Library, Base on [NLopt](https://nlopt.readthedocs.io/en/latest/).
- Provide ElectronVisualizer, Base on [Electron](https://electronjs.org) and [Cesium](https://cesiumjs.org/).
### Later support

- Perturbation Models: Earth Tide, Relativistic Effect .
- Relative Motion Simulation of Spacecraft.
- Orbital Maneuver Siumulation Features.
- Orbit Deviation Prediction.

## Dependences

- [Eigen3 library](http://eigen.tuxfamily.org): a C++ template library for linear algebra.`Be careful！ The library must be recompiled when using SpaceDSL dynamic library`.
All the files of the Eigen3.3.7 have been included in the project.
- [JSON for Modern C++ V3.6.1](https://github.com/nlohmann/json): a headonly JSON lib, Used the c++11 standard and Support STL.
- [OpenSSL](https://www.openssl.org/):OpenSSL is a robust, commercial-grade, and full-featured toolkit for the Transport Layer Security (TLS) and Secure Sockets Layer (SSL) protocols.
If you want to use IERS Web Service, this library must be Precompiled.

## Supported compilers

1. Clang/LLVM 3.3 or newer (for Apple Xcode's clang, this is 5.0.0 or newer)
2. GCC 7.2 or newer
3. Microsoft Visual Studio 2015 Update 3 or newer
4. Intel C++ compiler 16 or newer 

## Reference
1. The formula used in this project is from `Oliver Montenbruck Eberhard Gill`'s book ``Satellite Orbit Model, Method and Appliction``.

## About

This project was created by `Niu Zhiyong`
During the development process, we received the guidance and help from `Professor Wang Hua` of NUDT Aerospace College.
Significant features and/or improvements to the code were contributed by
`Sun Zhenjiang`,`Xiao Gongwei`


## License

**SpaceDSL** is provided under MIT license that can be found in the
``LICENSE`` file. By using, distributing, or contributing to this project,
you agree to the terms and conditions of this license.
