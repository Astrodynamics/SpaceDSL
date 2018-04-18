![logo](https://github.com/Astrodynamics/SpaceDSL/blob/master/doc/logo.png)

# SpaceDSL

**SpaceDSL** is a astrodynamics simulation library. This library is Written by C++.
The purpose is to provide an open framework for astronaut dynamics enthusiasts, and more freely to achieve astrodynamics simulation. 
The project is open under the 'MIT' protocol, and it is also for freer purposes. 
The project is built with CMake and can be used on 'Windows', 'Linux' and 'Mac OS'. 
This library can compiled into 'static library' or 'dynamic library'. Of course, it can also be used directly.

## Core features

**SpaceDSL** contain all basic functions of astrodynamics simulation, including:
- Astronomical Time System: 'Support EOP web Service', including the definition and conversion of various astronomical time. such as UTC, UT1, TT...
- Reference Coordinate System: including the transformation equations of commonly used spatial coordinate system. such as ECI, TOD, ECEF, VVLH...
- JPL Ephemeris: Repackage the JPL ephemeris reading module, providing the latest 'DE436' file, including data from '1950' to '2050'.
- Earth Gravity Model: Using the spherical harmonic function to build the earth's high-precision gravitational model, with 'EGM200'8 data(80X80).
- Other Perturbation Models: Atmospheric Drag('USSA1976'), Solar Radiation Pressure, Third Body Gravity.
- High Precision Orbit Prediction For Spacecraft.

### Later support

- Perturbation Models: Earth Tide, Relativistic Effect .
- Atmospheric Models: Harris-Priester Model.
- Orbital Maneuver Siumulation Features.

## Dependences

- [Eigen3 library](http://eigen.tuxfamily.org): a C++ template library for linear algebra.'Be careful！ The library must be recompiled when using SpaceDSL dynamic library'.
All the files of the Eigen3.3.4 have been included in the project.

## Supported compilers

1. Clang/LLVM 3.3 or newer (for Apple Xcode's clang, this is 5.0.0 or newer)
2. GCC 4.8 or newer
3. Microsoft Visual Studio 2015 Update 3 or newer
4. Intel C++ compiler 16 or newer 

## Reference
The formula used in this project is from 'Oliver Montenbruck Eberhard Gill''s book 'Satellite Orbit Model, Method and Appliction'.


## About

This project was created by 'Niu Zhiyong'
During the development process, we received the guidance and help from 'Professor Wang Hua' of NUDT Aerospace College.
Significant features and/or improvements to the code were contributed by
'Sun Zhenjiang'


## License

**SpaceDSL** is provided under MIT license that can be found in the
``LICENSE`` file. By using, distributing, or contributing to this project,
you agree to the terms and conditions of this license.
