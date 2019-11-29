# PySpaceDSL Quick Start
At present, **PySpaceDSL** inherits most functions of **SpaceDSL**, 
including most classes and functions. Thread related classes are not included. 
The naming style will follow the C++ version, 
making it easy for users to migrate their previous work to python
## Install
Copy `astrodata` and `PySpaceDSL.cp37-win_amd64.pyd` into your Python Project.
And import PySpaceDSL like `from PySpaceDSL import *`. Then you can use Class in SpaceDSL C++ Lib as follows:
CLASSES
    builtins.Exception(builtins.BaseException)
        SPException
    pybind11_builtins.pybind11_object(builtins.object)
        AccessAnalysis
        AtmosphereModel
        AtmosphericDrag
        CZMLScript
        CalendarTime
            UTCCalTime
        CartState
        Environment
        GeodeticCoord
        GeodeticCoordSystem
        GravityModel
        IntegMethodType
        InterpolationType
        J2OrbitPredict
        JplEphemeris
        Mission
        NormalizeParameter
        Observation
        OrbitElem
        OrbitPredict
        OrbitPredictConfig
        Propagator
        RungeKutta
        SolarRadPressure
        SolarSysStarType
        SpaceVehicle
        TLEOrbitPredict
        Target
            PointTarget
                Facility
        ThirdBodyGravity
        ThirdBodyGravitySign
## Help
You Can usr `help()` function to get Class and Dunction description, like `help(PySpaceDSL)`.

## Run the Test
Copy `astrodata`, `PySpaceDSL.cp37-win_amd64.pyd` and `TestPython.py` together. 
And run `python TestPython.py`.

```

