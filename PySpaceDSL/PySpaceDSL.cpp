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
#include <pybind11/stl.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <SpaceDSL/SpaceDSL.h>

using namespace SpaceDSL;
using namespace Eigen;
namespace py = pybind11;

PYBIND11_MODULE(PySpaceDSL, m) {
	
	m.doc() = R"pbdoc(
        SpaceDSL is a astrodynamics simulation library. This library is Written by C++.
        The purpose is to provide an open framework for astronaut dynamics enthusiasts,
        and more freely to achieve astrodynamics simulation.
        The project is open under the MIT protocol, and it is also for freer purposes.
        The project is built with CMake and can be used on Windows, Linux and Mac OS.
        This library can compiled into static library, dynamic library and Python library.
    )pbdoc";
    // **************************SpConst.h**************************
    m.def("EPS",[](){return EPS;});
    m.def("PI",[](){return PI;});
    m.def("TwoPI",[](){return TwoPI;});
    m.def("FourPI",[](){return FourPI;});
    m.def("HalfPI",[](){return HalfPI;});
    m.def("ThirdPI",[](){return ThirdPI;});
    m.def("QuarterPI",[](){return QuarterPI;});
    m.def("SqrPI",[](){return SqrPI;});

    m.def("MJDOffset",[](){return MJDOffset;});
    m.def("TTMinusTAI",[](){return TTMinusTAI;});

    m.def("B1950Epoch",[](){return B1950Epoch;});
    m.def("J2000Epoch",[](){return J2000Epoch;});
    m.def("MJD_J2000",[](){return MJD_J2000;});

    m.def("EarthSiderealDay",[](){return EarthSiderealDay;});
    m.def("EarthSiderealYear",[](){return EarthSiderealYear;});

    m.def("Grav",[](){return Grav;});
    m.def("GM_Earth",[](){return GM_Earth;});
    m.def("AU",[](){return AU;});
    m.def("EarthRadius",[](){return EarthRadius;});
    m.def("EarthFlatFact",[](){return EarthFlatFact;});
    m.def("LightSpeed",[](){return LightSpeed;});
    m.def("EarthAngVel",[](){return EarthAngVel;});
    m.def("EarthMeanMotion",[](){return EarthMeanMotion;});
    m.def("EarthRPSDay",[](){return EarthRPSDay;});

    m.def("Earth_J2",[](){return Earth_J2;});
    m.def("Earth_J3",[](){return Earth_J3;});
    m.def("Earth_J4",[](){return Earth_J4;});

    m.def("WGS84RE",[](){return WGS84RE;});
    m.def("WGS84RP",[](){return WGS84RP;});
    m.def("WGS84F",[](){return WGS84F;});
    m.def("WGS84GM",[](){return WGS84GM;});
    m.def("WGS84WR",[](){return WGS84WR;});
    m.def("WGS84WR_INERTIAL",[](){return WGS84WR_INERTIAL;});
    m.def("WGS84SRE",[](){return WGS84SRE;});

    m.def("GM_Moon",[](){return GM_Moon;});
    m.def("MoonRadius",[](){return MoonRadius;});
    m.def("MoonMinRadius",[](){return MoonMinRadius;});
    m.def("Moon_J2",[](){return Moon_J2;});

    m.def("GM_Jupiter",[](){return GM_Jupiter;});
    m.def("JupiterRadius",[](){return JupiterRadius;});
    m.def("JupiterMinRadius",[](){return JupiterMinRadius;});
    m.def("Jupiter_J2",[](){return Jupiter_J2;});

    m.def("GM_Mars",[](){return GM_Mars;});
    m.def("MarsRadius",[](){return MarsRadius;});
    m.def("MarsMinRadius",[](){return MarsMinRadius;});
    m.def("Mars_J2",[](){return Mars_J2;});

    m.def("GM_Mercury",[](){return GM_Mercury;});
    m.def("MercuryRadius",[](){return MercuryRadius;});
    m.def("MercuryMinRadius",[](){return MercuryMinRadius;});
    m.def("Mercury_J2",[](){return Mercury_J2;});

    m.def("GM_Neptune",[](){return GM_Neptune;});
    m.def("NeptuneRadius",[](){return NeptuneRadius;});
    m.def("NeptuneMinRadius",[](){return NeptuneMinRadius;});
    m.def("Neptune_J2",[](){return Neptune_J2;});

    m.def("GM_Pluto",[](){return GM_Pluto;});
    m.def("PlutoRadius",[](){return PlutoRadius;});
    m.def("PlutoMinRadius",[](){return PlutoMinRadius;});
    m.def("Pluto_J2",[](){return Pluto_J2;});

    m.def("GM_Saturn",[](){return GM_Saturn;});
    m.def("SaturnRadius",[](){return SaturnRadius;});
    m.def("SaturnMinRadius",[](){return SaturnMinRadius;});
    m.def("Saturn_J2",[](){return Saturn_J2;});

    m.def("GM_Sun",[](){return GM_Sun;});
    m.def("SunRadius",[](){return SunRadius;});
    m.def("SunMinRadius",[](){return SunMinRadius;});
    m.def("SolarRadPreAtAU",[](){return SolarRadPreAtAU;});
    m.def("Sun_J2",[](){return Sun_J2;});

    m.def("GM_Uranus",[](){return GM_Uranus;});
    m.def("UranusRadius",[](){return UranusRadius;});
    m.def("UranusMinRadius",[](){return UranusMinRadius;});
    m.def("Uranus_J2",[](){return Uranus_J2;});

    m.def("GM_Venus",[](){return GM_Venus;});
    m.def("VenusRadius",[](){return VenusRadius;});
    m.def("VenusMinRadius",[](){return VenusMinRadius;});
    m.def("Venus_J2",[](){return Venus_J2;});

    m.def("MeterToFeet",[](){return MeterToFeet;});
    m.def("MeterToStatMi",[](){return MeterToStatMi;});
    m.def("MeterToNautMi",[](){return MeterToNautMi;});
    m.def("MeterToKilometer",[](){return MeterToKilometer;});
    m.def("MeterToKiloFeet",[](){return MeterToKiloFeet;});

    m.def("FeetToMeter",[](){return FeetToMeter;});
    m.def("StatMiToMeter",[](){return StatMiToMeter;});
    m.def("NautMiToMeter",[](){return NautMiToMeter;});
    m.def("KilometerToMeter",[](){return KilometerToMeter;});
    m.def("KiloFeetToMeter",[](){return KiloFeetToMeter;});

    m.def("MeterToInch",[](){return MeterToInch;});
    m.def("MeterToCentimeter",[](){return MeterToCentimeter;});
    m.def("MeterToMillimeter",[](){return MeterToMillimeter;});
    m.def("MeterToMicron",[](){return MeterToMicron;});
    m.def("MeterToNanometer",[](){return MeterToNanometer;});

    m.def("InchToMeter",[](){return InchToMeter;});
    m.def("CentimeterToMeter",[](){return CentimeterToMeter;});
    m.def("MillimeterToMeter",[](){return MillimeterToMeter;});
    m.def("MicronToMeter",[](){return MicronToMeter;});
    m.def("NanometerToMeter",[](){return NanometerToMeter;});

    m.def("SecToMin",[](){return SecToMin;});
    m.def("SecToHour",[](){return SecToHour;});
    m.def("SecToDay",[](){return SecToDay;});
    m.def("SecToCenday",[](){return SecToCenday;});
    m.def("SecToEarthTU",[](){return SecToEarthTU;});
    m.def("SecToSunTU",[](){return SecToSunTU;});

    m.def("MinToSec",[](){return MinToSec;});
    m.def("MinToHour",[](){return MinToHour;});
    m.def("MinToDay",[](){return MinToDay;});

    m.def("HourToSec",[](){return HourToSec;});
    m.def("HourToMin",[](){return HourToMin;});
    m.def("HourToDay",[](){return HourToDay;});

    m.def("DayToSec",[](){return DayToSec;});
    m.def("DayToMin",[](){return DayToMin;});
    m.def("DayToHour",[](){return DayToHour;});

    m.def("CendayToSec",[](){return CendayToSec;});
    m.def("EarthTUToSec",[](){return EarthTUToSec;});
    m.def("SunTUToSec",[](){return SunTUToSec;});
    m.def("AUPerDay",[](){return AUPerDay;});

    m.def("MilliSecToSec",[](){return MilliSecToSec;});
    m.def("MicroSecToSec",[](){return MicroSecToSec;});
    m.def("NanoSecToSec",[](){return NanoSecToSec;});
    m.def("PicoSecToSec",[](){return PicoSecToSec;});

    m.def("SecToMilliSec",[](){return SecToMilliSec;});
    m.def("SecToMicroSec",[](){return SecToMicroSec;});
    m.def("SecToNanoSec",[](){return SecToNanoSec;});
    m.def("SecToPicoSec",[](){return SecToPicoSec;});

    m.def("RadToDeg",[](){return RadToDeg;});
    m.def("RadToArcSec",[](){return RadToArcSec;});
    m.def("RadToArcMin",[](){return RadToArcMin;});
    m.def("RadToSecArc",[](){return RadToSecArc;});

    m.def("DegToRad",[](){return DegToRad;});
    m.def("ArcSecToRad",[](){return ArcSecToRad;});
    m.def("ArcMinToRad",[](){return ArcMinToRad;});
    m.def("SecArcToRad",[](){return SecArcToRad;});
    m.def("MinArcToRad",[](){return MinArcToRad;});

    m.def("KilogramToSlug",[](){return KilogramToSlug;});
    m.def("KilogramToPound",[](){return KilogramToPound;});

    m.def("SlugToKilogram",[](){return SlugToKilogram;});
    m.def("PoundToKilogram",[](){return PoundToKilogram;});

    m.def("WattToMilliwatt",[](){return WattToMilliwatt;});
    m.def("WattToKilowatt",[](){return WattToKilowatt;});
    m.def("WattToMegawatt",[](){return WattToMegawatt;});
    m.def("WattToGigawatt",[](){return WattToGigawatt;});

    m.def("MilliwattToWatt",[](){return MilliwattToWatt;});
    m.def("KilowattToWatt",[](){return KilowattToWatt;});
    m.def("MegawattToWatt",[](){return MegawattToWatt;});
    m.def("GigawattToWatt",[](){return GigawattToWatt;});

    m.def("HertzToKilohertz",[](){return HertzToKilohertz;});
    m.def("HertzToMegahertz",[](){return HertzToMegahertz;});
    m.def("HertzToGigahertz",[](){return HertzToGigahertz;});
    m.def("HertzToTerahertz",[](){return HertzToTerahertz;});

    m.def("KilohertzToHertz",[](){return KilohertzToHertz;});
    m.def("MegahertzToHertz",[](){return MegahertzToHertz;});
    m.def("GigahertzToHertz",[](){return GigahertzToHertz;});
    m.def("TerahertzToHertz",[](){return TerahertzToHertz;});

    //**************************SpMath.h**************************
    m       .def("Fraction", &Fraction)
            .def("Modulo", &Modulo)
            .def("Delta", &Delta)
            .def("Factorial", &Factorial);

    //**************************SpThread.h**************************
    m       .def("GetHardwareConcurrency", &GetHardwareConcurrency);
    /*
    py::class_<SpThread> spThread(m, "SpThread",
                          R"pbdoc(
                             Thread Class of SpaceDSL
                         )pbdoc");
    spThread.def(py::init<>())
            .def("SetPriority", &SpThread::SetPriority)
            .def("Getpriority", &SpThread::Getpriority)
            .def("GetThreadID", &SpThread::GetThreadID)
            .def("GetCreateTime", &SpThread::GetCreateTime)
            .def("Start", &SpThread::Start)
            .def("Suspend", &SpThread::Suspend)
            .def("Resume", &SpThread::Resume)
            .def("Wait", &SpThread::Wait)
            .def("isRunning", &SpThread::isRunning)
            .def("isFinished", &SpThread::isFinished);

    py::enum_<SpThread::Priority>(spThread, "Priority")
            .value("NormalPriority", SpThread::Priority::NormalPriority)
            .value("IdlePriority", SpThread::Priority::IdlePriority)
            .value("LowestPriority", SpThread::Priority::LowestPriority)
            .value("LowPriority", SpThread::Priority::LowPriority)
            .value("HighPriority", SpThread::Priority::HighPriority)
            .value("HighestPriority", SpThread::Priority::HighestPriority)
            .value("TimeCriticalPriority", SpThread::Priority::TimeCriticalPriority)
            .export_values();


    py::class_<SpThreadPool>(m, "SpThread",
                          R"pbdoc(
                             Thread Class of SpaceDSL
                         )pbdoc")
            .def(py::init<>())
            .def("Start", &SpThreadPool::Start)
            .def("Clear", &SpThreadPool::Clear)
            .def("WaitForDone", &SpThreadPool::WaitForDone, py::arg("msecs") = -1)
            .def("SetMaxThreadCount", &SpThreadPool::SetMaxThreadCount)
            .def("GetMaxThreadCount", &SpThreadPool::GetMaxThreadCount)
            .def("GetActiveThreadCount", &SpThreadPool::GetActiveThreadCount);
    */
    // **************************SpTimeSystem.h**************************
    py::class_<CalendarTime>(m, "CalendarTime",
                          R"pbdoc(
                             Gregorian Calendar Time
                         )pbdoc")
            .def(py::init<>())
            .def(py::self - py::self)
            .def(py::self == py::self)
            .def(py::self != py::self)
            .def(py::self >= py::self)
            .def(py::self > py::self)
            .def(py::self <= py::self)
            .def(py::self < py::self)
            .def("Year", &CalendarTime::Year)
            .def("Mon",  &CalendarTime::Mon)
            .def("Day",  &CalendarTime::Day)
            .def("Hour", &CalendarTime::Hour)
            .def("Min",  &CalendarTime::Min)
            .def("Sec",  &CalendarTime::Sec)
            .def("SetYear", &CalendarTime::SetYear)
            .def("SetMon",  &CalendarTime::SetMon)
            .def("SetDay",  &CalendarTime::SetDay)
            .def("SetHour", &CalendarTime::SetHour)
            .def("SetMin",  &CalendarTime::SetMin)
            .def("SetSec",  &CalendarTime::SetSec)
            .def("ToString",  &CalendarTime::ToString)
            .def("__repr__",  &CalendarTime::ToString);

    py::class_<UTCCalTime, CalendarTime>(m, "UTCCalTime",
                                         R"pbdoc(
                                            UTC Time
                                        )pbdoc")
            .def(py::init<>())
            .def(py::init<int, int, int, int, int, double>());

    m.def("CalendarTimeToMjd",py::overload_cast<int, int, int, int, int, double>(&CalendarTimeToMjd));
    m.def("CalendarTimeToMjd",py::overload_cast<const CalendarTime &>(&CalendarTimeToMjd));

    m.def("MjdToCalendarTime",py::overload_cast<double, int&, int&, int&, int&, int&, double&>(&MjdToCalendarTime));
    m.def("MjdToCalendarTime",py::overload_cast<double, CalendarTime &>(&MjdToCalendarTime));

    // **************************SpOrbitParam.h**************************
    py::class_<CartState>(m, "CartState",
                          R"pbdoc(
                             Cartesian State Elements.
                             Position and velocity in cartesian coordinates.
                         )pbdoc")
            .def(py::init<>())
            .def(py::init<const double, const double, const double, const double, const double, const double>())
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

    py::class_<OrbitElem>(m, "OrbitElem",
                          R"pbdoc(
                             Classic Orbit Element
                         )pbdoc")
            .def(py::init<>())
            .def(py::init<const double, const double, const double, const double, const double, const double>())
            .def("SMajAx", &OrbitElem::SMajAx)
            .def("Ecc",  &OrbitElem::Ecc)
            .def("I",  &OrbitElem::I)
            .def("RAAN", &OrbitElem::RAAN)
            .def("ArgPer",  &OrbitElem::ArgPeri)
            .def("TrueA",  &OrbitElem::TrueA)
            .def("SetSMajAx", &OrbitElem::SetSMajAx)
            .def("SetEcc",  &OrbitElem::SetEcc)
            .def("SetI",  &OrbitElem::SetI)
            .def("SetRAAN", &OrbitElem::SetRAAN)
            .def("SetArgPeri",  &OrbitElem::SetArgPeri)
            .def("SetTrueA",  &OrbitElem::SetTrueA);

    py::class_<GeodeticCoord>(m, "GeodeticCoord",
                          R"pbdoc(
                              Geodetic Coordinate
                             Longitude [rad] Latitude [rad] Altitude [m]
                         )pbdoc")
            .def(py::init<>())
            .def(py::init<const double, const double, const double>())
            .def("Longitude", &GeodeticCoord::Longitude)
            .def("Latitude",  &GeodeticCoord::Latitude)
            .def("Altitude",  &GeodeticCoord::Altitude)
            .def("SetLongitude", &GeodeticCoord::SetLongitude)
            .def("SetLatitude",  &GeodeticCoord::SetLatitude)
            .def("SetAltitude",  &GeodeticCoord::SetAltitude)
            .def("SetGeodeticCoord",  &GeodeticCoord::SetGeodeticCoord);

    m.def("CheakEccentricity", &CheakEccentricity);
    m.def("CheakOrbit", &CheakOrbit);

    m.def("ApogeeRadToApoAlt", &ApogeeRadToApoAlt);
    m.def("ApogeeAltToApoRad", &ApogeeAltToApoRad);

    m.def("ApogeeRadToMeanMotn", &ApogeeRadToMeanMotn);
    m.def("MeanMotionToApoRad", &MeanMotionToApoRad);

    m.def("ApogeeRadToPeriRad", &ApogeeRadToPeriRad);
    m.def("PerigeeRadToApoRad", &PerigeeRadToApoRad);

    m.def("MeanAnomalyToEcc", &MeanAnomalyToEcc);
    m.def("EccAnomalyToMean", &EccAnomalyToMean);

    m.def("TrueAnomalyToEcc", &TrueAnomalyToEcc);
    m.def("EccAnomalyToTrue", &EccAnomalyToTrue);

    m.def("TrueAnomalyToEcc", &MeanAnomalyToTrue);
    m.def("EccAnomalyToTrue", &TrueAnomalyToMean);

    m.def("OrbitElemToCart", &OrbitElemToCart);
    m.def("CartToOrbitElem", &CartToOrbitElem);

    m.def("TLEToCartState", &TLEToCartState);
    m.def("TLEToOrbitElem", &TLEToOrbitElem);

    //**************************SpTarget.h**************************
    py::class_<Target> target(m, "Target",
                          R"pbdoc(
                             The Base class of SpaceDSL Target
                         )pbdoc");
    target  .def(py::init<>())
            .def("GetName", &Target::GetName)
            .def("GetID", &Target::GetID)
            .def("GetTargetType", &Target::GetTargetType)
            .def("GetMinElevation", &Target::GetMinElevation)
            .def("SetName", &Target::SetName)
            .def("SetMinElevation", &Target::SetMinElevation);

    py::enum_<Target::TargetType>(target, "TargetType")
            .value("E_NotDefindTargetType", Target::TargetType::E_NotDefindTargetType)
            .value("E_PointTarget", Target::TargetType::E_PointTarget)
            .value("E_LineTarget", Target::TargetType::E_LineTarget)
            .value("E_AreaTarget", Target::TargetType::E_AreaTarget)
            .value("E_Facility", Target::TargetType::E_Facility)
            .export_values();

    py::class_<PointTarget, Target>(m, "PointTarget",
                                         R"pbdoc(
                                            SpaceDSL Point Target
                                            Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
                                        )pbdoc")
            .def(py::init<>())
            .def(py::init<const string &, const double, const double, const double, const double>())
            .def(py::init<const string &, const GeodeticCoord &, const double>())
            .def("SetGeodeticCoord", py::overload_cast<const GeodeticCoord &>(&PointTarget::SetGeodeticCoord))
            .def("SetGeodeticCoord", py::overload_cast<const double, const double, const double>(&PointTarget::SetGeodeticCoord))
            .def("GetGeodeticCoord", &PointTarget::GetGeodeticCoord, py::return_value_policy::reference_internal)
            .def("GetGeodeticPos", &PointTarget::GetGeodeticPos);

    //**************************SpFacility.h**************************
    py::class_<Facility, PointTarget>(m, "Facility",
                                         R"pbdoc(
                                            SpaceDSL Facility
                                            Longitude [rad] Latitude [rad] Altitude [m]  MinElevation [rad]
                                        )pbdoc")
            .def(py::init<>())
            .def(py::init<const string &, const double, const double, const double, const double>())
            .def(py::init<const string &, const GeodeticCoord &, const double>())
            .def("InsertSensor", &Facility::InsertSensor,
                 py::arg("name"), py::arg("type"),
                 py::arg("halfAngle1"), py::arg("halfAngle2") = 0)
            .def("RemoveSensor", &Facility::RemoveSensor);

    //**************************SpSpaceVehicle.h**************************
    py::class_<SpaceVehicle>(m, "SpaceVehicle",
                                 R"pbdoc(
                                    All Kinds of Space Vehicle
                                )pbdoc")
            .def(py::init<>())
            .def(py::init<const string &, const CalendarTime& ,
                 const CartState& , const double ,
                 const double , const double ,
                 const double , const double >())
            .def(py::init<const string &, const double ,
                 const CartState& , const double ,
                 const double , const double ,
                 const double , const double >())
            .def("SetName", &SpaceVehicle::SetName)
            .def("SetInitialCartState", &SpaceVehicle::SetInitialCartState)
            .def("SetInitialEpoch", py::overload_cast<const CalendarTime&>(&SpaceVehicle::SetInitialEpoch))
            .def("SetInitialEpoch", py::overload_cast<const double>(&SpaceVehicle::SetInitialEpoch))
            .def("SetInitialMass", &SpaceVehicle::SetInitialMass)
            .def("SetCartState", &SpaceVehicle::SetCartState)
            .def("SetTime", py::overload_cast<const CalendarTime&>(&SpaceVehicle::SetTime))
            .def("SetTime", py::overload_cast<const double>(&SpaceVehicle::SetTime))
            .def("SetMass", &SpaceVehicle::SetMass)
            .def("SetDragCoef", &SpaceVehicle::SetDragCoef)
            .def("SetDragArea", &SpaceVehicle::SetDragArea)
            .def("SetSRPCoef", &SpaceVehicle::SetSRPCoef)
            .def("SetSRPArea", &SpaceVehicle::SetSRPArea)
            .def("GetInitialCartState", &SpaceVehicle::GetInitialCartState, py::return_value_policy::reference_internal)
            .def("GetInitialEpoch", &SpaceVehicle::GetInitialEpoch)
            .def("GetInitialMass", &SpaceVehicle::GetInitialMass)
            .def("GetCartState", &SpaceVehicle::GetCartState, py::return_value_policy::reference_internal)
            .def("GetEpoch", &SpaceVehicle::GetEpoch)
            .def("GetDragCoef", &SpaceVehicle::GetDragCoef)
            .def("GetDragArea", &SpaceVehicle::GetDragArea)
            .def("GetSRPCoef", &SpaceVehicle::GetSRPCoef)
            .def("GetSRPArea", &SpaceVehicle::GetSRPArea)
            .def("UpdateState", py::overload_cast<const double, const CartState&, const double>(&SpaceVehicle::UpdateState))
            .def("UpdateState", py::overload_cast<const double, const Vector3d&, const Vector3d&,const double>(&SpaceVehicle::UpdateState))
            .def("Reset", &SpaceVehicle::Reset)
            .def("InsertSensor", &SpaceVehicle::InsertSensor,
                 py::arg("name"), py::arg("type"),
                 py::arg("halfAngle1"), py::arg("halfAngle2") = 0)
            .def("RemoveSensor", &SpaceVehicle::RemoveSensor);

    //**************************SpCoordSystem.h**************************
    m.def("RotateX", &RotateX, R"pbdoc(
                               Elementary Rotations
                               )pbdoc");
    m.def("RotateY", &RotateY, R"pbdoc(
                               Elementary Rotations
                               )pbdoc");
    m.def("RotateZ", &RotateZ, R"pbdoc(
                               Elementary Rotations
                               )pbdoc");

    m.def("GMST", &GMST, R"pbdoc(
                         Greenwich Mean Sidereal Time
                         )pbdoc");

    m.def("GAST", &GAST, R"pbdoc(
                         Greenwich Apparent Sidereal Time
                         )pbdoc");

    m.def("MeanObliquity", &MeanObliquity, R"pbdoc(
                                           Computes the Mean Obliquity of the Ecliptic
                                           )pbdoc");

    m.def("NutationAngles", &NutationAngles, R"pbdoc(
                                             Nutation in Longitude and Obliquity
                                             )pbdoc");

    m.def("EquationEquinox", &EquationEquinox, R"pbdoc(
                                               Computation of the Equation of the Equinoxes
                                               )pbdoc");

    m.def("PrecessMatrix", &PrecessMatrix,
          py::arg("Mjd_TT2"), py::arg("Mjd_TT1") = MJD_J2000,
          R"pbdoc(
          Precession Transformation of Equatorial Coordinates
          )pbdoc");

    m.def("NutationMatrix", &NutationMatrix, R"pbdoc(
                                             Transformation from Mean to True Equator and Equinox
                                             )pbdoc");

    m.def("GWHourAngMatrix", &GWHourAngMatrix, R"pbdoc(
                                               Transformation from True Equator and Equinox to
                                               Earth Equator and Greenwich Meridian System
                                               )pbdoc");

    m.def("PoleMatrix", &PoleMatrix, R"pbdoc(
                                     Transformation from Pseudo Earth-fixed to Earth-fixed Coordinates
                                     )pbdoc");

    m.def("GMSToLTCMtx", &GMSToLTCMtx, R"pbdoc(
                                       Transformation from Greenwich Meridian System to Local Tangent Coordinates
                                       )pbdoc");

    m.def("GetAzEl", &GetAzEl, R"pbdoc(
                               Computes Azimuth and Elevation from Local Tangent Coordinates
                               )pbdoc");

    m.def("VVLHToICSMtx", &VVLHToICSMtx, R"pbdoc(
                                         Transformation VVLH(Vehicle Velocity Local Horizontal) to ICRS
                                         )pbdoc");

    m.def("ICSToVVLHMtx", &ICSToVVLHMtx, R"pbdoc(
                                         Transformation ICRS To VVLH(Vehicle Velocity Local Horizontal)
                                         )pbdoc");

    py::class_<GeodeticCoordSystem> geoCoordSys (m, "GeodeticCoordSystem",
                                                 R"pbdoc(
                                                 Geodetic Coordingot System
                                                 )pbdoc");
    geoCoordSys
            .def(py::init<>())
            .def(py::init<GeodeticCoordSystem::GeodeticCoordType>())
            .def_static("GetJ2000ToECFMtx", &GeodeticCoordSystem::GetJ2000ToECFMtx,
                        py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000)
            .def_static("GetECFToJ2000Mtx", &GeodeticCoordSystem::GetECFToJ2000Mtx,
                        py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000)
            .def_static("GetJ2000ToTODMtx", &GeodeticCoordSystem::GetJ2000ToTODMtx,
                        py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000)
            .def_static("GetTODToJ2000Mtx", &GeodeticCoordSystem::GetTODToJ2000Mtx,
                        py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000)
            .def("GetGeodeticCoord", py::overload_cast<const Vector3d &>(&GeodeticCoordSystem::GetGeodeticCoord), py::arg("pos"),
                 R"pbdoc(Conversion From ECF(Earth Centered Fixed) Position to the Geodetic Coordinate.)pbdoc")
            .def("GetPosition", py::overload_cast<const GeodeticCoord &>(&GeodeticCoordSystem::GetPosition), py::arg("lla"),
                 R"pbdoc(Conversion From the Geodetic Coordinate to ECF(Earth Centered Fixed) Position.)pbdoc")

            .def("GetGeodeticCoord", py::overload_cast<const Vector3d &, double, double>(&GeodeticCoordSystem::GetGeodeticCoord),
                 py::arg("pos"), py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000,
                 R"pbdoc(Conversion From the J2000 Position to the Geodetic Coordinate.)pbdoc")
            .def("GetPosition", py::overload_cast<const GeodeticCoord &, double, double>(&GeodeticCoordSystem::GetPosition),
                 py::arg("lla"), py::arg("Mjd_UTC2"), py::arg("Mjd_UTC1") = MJD_J2000,
                 R"pbdoc(Conversion From the Geodetic Coordinate to the J2000 Position.)pbdoc")
            .def("GetGroundVelocity", &GeodeticCoordSystem::GetGroundVelocity, py::arg("lla"));

    py::enum_<GeodeticCoordSystem::GeodeticCoordType>(geoCoordSys, "GeodeticCoordType")
            .value("E_NotDefinedGeodeticType", GeodeticCoordSystem::GeodeticCoordType::E_NotDefinedGeodeticType)
            .value("E_WGS84System", GeodeticCoordSystem::GeodeticCoordType::E_WGS84System)
            .export_values();
    //**************************SpGravity.h**************************
    py::class_<GravityModel> gravityModel (m, "GravityModel",
                                 R"pbdoc(
                                    The Class of Gravity Model of the Central Body
                                )pbdoc");
    gravityModel
            .def(py::init<>())
            .def(py::init<GravityModel::GravModelType>())
            .def("SetModelType", &GravityModel::SetModelType)
            .def("GetModelType", &GravityModel::GetModelType)
            .def("AccelHarmonicGravity", &GravityModel::AccelHarmonicGravity);

    py::enum_<GravityModel::GravModelType>(gravityModel, "GravModelType")
            .value("E_NotDefinedGravModel", GravityModel::GravModelType::E_NotDefinedGravModel)
            .value("E_EGM08Model", GravityModel::GravModelType::E_EGM08Model)
            .export_values();
    py::class_<ThirdBodyGravitySign>(m, "ThirdBodyGravitySign",
                                 R"pbdoc(
                                    Third Body Gravity Sign
                                )pbdoc")
            .def(py::init<>())
            .def_readwrite("bIsUseMercuryGrav",&ThirdBodyGravitySign::bIsUseMercuryGrav)
            .def_readwrite("bIsUseVenusGrav",&ThirdBodyGravitySign::bIsUseVenusGrav)
            .def_readwrite("bIsUseEarthGrav",&ThirdBodyGravitySign::bIsUseEarthGrav)
            .def_readwrite("bIsUseMarsGrav",&ThirdBodyGravitySign::bIsUseMarsGrav)
            .def_readwrite("bIsUseJupiterGrav",&ThirdBodyGravitySign::bIsUseJupiterGrav)
            .def_readwrite("bIsUseSaturnGrav",&ThirdBodyGravitySign::bIsUseSaturnGrav)
            .def_readwrite("bIsUseUranusGrav",&ThirdBodyGravitySign::bIsUseUranusGrav)
            .def_readwrite("bIsUseNeptuneGrav",&ThirdBodyGravitySign::bIsUseNeptuneGrav)
            .def_readwrite("bIsUsePlutoGrav",&ThirdBodyGravitySign::bIsUsePlutoGrav)
            .def_readwrite("bIsUseMoonGrav",&ThirdBodyGravitySign::bIsUseMoonGrav)
            .def_readwrite("bIsUseSunGrav",&ThirdBodyGravitySign::bIsUseSunGrav);

    //**************************SpJplEph.h**************************
    py::enum_<SolarSysStarType>(m, "SolarSysStarType")
            .value("E_NotDefinedStarType", SolarSysStarType::E_NotDefinedStarType)
            .value("E_Mercury", SolarSysStarType::E_Mercury)
            .value("E_Venus", SolarSysStarType::E_Venus)
            .value("E_Earth", SolarSysStarType::E_Earth)
            .value("E_Mars", SolarSysStarType::E_Mars)
            .value("E_Jupiter", SolarSysStarType::E_Jupiter)
            .value("E_Saturn", SolarSysStarType::E_Saturn)
            .value("E_Uranus", SolarSysStarType::E_Uranus)
            .value("E_Neptune", SolarSysStarType::E_Neptune)
            .value("E_Pluto", SolarSysStarType::E_Pluto)
            .value("E_Moon", SolarSysStarType::E_Moon)
            .value("E_Sun", SolarSysStarType::E_Sun)
            .export_values();

    py::class_<JplEphemeris>(m, "JplEphemeris",
                                 R"pbdoc(
                                    The class of Read JPL Ephemeris
                                )pbdoc")
            .def(py::init<>())
            .def("GetJplEphemerisStartJD", &JplEphemeris::GetJplEphemerisStartJD)
            .def("GetJplEphemerisEndJD", &JplEphemeris::GetJplEphemerisEndJD)
            .def("GetJplEphemerisStep", &JplEphemeris::GetJplEphemerisStep)
            .def("GetJplEphData", &JplEphemeris::GetJplEphData,
                 R"pbdoc(
                 @Param	type
                 JPL_EPHEM_START_JD               0
                 JPL_EPHEM_END_JD                 8
                 JPL_EPHEM_STEP                  16
                 JPL_EPHEM_N_CONSTANTS           24
                 JPL_EPHEM_AU_IN_KM              28
                 JPL_EPHEM_EARTH_MOON_RATIO      36
                 JPL_EPHEM_IPT_ARRAY             44
                 JPL_EPHEM_EPHEMERIS_VERSION    224
                 JPL_EPHEM_KERNEL_SIZE          228
                 JPL_EPHEM_KERNEL_RECORD_SIZE   232
                 JPL_EPHEM_KERNEL_NCOEFF        236
                 JPL_EPHEM_KERNEL_SWAP_BYTES    240)pbdoc")
            .def("GetJplEphemeris", py::overload_cast<double, SolarSysStarType, SolarSysStarType, Vector3d&>(&JplEphemeris::GetJplEphemeris))
            .def("GetJplEphemeris", py::overload_cast<double, SolarSysStarType, SolarSysStarType, Vector3d&, Vector3d&>(&JplEphemeris::GetJplEphemeris));

    //**************************SpAtmosphere.h**************************
    py::class_<AtmosphereModel> atmosphereModel (m, "AtmosphereModel",
                                 R"pbdoc(
                                    Atmosphere Model of Earth
                                )pbdoc");
    atmosphereModel
            .def(py::init<>())
            .def(py::init<AtmosphereModel::AtmosphereModelType>())
            .def("SetAtmosphereModelType", &AtmosphereModel::SetAtmosphereModelType)
            .def("GetAtmosphereModelType", &AtmosphereModel::GetAtmosphereModelType)
            .def("GetAtmosphereTemperature", &AtmosphereModel::GetAtmosphereTemperature,
                 py::arg("Mjd_UT1"), py::arg("altitude"),
                 py::arg("latitude") = 0, py::arg("longitude") = 0,
                 py::arg("f107A") = 150, py::arg("f107") = 150,
                 py::arg("ap") = nullptr, py::arg("useDailyAp") = true)
            .def("GetAtmospherePressure", &AtmosphereModel::GetAtmospherePressure,
                 py::arg("Mjd_UT1"), py::arg("altitude"),
                 py::arg("latitude") = 0, py::arg("longitude") = 0,
                 py::arg("f107A") = 150, py::arg("f107") = 150,
                 py::arg("ap") = nullptr, py::arg("useDailyAp") = true)
            .def("GetAtmosphereDensity", &AtmosphereModel::GetAtmosphereDensity,
                 py::arg("Mjd_UT1"), py::arg("altitude"),
                 py::arg("latitude") = 0, py::arg("longitude") = 0,
                 py::arg("f107A") = 150, py::arg("f107") = 150,
                 py::arg("ap") = nullptr, py::arg("useDailyAp") = true);

    py::enum_<AtmosphereModel::AtmosphereModelType>(atmosphereModel, "AtmosphereModelType")
            .value("E_NotDefinedAtmosphereModel", AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel)
            .value("E_NRLMSISE00Atmosphere", AtmosphereModel::AtmosphereModelType::E_NRLMSISE00Atmosphere)
            .export_values();

    //**************************SpPerturbation.h**************************
    py::class_<ThirdBodyGravity>(m, "ThirdBodyGravity",
                                     R"pbdoc(
                                        The class of The Third Body Gravity Perturbation
                                    )pbdoc")
            .def(py::init<>())
            .def(py::init<SolarSysStarType, SolarSysStarType>(),
                 R"pbdoc(
                 @Input
                 SolarSysStarType thirdBodyStarType,
                 SolarSysStarType centerStarType
                )pbdoc")
            .def("SetThirdBodyStar", &ThirdBodyGravity::SetThirdBodyStar)
            .def("SetCenterStar", &ThirdBodyGravity::SetCenterStar)
            .def("GetThirdBodyStar", &ThirdBodyGravity::GetThirdBodyStar)
            .def("GetCenterStar", &ThirdBodyGravity::GetCenterStar)
            .def("AccelPointMassGravity", &ThirdBodyGravity::AccelPointMassGravity,
                 R"pbdoc(
                 @Input
                 @Param	Mjd_TT          Terrestrial Time (Modified Julian Date)
                 @Param	pos             Satellite Position Vector in the inertial system
                 @Return Acceleration
                )pbdoc");

    py::class_<AtmosphericDrag>(m, "AtmosphericDrag",
                                     R"pbdoc(
                                        The class of The Atmospheric Drag. Perturbation
                                    )pbdoc")
            .def(py::init<>())
            .def(py::init<AtmosphereModel::AtmosphereModelType, GeodeticCoordSystem *>())
            .def("AccelAtmosphericDrag", &AtmosphericDrag::AccelAtmosphericDrag,
                 py::arg("Mjd_UTC"), py::arg("Mjd_UT1"),
                 py::arg("pos"), py::arg("vel"),
                 py::arg("area"), py::arg("dragCoef"), py::arg("mass"),
                 py::arg("f107A") = 150.0, py::arg("f107") = 150.0,
                 py::arg("ap") = nullptr,
                 R"pbdoc(
                 @Input
                 @Param  Mjd_TT          Terrestrial Time (Modified Julian Date)
                 @Param  pos             Satellite position vector in the inertial system [m]
                 @Param  vel             Satellite velocity vector in the inertial system [m/s]
                 @Param  ECIToTODMtx     Transformation matrix to true-of-date inertial system
                 @Param  area            Cross-section [m^2]
                 @Param  mass            Spacecraft mass [kg]
                 @Param  dragCoef        Drag coefficient
                 @Return Acceleration
                )pbdoc");

    py::class_<SolarRadPressure>(m, "SolarRadPressure",
                                     R"pbdoc(
                                        The class of Solar Radiation Pressure
                                    )pbdoc")
            .def(py::init<>())
            .def("AccelSolarRad", &SolarRadPressure::AccelSolarRad,
                 R"pbdoc(
                 @Input
                 @Param  Mjd_TT          Terrestrial Time (Modified Julian Date)
                 @Param  pos             Satellite position vector in the inertial system [m]
                 @Param  area            Cross-section [m^2]
                 @Param  mass            Spacecraft mass [kg]
                 @Param  solarCoef       Solar radiation pressure coefficient
                 @Return Acceleration
                )pbdoc");

    m.def("GeomagneticApToKp", &GeomagneticApToKp, R"pbdoc(Geomagnetic Ap to Kp)pbdoc");

    m.def("GeomagneticKpToAp", &GeomagneticKpToAp, R"pbdoc(Geomagnetic Kp to Ap)pbdoc");

    //**************************SpIntegration.h**************************
    py::enum_<IntegMethodType>(m, "IntegMethodType")
            .value("E_NotDefindIntegMethodType", IntegMethodType::E_NotDefindIntegMethodType)
            .value("E_RungeKutta4", IntegMethodType::E_RungeKutta4)
            .value("E_RungeKutta78", IntegMethodType::E_RungeKutta78)
            .export_values();

    py::class_<RungeKutta>(m, "RungeKutta",
                                     R"pbdoc(
                                        The class of the N th-order Runge-Kutta
                                    )pbdoc")
            .def(py::init<>())
            .def(py::init<IntegMethodType>())
            .def("SetIntegMethodType", &RungeKutta::SetIntegMethodType)
            .def("SetRelativeErrorThreshold", &RungeKutta::SetRelativeErrorThreshold)
            .def("GetRelativeErrorThreshold", &RungeKutta::GetRelativeErrorThreshold)
            .def("OneStep", &RungeKutta::OneStep,
                 py::arg("rightFunc"), py::arg("t"), py::arg("x"),
                 py::arg("initialStep"), py::arg("result"),
                 py::arg("minStep") = 0.0, py::arg("maxStep") = 0.0,
                 py::arg("maxStepAttempts") = 0.0,
                 py::arg("accuracyThreshold") = 0.0,
                 py::arg("bStopIfAccuracyIsViolated") = true,
                 R"pbdoc(
                 @Input
                 @Param  func            Right Function of ODE
                 @Param	t               The value of the independent variable
                 @Param	x               Initial function value
                 @Param	initialStep		Initial step
                 @Param	minStep                     Mix Adapted step
                 @Param	maxStep                     Max Adapted step
                 @Param	maxStepAttempts             Max Attempts
                 @Param	relativeErrorThreshold		relative Error Threshold
                 @Param  bStopIfAccuracyIsViolated   Throw Exception or Not
                 @Output
                 @Param  result  Integral Step
                )pbdoc")
            .def("MultStep", &RungeKutta::MultStep,
                 py::arg("rightFunc"), py::arg("t0"), py::arg("x"),
                 py::arg("initialStep"), py::arg("t"), py::arg("result"),
                 py::arg("minStep") = 0.0, py::arg("maxStep") = 0.0,
                 py::arg("maxStepAttempts") = 0.0,
                 py::arg("accuracyThreshold") = 0.0,
                 py::arg("bStopIfAccuracyIsViolated") = true,
                 R"pbdoc(
                 @Input
                 @Param  func            Right Function of ODE
                 @Param	t0              The Initial value of the independent variable
                 @Param	x               Initial function value
                 @Param	initialStep		Initial step
                 @Param	t               The End value of the independent variable
                 @Param	minStep                     Mix Adapted step
                 @Param	maxStep                     Max Adapted step
                 @Param	maxStepAttempts             Max Attempts
                 @Param	relativeErrorThreshold		relative Error Threshold
                 @Param  bStopIfAccuracyIsViolated   Throw Exception or Not
                 @Output
                 @Param  result  Integral Result
                )pbdoc");

    //**************************SpInterpolation.h**************************
    py::enum_<InterpolationType>(m, "InterpolationType")
            .value("E_NotDefinedInterpolation", InterpolationType::E_NotDefinedInterpolation)
            .value("E_LinearInterpolation", InterpolationType::E_LinearInterpolation)
            .value("E_HermitePolynomialInterpolation", InterpolationType::E_HermitePolynomialInterpolation)
            .value("E_LagrangePolynomialInterpolation", InterpolationType::E_LagrangePolynomialInterpolation)
            .export_values();

    m.def("LinearInterpolation", &LinearInterpolation,
          R"pbdoc(
          @Input
          @Param    x    	Independent Variable Array
          @Param	y		Function value array
          @Param	t		Interpolation Point
          @Output
          @Param    result  Interpolation Result
          )pbdoc");

    m.def("LagrangePolynomialInterpolation", &LagrangePolynomialInterpolation,
          R"pbdoc(
          @Input
          @Param    x    	Independent Variable Array
          @Param	y		Function value array
          @Param	t		Interpolation Point
          @Output
          @Param    result  Interpolation Result
          )pbdoc");

    m.def("HermitePolynomialInterpolation", &HermitePolynomialInterpolation,
          R"pbdoc(
          @Input
          @Param    x    	Independent Variable Array
          @Param	y		Function value array
          @Param	v		Value derivatives array(y')
          @Param	t		Interpolation Point
          @Output
          @Param    result  Interpolation Result [y, y']
          )pbdoc");

    m.def("LagrangePolynomialInterpolationGradient", &LagrangePolynomialInterpolationGradient,
          py::arg("x"), py::arg("y"),
          py::arg("t"), py::arg("step") = 0.01,
          R"pbdoc(
          @Input
          @Param    x    	Independent Variable Array
          @Param	y		Function value array
          @Param	t		Interpolation Point
          @Param	step	Difference Step
          @Output
          @Param    result  Gradient Result
          )pbdoc");

    //**************************SpPropagator.h**************************
    py::class_<Propagator>(m, "Propagator",
                                     R"pbdoc(
                                        Space Propagator Configuration Class
                                     )pbdoc")
            .def(py::init<>())
            .def(py::init<const IntegMethodType, const double, const double,
                 const double, const double, const int, const bool, const bool>(),
                 R"pbdoc(
                 const IntegMethodType integMethodType,
                 const double initialStep,
                 const double accuracy = 0,
                 const double minStep = 0,
                 const double maxStep = 0,
                 const int maxStepAttempts = 0,
                 const bool bStopIfAccuracyIsViolated = true,
                 const bool isUseNormalize = false
                 )pbdoc")
            .def("SetIntegMethodType", &Propagator::SetIntegMethodType)
            .def("SetInitialStep", &Propagator::SetInitialStep)
            .def("SetAdaptedStep", &Propagator::SetAdaptedStep)
            .def("SetAccuracy", &Propagator::SetAccuracy)
            .def("SetMinStep", &Propagator::SetMinStep)
            .def("SetMaxStep", &Propagator::SetMaxStep)
            .def("SetMaxStepAttempts", &Propagator::SetMaxStepAttempts)
            .def("SetStopIfAccuracyIsViolated", &Propagator::SetStopIfAccuracyIsViolated)
            .def("SetIsUseNormalize", &Propagator::SetIsUseNormalize)
            .def("GetIntegMethodType", &Propagator::GetIntegMethodType)
            .def("GetInitialStep", &Propagator::GetInitialStep)
            .def("GetAdaptedStep", &Propagator::GetAdaptedStep)
            .def("GetAccuracy", &Propagator::GetAccuracy)
            .def("GetMinStep", &Propagator::GetMinStep)
            .def("GetMaxStep", &Propagator::GetMaxStep)
            .def("GetMaxStepAttempts", &Propagator::GetMaxStepAttempts)
            .def("GetStopIfAccuracyIsViolated", &Propagator::GetStopIfAccuracyIsViolated)
            .def("GetIsUseNormalize", &Propagator::GetIsUseNormalize)
            .def("ResetAdaptedStep", &Propagator::ResetAdaptedStep);

    //**************************SpEnvironment.h**************************
    py::class_<Environment>(m, "Environment",
                                     R"pbdoc(
                                        Space Environment Configuration Class
                                     )pbdoc")
            .def(py::init<>())
            .def(py::init<const SolarSysStarType, const GravityModel::GravModelType ,
                 const int , const int , const ThirdBodyGravitySign,
                 const GeodeticCoordSystem::GeodeticCoordType ,
                 const AtmosphereModel::AtmosphereModelType ,
                 const double , const double, VectorXd,
                 const bool, const bool>(),
                 R"pbdoc(
                 const SolarSysStarType centerStarType,
                 const GravityModel::GravModelType gravModelType ,
                 const int maxDegree ,
                 const int maxOrder ,
                 const ThirdBodyGravitySign thirdBodyGravSign,
                 const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                 const AtmosphereModel::AtmosphereModelType atmModelType ,
                 const double f107A ,
                 const double f107,
                 VectorXd ap,
                 const bool isUseDrag,
                 const bool isUseSRP
                 )pbdoc")
            .def("SetCenterStarType", &Environment::SetCenterStarType)
            .def("SetGravModelType", &Environment::SetGravModelType)
            .def("SetGravMaxDegree", &Environment::SetGravMaxDegree)
            .def("SetGravMaxOrder", &Environment::SetGravMaxOrder)
            .def("SetThirdBodySign", &Environment::SetThirdBodySign)
            .def("SetGeodeticCoordType", &Environment::SetGeodeticCoordType)
            .def("SetAtmosphereModelType", &Environment::SetAtmosphereModelType)
            .def("SetAverageF107", &Environment::SetAverageF107)
            .def("SetDailyF107", &Environment::SetDailyF107)
            .def("SetGeomagneticIndex", &Environment::SetGeomagneticIndex)
            .def("SetIsUseDrag", &Environment::SetIsUseDrag)
            .def("SetIsUseSRP", &Environment::SetIsUseSRP)
            .def("GetCenterStarType", &Environment::GetCenterStarType)
            .def("GetGravModelType", &Environment::GetGravModelType)
            .def("GetGravMaxDegree", &Environment::GetGravMaxDegree)
            .def("GetGravMaxOrder", &Environment::GetGravMaxOrder)
            .def("GetThirdBodySign", &Environment::GetThirdBodySign)
            .def("GetGeodeticCoordType", &Environment::GetGeodeticCoordType)
            .def("GetAtmosphereModelType", &Environment::GetAtmosphereModelType)
            .def("GetAverageF107", &Environment::GetAverageF107)
            .def("GetDailyF107", &Environment::GetDailyF107)
            .def("GetGeomagneticIndex", &Environment::GetGeomagneticIndex)
            .def("GetIsUseDrag", &Environment::GetIsUseDrag)
            .def("GetIsUseSRP", &Environment::GetIsUseSRP);

    //**************************SpObservation.h**************************
    py::class_<Observation>(m, "Observation",
                                     R"pbdoc(
                                        The class of SpaceDSL Target Observation
                                     )pbdoc")
            .def(py::init<>())
            .def(py::init<const double, const double, const double, const double,
                 const Vector3d &, const Vector3d &>(),
                 R"pbdoc(
                 const double azimuth,
                 const double azimuthRate,
                 const double elevation,
                 const double elevationRate,
                 const Vector3d &posECF,
                 const Vector3d &velECF
                 )pbdoc")
            .def("Azimuth", &Observation::Azimuth)
            .def("AzimuthRate", &Observation::AzimuthRate)
            .def("Elevation", &Observation::Elevation)
            .def("ElevationRate", &Observation::ElevationRate)
            .def("RelativePosECF", &Observation::RelativePosECF)
            .def("RelativeVelECF", &Observation::RelativeVelECF)
            .def("SetAzimuth", &Observation::SetAzimuth)
            .def("SetAzimuthRate", &Observation::SetAzimuthRate)
            .def("SetElevation", &Observation::SetElevation)
            .def("SetElevationRate", &Observation::SetElevationRate)
            .def("SetRelativePosECF", &Observation::SetRelativePosECF)
            .def("SetRelativeVelECF", &Observation::SetRelativeVelECF);

    m.def("CalMaxObservationLat", &CalMaxObservationLat,
          R"pbdoc(
          @Input
          @Param      cart		Observation Object Cart State in J2000
          @Output
          @Return     result      Max Observation Latitude
          )pbdoc");

    m.def("CalObservation", &CalObservation,
          R"pbdoc(
          @Input
          @Param      Mjd         Modified Julian date of UTC
          @Param      cart		Observation Object Cart State in J2000
          @Param      target		Point Target Object Point
          @Output
          @Param      result
          @Return     false : The Target Can not be Seen
          )pbdoc");

    m.def("CalObservationAll", &CalObservationAll,
          R"pbdoc(
          @Input
          @Param      Mjd         Modified Julian date of UTC
          @Param      cart		Observation Object Cart State in J2000
          @Param      target		Point Target Object Point
          @Output
          @Param      result
          @Return     false : The Target Can not be Seen
          )pbdoc");

    m.def("CalSunObservation", &CalSunObservation,
          R"pbdoc(
          @Input
          @Param      Mjd         Modified Julian date of UTC
          @Param      target		Point Target Object Point
          @Output
          @Return     result      Observation Param
          )pbdoc");
    //**************************SpRightFunction.h**************************
//    template <class RightFuncBase = RightFunc> class PyRightFunc : public RightFuncBase {
//    public:
//        using RightFuncBase::RightFuncBase; // Inherit constructors
//        void operator()(double t, const VectorXd &x, VectorXd&result)
//        {
//            PYBIND11_OVERLOAD_PURE(void, RightFunc, (), t, x, result);
//        }
//    };


//    py::class_<RightFunc, PyRightFunc<>> rightFunc(m, "RightFunc");


    //**************************SpOrbitPredict.h**************************
    py::class_<NormalizeParameter>(m, "NormalizeParameter",
                                     R"pbdoc(
                                        The class of Normalization Parameter
                                     )pbdoc")
            .def(py::init<>())
            .def("GetLengthPara", &NormalizeParameter::GetLengthPara)
            .def("GetSpeedPara", &NormalizeParameter::GetSpeedPara)
            .def("GetTimePara", &NormalizeParameter::GetTimePara)
            .def("GetThrustPara", &NormalizeParameter::GetThrustPara);

    py::class_<OrbitPredictConfig>(m, "OrbitPredictConfig",
                                     R"pbdoc(
                                        The class of Orbit Prediction Parameters
                                     )pbdoc")
            .def(py::init<>())
            .def_readwrite_static("DefaultThirdBodySign", &OrbitPredictConfig::DefaultThirdBodySign)
            .def("Initializer", &OrbitPredictConfig::Initializer,
                 py::arg("Mjd_UTC") = 0.0, py::arg("centerStarType") = E_Earth,
                 py::arg("isUseNormalize") = false,
                 py::arg("gravModelType") = GravityModel::GravModelType::E_NotDefinedGravModel,
                 py::arg("maxDegree") = 0, py::arg("maxOrder") = 0,
                 py::arg("thirdBodyGravSign") = OrbitPredictConfig::DefaultThirdBodySign,
                 py::arg("geodeticType") = GeodeticCoordSystem::GeodeticCoordType::E_WGS84System,
                 py::arg("atmModelType") = AtmosphereModel::AtmosphereModelType::E_NotDefinedAtmosphereModel,
                 py::arg("dragCoef") = 0.0, py::arg("dragArea") = 0.0, py::arg("f107A") = 0.0,
                 py::arg("f107") = 0.0, py::arg("ap") = nullptr, py::arg("SRPCoef") = 0.0,
                 py::arg("SRPArea") = 0.0, py::arg("isUseDrag") = false, py::arg("isUseSRP") = false)
            .def("IsInitialized", &OrbitPredictConfig::IsInitialized)
            .def("Update", &OrbitPredictConfig::Update)
            .def("SetCenterStarType", &OrbitPredictConfig::SetCenterStarType)
            .def("GetCenterStarType", &OrbitPredictConfig::GetCenterStarType)
            .def("GetCenterStarGM", &OrbitPredictConfig::IsInitialized)
            .def("GetCenterStarJ2", &OrbitPredictConfig::GetCenterStarJ2)
            .def("GetCenterStarRadius", &OrbitPredictConfig::GetCenterStarRadius)
            .def("SetMJD_UTC", &OrbitPredictConfig::SetMJD_UTC)
            .def("GetMJD_UTC", &OrbitPredictConfig::GetMJD_UTC)
            .def("GetMJD_UT1", &OrbitPredictConfig::GetMJD_UT1)
            .def("GetMJD_TT", &OrbitPredictConfig::GetMJD_TT)
            .def("GetMJD_TAI", &OrbitPredictConfig::GetMJD_TAI)
            .def("GetTAI_UTC", &OrbitPredictConfig::GetTAI_UTC)
            .def("GetUT1_UTC", &OrbitPredictConfig::GetUT1_UTC)
            .def("GetTT_UTC", &OrbitPredictConfig::GetTT_UTC)
            .def("GetX_Pole", &OrbitPredictConfig::GetX_Pole)
            .def("GetY_Pole", &OrbitPredictConfig::GetY_Pole)
            .def("SetGravModelType", &OrbitPredictConfig::SetGravModelType)
            .def("GetGravModelType", &OrbitPredictConfig::GetGravModelType)
            .def("SetGravMaxDegree", &OrbitPredictConfig::SetGravMaxDegree)
            .def("GetGravMaxDegree", &OrbitPredictConfig::GetGravMaxDegree)
            .def("SetGravMaxOrder", &OrbitPredictConfig::SetGravMaxOrder)
            .def("GetGravMaxOrder", &OrbitPredictConfig::GetGravMaxOrder)
            .def("SetAtmosphereModelType", &OrbitPredictConfig::SetAtmosphereModelType)
            .def("GetAtmosphereModelType", &OrbitPredictConfig::GetAtmosphereModelType)
            .def("SetDragCoef", &OrbitPredictConfig::SetDragCoef)
            .def("GetDragCoef", &OrbitPredictConfig::GetDragCoef)
            .def("SetDragArea", &OrbitPredictConfig::SetDragArea)
            .def("GetDragArea", &OrbitPredictConfig::GetDragArea)
            .def("SetAverageF107", &OrbitPredictConfig::SetAverageF107)
            .def("GetAverageF107", &OrbitPredictConfig::GetAverageF107)
            .def("SetDailyF107", &OrbitPredictConfig::SetDailyF107)
            .def("GetDailyF107", &OrbitPredictConfig::GetDailyF107)
            .def("SetGeomagneticIndex", &OrbitPredictConfig::SetGeomagneticIndex)
            .def("GetGeomagneticIndex", &OrbitPredictConfig::GetGeomagneticIndex)
            .def("SetSRPCoef", &OrbitPredictConfig::SetSRPCoef)
            .def("GetSRPCoef", &OrbitPredictConfig::GetSRPCoef)
            .def("SetSRPArea", &OrbitPredictConfig::SetSRPArea)
            .def("GetSRPArea", &OrbitPredictConfig::GetSRPArea)
            .def("SetThirdBodySign", &OrbitPredictConfig::SetThirdBodySign)
            .def("GetThirdBodySign", &OrbitPredictConfig::GetThirdBodySign)
            .def("SetGeodeticCoordType", &OrbitPredictConfig::SetGeodeticCoordType)
            .def("GetGeodeticCoordType", &OrbitPredictConfig::GetGeodeticCoordType)
            .def("IsUseThirdBodyGravity", &OrbitPredictConfig::IsUseThirdBodyGravity)
            .def("IsUseSRP", &OrbitPredictConfig::IsUseSRP)
            .def("IsUseDrag", &OrbitPredictConfig::IsUseDrag)
            .def("IsUseNormalize", &OrbitPredictConfig::IsUseNormalize)
            .def("GetGravityModel", &OrbitPredictConfig::GetGravityModel)
            .def("GetGeodeticCoordSystem", &OrbitPredictConfig::GetGeodeticCoordSystem)
            .def("GetThirdBodyGravity", &OrbitPredictConfig::GetThirdBodyGravity)
            .def("GetAtmosphericDrag", &OrbitPredictConfig::GetAtmosphericDrag)
            .def("GetSolarRadPressure", &OrbitPredictConfig::GetSolarRadPressure);

    py::class_<OrbitPredict>(m, "OrbitPredict",
                                     R"pbdoc(
                                        The class of Orbit Prediction
                                     )pbdoc")
            .def(py::init<>())
            .def("OrbitStep", &OrbitPredict::OrbitStep,
                 R"pbdoc(
                 @Input
                 @Param  predictConfig       Orbit Prediction Parameters
                 @Param	pPropagator         Propagator
                 @In/Out
                 @Param	mass                kg
                 @Param	pos                 m
                 @Param	vel                 m/s
                 @Output
                 @Return adaptedStep         sec
                 )pbdoc");

    py::class_<TwoBodyOrbitPredict>(m, "TwoBodyOrbitPredict",
                                     R"pbdoc(
                                        The class of Tow Body Orbit Prediction
                                     )pbdoc")
            .def(py::init<>())
            .def("OrbitStep", &TwoBodyOrbitPredict::OrbitStep,
                 R"pbdoc(
                 @Input
                 @Param  predictConfig       Orbit Prediction Parameters
                 @Param	pPropagator         Propagator
                 @In/Out
                 @Param	mass                kg
                 @Param	pos                 m
                 @Param	vel                 m/s
                 @Output
                 @Return adaptedStep         sec
                 )pbdoc");

    py::class_<J2OrbitPredict>(m, "J2OrbitPredict",
                                     R"pbdoc(
                                        J2 Orbit Prediction
                                     )pbdoc")
            .def(py::init<>())
            .def("OrbitStep", &J2OrbitPredict::OrbitStep,
                 R"pbdoc(
                 @Input
                 @Param  predictConfig       Orbit Prediction Parameters
                 @Param	pPropagator         Propagator
                 @In/Out
                 @Param	mass                kg
                 @Param	pos                 m
                 @Param	vel                 m/s
                 @Output
                 @Return adaptedStep         sec
                 )pbdoc");

    py::class_<TLEOrbitPredict>(m, "TLEOrbitPredict",
                                     R"pbdoc(
                                        SGP4/SDP4 Orbit Prediction with TLE
                                     )pbdoc")
            .def(py::init<>())
            .def("SetTLEString", &TLEOrbitPredict::SetTLEString)
            .def("GetTLEString", &TLEOrbitPredict::GetTLEString)
            .def("OrbitStep", &TLEOrbitPredict::OrbitStep,
                 R"pbdoc(
                 @Input
                 @Param  Mjd      Modified Julian date of UTC
                 @Output
                 @Param	pos      m
                 @Param	vel      m/s
                 @Return
                 )pbdoc");


    //**************************SpAccess.h**************************
    py::class_<AccessAnalysis>(m, "AccessAnalysis",
                                     R"pbdoc(
                                        The class of SpaceDSL Access Analysis
                                     )pbdoc")
            .def(py::init<>())
            .def(py::init<Mission *>())
            .def("SetMission", &AccessAnalysis::SetMission)
            .def("GetMission", &AccessAnalysis::GetMission, py::return_value_policy::reference_internal)
            .def("CalTargetAccessData", py::overload_cast<int, const Target *, int , double>(&AccessAnalysis::CalTargetAccessData),
                 py::arg("vehicleID"), py::arg("target"),
                 py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      vehicleName         Vehicle Name
                 @Param      target              Point Target Object Point
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return     List<Start Mjd, End Mjd >
                 )pbdoc")
            .def("CalTargetAccessData", py::overload_cast<const Target *, int, double>(&AccessAnalysis::CalTargetAccessData),
                 py::arg("target"), py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      target              Point Target Object Point
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return     {Vehicle, List<Start Mjd, End Mjd >}
                 )pbdoc")
            .def("CalMissionAccessData", &AccessAnalysis::CalMissionAccessData,
                 py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return
                 )pbdoc");

    //**************************SpMission.h**************************
    py::class_<Mission, std::unique_ptr<Mission, py::nodelete>>(m, "Mission",
                                                                R"pbdoc(
                                                                   The class of SpaceDSL Mission
                                                                )pbdoc")
            .def(py::init<>())
            .def("Destory", &Mission::Destory)
            .def("InsertSpaceVehicle", &Mission::InsertSpaceVehicle, py::return_value_policy::reference_internal,
                 R"pbdoc(
                 @Input
                 const string &name,
                 const CalendarTime& initialEpoch,
                 const CartState& initialState,
                 const double initialMass,
                 const double dragCoef,
                 const double dragArea,
                 const double SRPCoef,
                 const double SRPArea
                 )pbdoc")
            .def("RemoveSpaceVehicle", &Mission::RemoveSpaceVehicle)
            .def("InsertFacility", &Mission::InsertFacility, py::return_value_policy::reference_internal,
                 R"pbdoc(
                 @Input
                 const string &name,
                 const double longitude,
                 const double latitude,
                 const double altitude,
                 const double minElevation
                 )pbdoc")
            .def("RemoveFacility", &Mission::RemoveFacility)
            .def("InsertTarget", &Mission::InsertTarget, py::return_value_policy::reference_internal,
                 R"pbdoc(
                 @Input
                 const string &name,
                 const Target::TargetType type
                 )pbdoc")
            .def("RemoveTarget", &Mission::RemoveTarget)
            .def("SetEnvironment", &Mission::SetEnvironment,
                 R"pbdoc(
                 @Input
                 const SolarSysStarType centerStarType,
                 const GravityModel::GravModelType gravModelType ,
                 const int maxDegree ,
                 const int maxOrder ,
                 const ThirdBodyGravitySign thirdBodyGravSign,
                 const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                 const AtmosphereModel::AtmosphereModelType atmModelType ,
                 const double f107A ,
                 const double f107,
                 VectorXd ap,
                 bool isUseDrag,
                 bool isUseSRP
                 )pbdoc")
            .def("SetPropagator", &Mission::SetPropagator,
                 py::arg("integMethodType"), py::arg("initialStep"),
                 py::arg("accuracy") = 0.0, py::arg("minStep") = 0.0, py::arg("maxStep") = 0.0,
                 py::arg("maxStepAttempts") = 0, py::arg("bStopIfAccuracyIsViolated") = true,
                 py::arg("isUseNormalize") = false,
                 R"pbdoc(
                 @Input
                 const IntegMethodType integMethodType,
                 const double initialStep,
                 const double accuracy = 0,
                 const double  minStep = 0,
                 const double  maxStep = 0,
                 const int maxStepAttempts = 0,
                 const bool bStopIfAccuracyIsViolated = true,
                 const bool isUseNormalize = false
                 )pbdoc")
            .def("SetMissionSequence", py::overload_cast<const CalendarTime&,  double>(&Mission::SetMissionSequence),
                 R"pbdoc(
                 @Input
                 const CalendarTime& initialEpochDate,
                 double durationSec
                 )pbdoc")
            .def("SetMissionSequence", py::overload_cast<const CalendarTime& ,  const CalendarTime&>(&Mission::SetMissionSequence),
                 R"pbdoc(
                 @Input
                 const CalendarTime& initialEpochDate,
                 const CalendarTime& terminationEpochDate,
                 double durationSec
                 )pbdoc")
            .def("SetMissionSequence", py::overload_cast<double,  double>(&Mission::SetMissionSequence),
                 R"pbdoc(
                 @Input
                 double initialEpochMjd,
                 double terminationEpochMjd
                 )pbdoc")
            .def("GetSpaceVehicleMap", &Mission::GetSpaceVehicleMap, py::return_value_policy::reference_internal)
            .def("GetSpaceVehicleNumber", &Mission::GetSpaceVehicleNumber)
            .def("GetFacilityMap", &Mission::GetFacilityMap, py::return_value_policy::reference_internal)
            .def("GetFacilityNumber", &Mission::GetFacilityNumber)
            .def("GetTargetMap", &Mission::GetTargetMap, py::return_value_policy::reference_internal)
            .def("GetTargetNumber", &Mission::GetTargetNumber)
            .def("GetEnvironment", &Mission::GetEnvironment, py::return_value_policy::reference_internal)
            .def("GetInitialPropagator", &Mission::GetInitialPropagator, py::return_value_policy::reference_internal)
            .def("GetStartEpochDate", &Mission::GetStartEpochDate)
            .def("GetEndEpochDate", &Mission::GetEndEpochDate)
            .def("GetTerminationEpochDate", &Mission::GetTerminationEpochDate)
            .def("GetStartEpoch", &Mission::GetStartEpoch)
            .def("GetEndEpoch", &Mission::GetEndEpoch)
            .def("GetTerminationEpoch", &Mission::GetTerminationEpoch)
            .def("GetDurationTime", &Mission::GetDurationTime)
            .def("GetAverageOrbitalPeriod", &Mission::GetAverageOrbitalPeriod)
            .def("CalTargetAccessData", py::overload_cast<int, const Target *, int, double>(&Mission::CalTargetAccessData),
                 py::arg("vehicleID"), py::arg("target"),
                 py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      vehicleName         Vehicle Name
                 @Param      target              Point Target Object Point
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return     List<Start Mjd, End Mjd >
                 )pbdoc")
            .def("CalTargetAccessData", py::overload_cast<const Target *, int, double>(&Mission::CalTargetAccessData),
                 py::arg("target"), py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      target              Point Target Object Point
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return     {Vehicle Name, List<Start Mjd, End Mjd >}
                 )pbdoc")
            .def("CalMissionAccessData", &Mission::CalMissionAccessData,
                 py::arg("order") = 5, py::arg("precision") = 0.01,
                 R"pbdoc(
                 @Input
                 @Param      order               Lagrange Polynomial Interpolation Order
                 @Param      precision           Iterative precision (unit: sec)
                 @Output
                 @Return
                 )pbdoc")
            .def("Start", &Mission::Start, py::arg("bIsMultThread") = false)
            .def("ClearProcessData", &Mission::ClearProcessData)
            .def("Clear", &Mission::Clear)
            .def("GetProcessDataMap", &Mission::GetProcessDataMap, py::return_value_policy::reference_internal)
            .def("GetAccessData", &Mission::GetAccessData, py::return_value_policy::reference_internal);

    //**************************SpCZMLScript.h**************************
    py::class_<CZMLScript>(m, "CZMLScript",
                                 R"pbdoc(
                                    The class of SpaceDSL CZML File Operation
                                )pbdoc")
            .def(py::init<>())
            .def("Initializer", &CZMLScript::Initializer,
                 py::arg("filePath"), py::arg("pMission"), py::arg("step") = 300,
                 R"pbdoc(
                 @Input
                 const string &filePath,
                 const Mission *pMission,
                 const double step = 300
                 )pbdoc")
            .def("WirteCZML", &CZMLScript::WirteCZML);

    //**************************SpUtils.h**************************
    py::register_exception<SPException>(m, "SPException");


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}


