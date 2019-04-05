/************************************************************************
* Copyright (C) 2018 Niu ZhiYong
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
* Date:2018-03-20
* Description:
*   SpConst.h
*
*   Purpose:
*
*       Define The Constants in Library
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/
#ifndef SPCONST_H
#define SPCONST_H

// Standard C++ version
#include <cmath>
#include <limits>

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
//
// Machine accuracy
//
const double EPS = std::numeric_limits<double>::epsilon();

//
// Constants representing various multiples of PI.
//
const double PI          = 3.141592653589793238462643383279502884197169399375105;
const double TwoPI       = 6.283185307179586476925286766558;		///< PI+PI
const double FourPI      = 12.566370614359172953850573533116;		///< 4*PI
const double HalfPI      = 1.5707963267948966192313216916395;		///< 0.5*PI
const double ThirdPI     = 1.047197551196597746154214461093;		///< PI/3
const double QuarterPI   = 0.78539816339744830961566084581975;		///< PI/4
const double SqrPI       = 9.869604401089358618834490999873;		///< PI*PI, PI squared

///
/// Date Conversion Constants
///
const double MJDOffset  = 2400000.5;
const double TTMinusTAI = 32.184;


///
/// Coordinate System Epochs
///
const double B1950Epoch = 2433282.4234591;
const double J2000Epoch = 2451545.0;			//2000-01-01 12:00:00 TDB
const double MJD_J2000  = 51544.5;              // Modif. Julian Date of J2000.0


///
/// Earth_Motion_Constants
///
const double EarthSiderealDay  = 86164.09054;
const double EarthSiderealYear = 365.25636;

//===================================
// Data for physical constant systems
//===================================

// 
//           Item                   Value                      Comments
//
const double Grav           	= 6.673e-11;				///< Constant of gravitation [m^3/kg*s^2] 
const double GM_Earth         	= 3.986004418e14;			///< Geocentric gravitation constant (WGS84) [m^3/s^2]
const double AU             	= 1.49597870700e11;			///< Astronomical unit in meters (IAU 2009)[m]
const double EarthRadius    	= 6378137.0;				///< Equatorial radius of the Earth (WGS84)[m]
const double EarthMinRadius 	= 6.35675231424E6;			///< (WGS84)[m]
const double EarthFlatFact  	= 3.35281066475E-3;			///< Flattening factor of the Earth (WGS84)
const double LightSpeed     	= 2.99792458e8;				///< speed of light (IAU 1976 value) [m/s]		
const double EarthAngVel	   	= 7.2921151467e-5;			///< Nominal mean angular velocity of the Earth (WGS84) [rad/s](TwoPI/86164.0919)
const double EarthMeanMotion	= 1.9910643985790994404796035026544e-7;	///< (TwoPI/365.2420897/86400) rad/sec
const double EarthRPSDay        = 1.00273790934;            ///< Earth Rotations Per Siderial Day

const double Earth_J2          	= 1.0826261e-3;			///< Earth Perturbation J2 Term
const double Earth_J3          	= -2.54e-6;				///< Earth Perturbation J3 Term
const double Earth_J4          	= -1.61e-6;				///< Earth Perturbation J4 Term

//
//WGS84 constant
//
const double WGS84RE     		= 6.378137E6;
const double WGS84RP     		= 6.35675231424E6;
const double WGS84F      		= 3.35281066475E-3;
const double WGS84GM     		= 3.986004418E14;
const double WGS84WR     		= 7.2921158553E-5;
const double WGS84WR_INERTIAL 	= 7.2921151467E-5;
const double WGS84SRE    		= 6.96E8;			//IAU76


//
// moon parameter 
//
const double GM_Moon            = 4.9027949e12;		///< moon gravitation constant [m^3/s^2]
const double MoonRadius         = 1738200.0;		///< Equatorial radius of the Moon [m]
const double MoonMinRadius      = 1738200.0;
const double Moon_J2            = 0.2027e-3;

//
// Jupiter parameter
//
const double GM_Jupiter         = 1.26712000000000e+017;
const double JupiterRadius      = 7.14920000000000e+007;
const double JupiterMinRadius   = 6.68540000000000e+007;
const double Jupiter_J2         = 0.01475;

//
// Mars parameter
//
const double GM_Mars            = 4.28282868534000e+013;
const double MarsRadius         = 3.39700000000000e+006;
const double MarsMinRadius      = 3.37500000000000e+006;
const double Mars_J2            = 1.96045e-3;


//
// Mercury parameter
//
const double GM_Mercury         = 2.20320800000000e+013;
const double MercuryRadius      = 2.43970000000000e+006;
const double MercuryMinRadius   = 2.43970000000000e+006;
const double Mercury_J2         = 50.3e-6;

//
// Neptune parameter 
//
const double GM_Neptune         = 6.87130000000000e+015;
const double NeptuneRadius      = 2.52690000000000e+007;
const double NeptuneMinRadius   = 2.48000000000000e+007;
const double Neptune_J2          = 3.411e-3;
//
// Pluto parameter
//
const double GM_Pluto           = 1.00907600000000e+012;
const double PlutoRadius        = 1.16200000000000e+006;
const double PlutoMinRadius     = 1.16200000000000e+006;
const double Pluto_J2           = 0.0;
//
// Saturn parameter
//
const double GM_Saturn          = 3.79340000000000e+016;
const double SaturnRadius       = 6.02680000000000e+007;
const double SaturnMinRadius    = 5.43640000000000e+007;
const double Saturn_J2          = 0.016298;

//
// Sun parameter
//
const double GM_Sun             = 1.327122E20;				///< Heliocentric gravitation constant [m^3/s^2]
const double SunRadius          = 695990000.0;				///< Equatorial radius of the Sun [m], Seidelmann 1992
const double SunMinRadius       = 695990000.0;
const double SolarRadPreAtAU    = 4.560E-6;                 ///< Solar Radiation pressure at 1 AU [N/m^2] (~1367 W/m^2); IERS 96
const double Sun_J2             = 0.0;
//
// Uranus parameter
//
const double GM_Uranus          = 5.80320000000000e+015;
const double UranusRadius       = 2.55590000000000e+007;
const double UranusMinRadius    = 2.49730000000000e+007;
const double Uranus_J2          = 3.34343e-3;

//
// Venus parameter
//
const double GM_Venus           = 3.24858800000000e+014;
const double VenusRadius        = 6.05190000000000e+006;
const double VenusMinRadius     = 6.05190000000000e+006;
const double Venus_J2           = 4.458e-6;

//===================================
// Unit conversion constants
//===================================

//
// Constants:   Distance_Dimension_Constants
//
const double MeterToFeet       = 3.28083989501323;
const double MeterToStatMi     = 6.2137119223733e-04;	///< Statute Miles
const double MeterToNautMi     = 5.3995680345572e-04;	///< Nautical Miles
const double MeterToKilometer  = 1.0e-03;
const double MeterToKiloFeet   = 3.28083989501323e-03;

const double FeetToMeter       = 3.048e-01;
const double StatMiToMeter     = 1.609344e03;
const double NautMiToMeter     = 1.852e03;
const double KilometerToMeter  = 1.0e03;
const double KiloFeetToMeter   = 3.048e02;


//
// Constants:   Small_Distance_Dimension_Constants
//
const double MeterToInch       = 3.28083989501323*12.0;		///< MeterToFeet*12.0
const double MeterToCentimeter = 1.0e+02;
const double MeterToMillimeter = 1.0e+03;
const double MeterToMicron     = 1.0e+06;
const double MeterToNanometer  = 1.0e+09;

const double InchToMeter       = 3.048e-01/12.0;			///< FeetToMeter/12.0
const double CentimeterToMeter = 1.e-02;
const double MillimeterToMeter = 1.e-03;
const double MicronToMeter     = 1.e-06;
const double NanometerToMeter  = 1.e-09;


//  
// Constants:   Time_Dimension_Constants 
//
const double SecToMin          = 0.0166666666666666667;
const double SecToHour         = 0.000277777777777777778;
const double SecToDay          = 0.0000115740740740740741;
const double SecToCenday       = 0.0000115740740740740741*100;					///< (SecToDay * 100)
const double SecToEarthTU      = sqrt(3.986004418e14/pow(6.378137e06,3));		///< sqrt(EarthGrav/pow(ER,3)), WGS84
const double SecToSunTU        = sqrt(1.32712442076e20/pow(1.49597870e11,3));	///< sqrt(SunGrav/pow(AU,3))

const double MinToSec          = 60.0;
const double MinToHour         = 0.0166666666666666667;
const double MinToDay          = 0.000694444444444444444;

const double HourToSec         = 3600.0;
const double HourToMin         = 60.0;
const double HourToDay         = 0.0416666666666666667;

const double DayToSec          = 8.64e04;
const double DayToMin          = 1440.0;
const double DayToHour         = 24.0;

const double CendayToSec       = 864.0;
const double EarthTUToSec      = sqrt(pow(6.378137e06,3)/3.986004418e14);		///< sqrt(pow(ER,3)/EarthGrav), WGS84
const double SunTUToSec        = sqrt(pow(1.49597870e11,3)/1.32712442076e20);	///< sqrt(pow(AU,3)/SunGrav)
const double AUPerDay          = AU/DayToSec;                                   ///< AU/Day (IAU 2009)[m/s]

//
// Constants:   SmallTime_Dimension_Constants 
//
const double MilliSecToSec     = 1.0e-03;
const double MicroSecToSec     = 1.0e-06;
const double NanoSecToSec      = 1.0e-09;
const double PicoSecToSec      = 1.0e-12;

const double SecToMilliSec     = 1.0e+03;
const double SecToMicroSec     = 1.0e+06;
const double SecToNanoSec      = 1.0e+09;
const double SecToPicoSec      = 1.0e+12;


//
// Constants:   Angle_Dimension_Constants 
//
const double RadToDeg          = 57.295779513082320876798154814105;	///< 180.0/PI
const double RadToArcSec       = 206264.80624709635515647335733078;	///< RadToDeg*3600.0
const double RadToArcMin       = 57.2957795130823208767*60.0;		///< RadToDeg*60.0
const double RadToSecArc       = 206264.8062470964/15;				///< RadToArcSec/15

const double DegToRad          = 0.017453292519943295769236907684886;	///< PI/180.0
const double ArcSecToRad       = 4.8481368110953599358991410235795e-6;	///< DegToRad/3600.0
const double ArcMinToRad       = 2.9088820866572159615394846141477e-4;	///< DegToRad/60.0
const double SecArcToRad       = 7.2722052166430399038487115353692e-5;	///< ArcSecToRad*15
const double MinArcToRad       = 0.0043633231299858239423092269212215;


//
// Constants:   Mass_Dimension_Constants
//
const double KilogramToSlug    = 6.85248397168e-02;
const double KilogramToPound   = 2.2046226;

const double SlugToKilogram    = 14.5932482897;
const double PoundToKilogram   = 4.5359237e-01;


//
// Constants:   Power_Dimension_Constants 
//
const double WattToMilliwatt   = 1.0e+03;
const double WattToKilowatt    = 1.0e-03;  
const double WattToMegawatt    = 1.0e-06;
const double WattToGigawatt    = 1.0e-09;

const double MilliwattToWatt   = 1.0e-03;   
const double KilowattToWatt    = 1.0e+03;
const double MegawattToWatt    = 1.0e+06;
const double GigawattToWatt    = 1.0e+09;  


//
// Constants:   Frequency_Dimension_Constants  
//
const double HertzToKilohertz  = 1.0e-03;   
const double HertzToMegahertz  = 1.0e-06;
const double HertzToGigahertz  = 1.0e-09;
const double HertzToTerahertz  = 1.0e-12;

const double KilohertzToHertz  = 1.0e+03;
const double MegahertzToHertz  = 1.0e+06;
const double GigahertzToHertz  = 1.0e+09;   
const double TerahertzToHertz  = 1.0e+12;   

}
#endif //SPCONST_H
