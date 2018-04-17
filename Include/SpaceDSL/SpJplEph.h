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
*   SpJplEph.h
*
*   Purpose:
*
*       JPL planetary and lunar ephemerides
*
*
*   Last modified:
*
*   2018-03-20  Niu Zhiyong (1st edition)
*
*************************************************************************/

#ifndef SPJPLEPH_H
#define SPJPLEPH_H

#include "SpaceDSL_Global.h"

#include <Eigen/Core>

using namespace Eigen;

/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
    /********************************************************************
    **                                                                 **
    **   1 = mercury           8 = neptune                             **
    **   2 = venus             9 = pluto                               **
    **   3 = earth            10 = moon                                **
    **   4 = mars             11 = sun                                 **
    **   5 = jupiter          12 = solar-system barycenter             **
    **   6 = saturn           13 = earth-moon barycenter               **
    **   7 = uranus           14 = nutations (longitude and obliq)     **
    **                        15 = librations, if on eph. file         **
    **                        16 = lunar mantle omega_x,omega_y,omega_z**
    **                        17 = TT-TDB, if on eph. file             **
    **   If nutations are wanted, set ntarg = 14.                      **
    **   For librations, set ntarg = 15. set ncent= 0.                 **
    **   For TT-TDB,  set ntarg = 17.                                  **
    **   I've not actually seen an ntarg = 16 case yet.)               **
    *********************************************************************/
    enum SPACEDSL_API SolarSysStarType
    {
        E_NotDefinedStarType    =   0,
        E_Mercury               =   1,   ///< Mercury
        E_Venus                 =   2,   ///< Venus
        E_Earth                 =   3,   ///< Earth
        E_Mars                  =   4,   ///< Mars
        E_Jupiter               =   5,   ///< Jupiter
        E_Saturn                =   6,   ///< Saturn
        E_Uranus                =   7,   ///< Uranus
        E_Neptune               =   8,   ///< Neptune
        E_Pluto                 =   9,   ///< Pluto
        E_Moon                  =   10,  ///< Moon
        E_Sun                   =  11,   ///< Sun

        E_SolarSysBarycenter    =  12,   ///< Solar System barycenter
        E_EarthMoonBaryCenter   =  13,   ///< Earth Moon barycenter
        E_JplNutation           =  14,   ///< Nutations
        E_JplLnrLibration       =  15,   ///< Lunar Librations
        E_JplLunarMantle        =  16,   ///< Lunar Mantle omega_x,omega_y,omega_z
        E_JplTT_TDB             =  17,   ///< TT-TDB, if on eph. file

        E_SelfDefinedStarType   =   99,
    };

    /*************************************************
     * Class type: The class of Read JPL Ephemeris
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    class SPACEDSL_API JplEphemeris
    {
    public:
        JplEphemeris();
        ~JplEphemeris();

        double      GetJplEphemerisStartJD();
        double      GetJplEphemerisEndJD();
        double      GetJplEphemerisStep();

    public:
        //********************************************************************
        ///
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	type
        ///                 JPL_EPHEM_START_JD               0
        ///                 JPL_EPHEM_END_JD                 8
        ///                 JPL_EPHEM_STEP                  16
        ///                 JPL_EPHEM_N_CONSTANTS           24
        ///                 JPL_EPHEM_AU_IN_KM              28
        ///                 JPL_EPHEM_EARTH_MOON_RATIO      36
        ///                 JPL_EPHEM_IPT_ARRAY             44
        ///                 JPL_EPHEM_EPHEMERIS_VERSION    224
        ///                 JPL_EPHEM_KERNEL_SIZE          228
        ///                 JPL_EPHEM_KERNEL_RECORD_SIZE   232
        ///                 JPL_EPHEM_KERNEL_NCOEFF        236
        ///                 JPL_EPHEM_KERNEL_SWAP_BYTES    240
        /// @Retrun
        //********************************************************************
        double GetJplEphData( int type);

        //********************************************************************
        /// computes the position of the target object with respect to
        ///   reference object at the specified julian ephemeris date.
        ///   The target and reference codes should be chosen from
        ///   the JPL planetary ephemeris data request codes.
        /// @Author	Niu Zhiyong
        /// @Date	2018-03-20
        /// @Input
        /// @Param	jd				Julian_TT
        /// @Param	target			Star Needs to Calculate the Velocity and Position
        /// @Param	center          Reference Body
        /// @Output
        /// @Param	pos				(m)
        /// @Param	vel				(m/s)
        //********************************************************************
        void GetJplEphemeris (
                    double jd,
                    SolarSysStarType target,
                    SolarSysStarType center,
                    Vector3d& pos);

        void GetJplEphemeris (
                    double jd,
                    SolarSysStarType target,
                    SolarSysStarType center,
                    Vector3d& pos,
                    Vector3d& vel);

    private:

        void        *m_pJplEph;         ///< a Pointer to The jpl_eph_data Structure
        double      m_EphStartJD;       ///< The start dates are given in Julian Ephemeris Date (JD format of TDT)
        double      m_EphEndJD;         ///< The stop dates are given in Julian Ephemeris Date (JD format of TDT)
        double      m_EphStep;

    };
	
}

#endif //SPJPLEPH_H
