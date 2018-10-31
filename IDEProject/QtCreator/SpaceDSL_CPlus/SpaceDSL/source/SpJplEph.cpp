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
*   SpJplEph.cpp
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

#include "../Include/SpJplEph.h"



namespace SpaceDSL {

    /*************************************************
     * Class type: The class of Read JPL Ephemeris
     * Author: Niu ZhiYong
     * Date:2018-03-20
     * Description:
    **************************************************/
    JplEphemeris::JplEphemeris()
    {
        char nams[2000][6];
        double vals[2000];
        m_pJplEph = jpl_init_ephemeris("./astrodata/Ephemeris/DE436.1950.2050", nams, vals);
        int result = jpl_init_error_code();
        switch (result)
        {
        case 0:
            m_EphStartJD = jpl_get_double(m_pJplEph, JPL_EPHEM_START_JD);
            m_EphEndJD = jpl_get_double(m_pJplEph, JPL_EPHEM_END_JD);
            m_EphStep = jpl_get_double(m_pJplEph, JPL_EPHEM_STEP);
            break;
        case -1:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FILE_NOT_FOUND");
            break;
        case -2:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FSEEK_FAILED");
            break;
        case -3:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FREAD_FAILED");
            break;
        case -4:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FREAD2_FAILED");
            break;
        case -5:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FILE_CORRUPT");
            break;
        case -6:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_MEMORY_FAILURE");
            break;
        case -7:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FREAD3_FAILED");
            break;
        case -8:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FREAD4_FAILED");
            break;
        case -9:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_NOT_CALLED");
            break;
        case -10:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "JplEphemeris Init: JPL_INIT_FREAD5_FAILED");
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "JplEphemeris: Result is out of Known Situation");
            break;
        }

    }

    JplEphemeris::~JplEphemeris()
    {
        jpl_close_ephemeris(m_pJplEph);
    }

    double JplEphemeris::GetJplEphemerisStartJD()
    {
        return m_EphStartJD;
    }

    double JplEphemeris::GetJplEphemerisEndJD()
    {
        return m_EphEndJD;
    }

    double JplEphemeris::GetJplEphemerisStep()
    {
        return m_EphStep;
    }

    double JplEphemeris::GetJplEphData(int type)
    {
        return jpl_get_double(m_pJplEph, type);
    }

    void JplEphemeris::GetJplEphemeris(double jd, SolarSysStarType target, SolarSysStarType center, Vector3d &pos)
    {
        double r_p[6];
        int result = 0;
        result = jpl_pleph(m_pJplEph, jd, target, center, r_p, 0);
        switch (result)
        {
        case 0:
            pos(0) = r_p[0] * AU;
            pos(1) = r_p[1] * AU;
            pos(2) = r_p[2] * AU;
            break;
        case -1:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris: JPL_EPH_OUTSIDE_RANGE");
            break;
        case -2:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_READ_ERROR");
            break;
        case -3:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_QUANTITY_NOT_IN_EPHEMERIS");
            break;
        case -5:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_INVALID_INDEX");
            break;
        case -6:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_FSEEK_ERROR");
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "JplEphemeris: Result is out of Known Situation");
            break;
        }

    }

    void JplEphemeris::GetJplEphemeris(double jd, SolarSysStarType target, SolarSysStarType center, Vector3d &pos, Vector3d &vel)
    {
        double r_p[6];
        int result = 0;
        result = jpl_pleph(m_pJplEph, jd, target, center, r_p, 1);
        switch (result)
        {
        case 0:
            pos(0) = r_p[0] * AU;
            pos(1) = r_p[1] * AU;
            pos(2) = r_p[2] * AU;

            vel(0) = r_p[3] * AUPerDay;
            vel(1) = r_p[4] * AUPerDay;
            vel(2) = r_p[5] * AUPerDay;
            break;
        case -1:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris: JPL_EPH_OUTSIDE_RANGE");
            break;
        case -2:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_READ_ERROR");
            break;
        case -3:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_QUANTITY_NOT_IN_EPHEMERIS");
            break;
        case -5:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_INVALID_INDEX");
            break;
        case -6:
            throw SPException(__FILE__, __FUNCTION__, __LINE__,
                  "GetJplEphemeris:JPL_EPH_FSEEK_ERROR");
            break;
        default:
            throw SPException(__FILE__, __FUNCTION__, __LINE__, "JplEphemeris: Result is out of Known Situation");
            break;
        }
    }

	
	
}

