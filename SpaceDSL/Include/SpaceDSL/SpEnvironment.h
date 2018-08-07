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
* Date:2018-07-30
* Description:
*   SpEnvironment.h
*
*   Purpose:
*
*       Space Environment Configuration Class
*
*
*   Last modified:
*
*   2018-07-30  Niu Zhiyong (1st edition)
*
*************************************************************************/
#ifndef SPENVIRONMENT_H
#define SPENVIRONMENT_H

#include "SpaceDSL_Global.h"
#include "SpGravity.h"
#include "SpJplEph.h"
#include "SpAtmosphere.h"
#include "SpCoordSystem.h"

using namespace std;
/// All the functions are in the namespace SpaceDSL
///
namespace SpaceDSL {
	
    /*************************************************
     * Class type: Space Environment Configuration Class
     * Author: Niu ZhiYong
     * Date:2018-07-30
     * Description:
    **************************************************/
    class SPACEDSL_API Environment
    {
    public:
        explicit Environment();
        Environment(const SolarSysStarType centerStarType, const GravityModel::GravModelType gravModelType ,
                    const int maxDegree , const int maxOrder , const ThirdBodyGravitySign thirdBodyGravSign,
                    const GeodeticCoordSystem::GeodeticCoordType geodeticType ,
                    const AtmosphereModel::AtmosphereModelType atmModelType ,
                    const double f107A , const double f107, double *ap);
        ~Environment();

    public:
        inline  void    SetCenterStarType(const SolarSysStarType type)            { m_CenterStarType = type; }

        inline  void    SetGravModelType(const GravityModel::GravModelType type)  { m_GravModelType = type; }

        inline  void    SetGravMaxDegree(const int maxDegree)                     { m_MaxDegree = maxDegree; }

        inline  void    SetGravMaxOrder(const int maxOrder)                       { m_MaxOrder = maxOrder; }

        inline  void    SetThirdBodySign(const ThirdBodyGravitySign sign)         { m_ThirdBodySign = sign; }

        inline  void    SetGeodeticCoordType(const GeodeticCoordSystem::GeodeticCoordType type)
                                                                                    { m_GeodeticCoordType = type; }

        inline  void    SetAtmosphereModelType(const AtmosphereModel::AtmosphereModelType type)
                                                                                    { m_AtmModelType = type; }

        inline  void    SetAverageF107(const double f107A)                          { m_F107A = f107A; }

        inline  void    SetDailyF107(const double f107)                             { m_F107 = f107; }

        inline  void    SetGeomagneticIndex(double *ap)                             { m_Ap = ap; }

        inline SolarSysStarType                 GetCenterStarType() const       { return m_CenterStarType; }

        inline  GravityModel::GravModelType     GetGravModelType() const        { return m_GravModelType; }

        inline  int                             GetGravMaxDegree() const        { return m_MaxDegree; }

        inline  int                             GetGravMaxOrder() const         { return m_MaxOrder; }

        inline  ThirdBodyGravitySign            GetThirdBodySign() const        { return m_ThirdBodySign; }

        inline  GeodeticCoordSystem::GeodeticCoordType
                                                GetGeodeticCoordType() const    { return m_GeodeticCoordType; }

        inline  AtmosphereModel::AtmosphereModelType
                                                GetAtmosphereModelType() const  { return m_AtmModelType; }

        inline  double                          GetAverageF107() const          { return m_F107A; }

        inline  double                          GetDailyF107() const            { return m_F107; }

        inline  double                          *GetGeomagneticIndex() const    { return m_Ap; }


    //
    // Attribute.
    //
    protected:
        SolarSysStarType            m_CenterStarType;

        //Third Body Gravity Sign
        ThirdBodyGravitySign        m_ThirdBodySign;

        //Gravity Parameters
        GravityModel::GravModelType m_GravModelType;	///< Gravity Model
        int                         m_MaxDegree;		///< Gravity Degree[n]
        int                         m_MaxOrder;			///< Degree Order  [m]

        GeodeticCoordSystem::GeodeticCoordType
                                    m_GeodeticCoordType;
        //Atmosphere Parameters
        AtmosphereModel::AtmosphereModelType
                                    m_AtmModelType;
        double                      m_F107A;            ///< average F10.7
        double                      m_F107;             ///< daily F10.7
        double                      *m_Ap;              ///< geomagnetic index



    };
}

#endif //SPENVIRONMENT_H
