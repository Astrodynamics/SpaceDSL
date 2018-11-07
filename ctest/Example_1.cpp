/********************************************************************/
/// Referring to Niu ZhiYong experiments, this example calls
///  void CZMLScript:: WirteCZML (vector < double > data_MJD_POS, vector < double > data_MJD_Vel, string vehicl_name)
/// @Author	xiaogongwei
/// @Date	2018-11-06
/********************************************************************/

#include <iostream>
#include "SpaceDSL/SpaceDSL.h"

using namespace std;
using namespace SpaceDSL;

int main()
{
    cout<<"SpaceDSL test begin!"<<endl;

    GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
    GeodeticCoord LLA;// BLH CoordSystem
    OrbitPredictConfig predictConfig;
    OrbitPredict orbit;
    Vector3d pos,vel;
    double  mass;

    // set pos, speed and mass of vehicle
    pos(0) = -5.04649e+06; pos(1) = -3.53951e+06; pos(2) = -2.44795e+06; // m
    vel(0) = 4954.67; vel(1) = -4008.83; vel(2) = -4417.73;// m/s
    mass = 1000; // kg
    // set  ThirdBody
    ThirdBodyGravitySign thirdGravSign;
    thirdGravSign.bIsUseSunGrav = true;
    thirdGravSign.bIsUseMoonGrav = true;
    // set init MJD
    UTCCalTime initial_time     (2018,1,4,0,0,0);
    UTCCalTime termination_time(2018,1,4,0,30,0);
    double Mjd_Start = 0, MJD_end = 0, Mjd_UTC = 0;
    Mjd_Start = CalendarTimeToMjd(initial_time);// transfer start UTC to MJD
    MJD_end = CalendarTimeToMjd(termination_time);// transfer end UTC to MJD

    Mjd_UTC = Mjd_Start;
    // set Geomagnetic index
    double ap[7];
    for (int i = 0; i < 7; ++i)
        ap[i] = 0;
    ap[0] = 14.9186481659685;
    //init predict configure
    predictConfig.Initializer(Mjd_Start,SolarSysStarType::E_Earth,// use Mjd_Start is right?
                              false,
                              GravityModel::GravModelType::E_EGM08Model,
                              20 ,// m_pEnvironment->GetGravMaxDegree()
                              20 ,// m_pEnvironment->GetGravMaxOrder()
                              thirdGravSign, // m_pEnvironment->GetThirdBodySign()
                              GeodeticCoordSystem::GeodeticCoordType::E_WGS84System, // m_pEnvironment->GetGeodeticCoordType(),
                              AtmosphereModel::AtmosphereModelType::E_NRLMSISE00Atmosphere,// m_pEnvironment->GetAtmosphereModelType()
                              2.2 , // pVehicle->GetDragCoef() // [1.5~3.0] sphere:2.0 Nonspheroid:2.0~2.3
                              10 ,// pVehicle->GetDragArea()
                              150 ,// m_pEnvironment->GetAverageF107()
                              150 ,// m_pEnvironment->GetDailyF107()
                              ap, //m_pEnvironment->GetGeomagneticIndex()
                              1.21 ,// pVehicle->GetSRPCoef()// [1.21,1.30,1.88]
                              10 ,// pVehicle->GetSRPArea()
                              true , // m_pEnvironment->GetIsUseDrag()
                              true // m_pEnvironment->GetIsUseSRP()
                              );

    LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);
    // set predict time
    double step = 60;// set predict time interval
    int predict_epoch = (int)((MJD_end - Mjd_Start)*DayToSec)/step + 1;

    cout.width(30);
    cout.precision(15);
    cout << " " <<  endl;
    vector<double> data_MJD_POS, data_MJD_Vel;// store MJD and POS
    // store current MJD and pos
    data_MJD_POS.push_back(Mjd_UTC);
    data_MJD_POS.push_back(pos(0));
    data_MJD_POS.push_back(pos(1));
    data_MJD_POS.push_back(pos(2));
    // store current MJD and Vel
    data_MJD_Vel.push_back(Mjd_UTC);
    data_MJD_Vel.push_back(vel(0));
    data_MJD_Vel.push_back(vel(1));
    data_MJD_Vel.push_back(vel(2));

    for (int i = 0; i < predict_epoch; ++i)
    {
        cout << "Mjd_UTC: " << Mjd_UTC << endl;
        cout << "Epoch time: " << i*step <<endl;
        cout << "Pos: " << pos(0) << ", "<< pos(1) << ", " << pos(2) << endl;
        cout << endl;
        // predict next pos
        predictConfig.SetMJD_UTC(Mjd_UTC);
        orbit.OrbitStep(predictConfig,step, E_RungeKutta4, mass, pos, vel);
        Mjd_UTC = Mjd_Start + (i+1) * step/DayToSec;
        LLA = GEO.GetGeodeticCoord(pos, Mjd_UTC);// this is BLH
        // store current MJD and pos
        data_MJD_POS.push_back(Mjd_UTC);
        data_MJD_POS.push_back(pos(0));
        data_MJD_POS.push_back(pos(1));
        data_MJD_POS.push_back(pos(2));
        // store current MJD and Vel
        data_MJD_Vel.push_back(Mjd_UTC);
        data_MJD_Vel.push_back(vel(0));
        data_MJD_Vel.push_back(vel(1));
        data_MJD_Vel.push_back(vel(2));
    }

    CZMLScript  script;
    script.Initializer("Xiao_Example.czml");
    script.WirteCZML(data_MJD_POS, data_MJD_Vel, "Grace_A");

    cout<<"SpaceDSL test End!"<<endl;
    return 0;
}

