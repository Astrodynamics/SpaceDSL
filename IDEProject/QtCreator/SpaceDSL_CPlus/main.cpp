#include <iostream>


#include "SpaceDSL/Include/SpaceDSL.h"

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
    UTCCalTime termination_time(2018,1,4,1,0,0);
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
                              2.2 , // pVehicle->GetDragCoef()
                              10 ,// pVehicle->GetDragArea()
                              150 ,// m_pEnvironment->GetAverageF107()
                              150 ,// m_pEnvironment->GetDailyF107()
                              ap, //m_pEnvironment->GetGeomagneticIndex()
                              1 ,// pVehicle->GetSRPCoef()
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


// other test Data
int main1(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {   /// Initial Data
        UTCCalTime initial_time     (2018,1,4,0,0,0);
        UTCCalTime termination_time(2018,1,4,1,0,0);

        string vehicle_name1 = "The First Vehicle";// h = 257km
        CartState vehicle1_cart0(-5.04649e+06, -3.53951e+06, -2.44795e+06,
                                    4954.67 , -4008.83, -4417.73 );
        double vehicle1_mass = 1000.0;

//        string vehicle_name2 = "The Second Vehicle"; // h = 1065km
//        CartState vehicle2_cart0(-5.65484e+06, -3.96619e+06, -2.74305e+06,
//                                    4680.57 , -3787.06, -4173.34 );
//        double vehicle2_mass = 1500.0;

        /// Mission Start
        Mission *pMission = new Mission();
        pMission->InsertSpaceVehicle(vehicle_name1,initial_time,vehicle1_cart0,vehicle1_mass, 2.2, 10, 1.0, 10);
//        pMission->InsertSpaceVehicle(vehicle_name2,initial_time,vehicle2_cart0,vehicle2_mass, 2.2, 20, 1.0, 20);
        ThirdBodyGravitySign thirdGravSign;
        thirdGravSign.bIsUseSunGrav = true;
        thirdGravSign.bIsUseMoonGrav = true;
        double ap[7];
        for (int i = 0; i < 7; ++i)
            ap[i] = 0;
        ap[0] = 14.9186481659685;
        pMission->SetEnvironment(E_Earth, GravityModel::GravModelType::E_EGM08Model,
                                 20 , 20, thirdGravSign,
                                 GeodeticCoordSystem::GeodeticCoordType::E_WGS84System,
                                AtmosphereModel::AtmosphereModelType::E_NRLMSISE00Atmosphere,
                                 150,150,ap,
                                 true, true);
        pMission->SetPropagator(E_RungeKutta4, 60, 1.0, 1, 120, 30, true, false);
        pMission->SetMissionSequence(initial_time, termination_time);
        pMission->Start(false);

        /// CZML File Wirte;
        CZMLScript  script;
        script.Initializer("TestData.czml", pMission);
        script.WirteCZML();

    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
