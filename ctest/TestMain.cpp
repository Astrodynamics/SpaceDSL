#include <iostream>

#include "SpaceDSL/SpaceDSL.h"

using namespace std;
using namespace SpaceDSL;

int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {   /// Initial Data
        UTCCalTime initial_time     (2018,1,4,16,58,11.1);
        UTCCalTime termination_time (2018,1,5,16,58,11.1);

        string vehicle_name1 = "The First Vehicle";// h = 257km
        CartState vehicle1_cart0(-5.04649e+06, -3.53951e+06, -2.44795e+06,
                                    4954.67 , -4008.83, -4417.73 );
        double vehicle1_mass = 1000.0;

        string vehicle_name2 = "The Second Vehicle"; // h = 1065km
               CartState vehicle2_cart0(-5.65484e+06, -3.96619e+06, -2.74305e+06,
                                    4680.57 , -3787.06, -4173.34 );
        double vehicle2_mass = 1500.0;

        /// Mission Start
        Mission *pMission = new Mission();
        pMission->InsertSpaceVehicle(vehicle_name1,initial_time,vehicle1_cart0,vehicle1_mass, 2.2, 10, 1.0, 10);
        pMission->InsertSpaceVehicle(vehicle_name2,initial_time,vehicle2_cart0,vehicle2_mass, 2.2, 20, 1.0, 20);
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
        pMission->SetPropagator(E_RungeKutta4, 60);
        //pMission->SetPropagator(E_RungeKutta78, 60, 0.01, 1, 120, 100);
        pMission->SetMissionSequence(initial_time, termination_time);
        pMission->Start(false);

        cout<<"Calculation Finished!"<<endl;

        /// CZML File Wirte;
        CZMLScript  script;
        script.Initializer("TestData.czml", pMission);
        script.WirteCZML();
        cout<<"CZML Output Finished!"<<endl;
        pMission->Destory();

    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
