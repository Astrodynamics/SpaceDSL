#include <iostream>

#include "SpaceDSL/SpaceDSL.h"
#include <fstream>
using namespace std;
using namespace SpaceDSL;
class MyTread : public SpThread
{
public:
    MyTread(){}
    ~MyTread() override {}

public:

    void Run() override
    {
        ofstream fileStream,fileLLA;
        fileStream.open("CartState.txt",ios::ate);
        fileLLA.open("LLAState.txt",ios::ate);
        UTCCalTime time(2018,1,4,16,58,11.1);

        GeodeticCoordSystem GEO(GeodeticCoordSystem::GeodeticCoordType::E_WGS84System);
        GeodeticCoord LLA;

        double mass0 = 1000.0;
        /// h = 257km
        //CartState cart0(-5.04649e+06, -3.53951e+06, -2.44795e+06,
        //                4954.67 , -4008.83, -4417.73 );

        /// h = 330km
        //CartState cart0(-5107324.219684929600, -3582177.314118019800, -2477461.707380509900,
        //                4925.070866580500, -3984.883326371030, -4391.339344033090);

        /// h = 566km
        CartState cart0(-5.27462e+06 , -3.69952e+06, -2.55861e+06,
                       4846.34 , -3921.18, -4321.14 );
        /// h = 1065km
        //CartState cart0(-5.65484e+06, -3.96619e+06, -2.74305e+06,
        //               4680.57 , -3787.06, -4173.34 );


        double Mjd_UTC0 = CalendarTimeToMjd(time);

        double step = 10.0;
        double Mjd_UTC = Mjd_UTC0;

        OrbitPredictConfig::ThirdBodyGravitySign thirdGravSign;
        thirdGravSign.bIsUseSunGrav = true;
        thirdGravSign.bIsUseMoonGrav = true;

        OrbitPredictConfig preConfig1,preConfig2,preConfig_twoBody;
        double ap[7];
        for (int i = 0; i < 7; ++i)
            ap[i] = 0;
        ap[0] = 14.9186481659685;
        preConfig1.Initializer(Mjd_UTC0, E_Earth, false,
                              GravityModel::GravModelType::E_EGM08Model,20 , 20, thirdGravSign,
                               GeodeticCoordSystem::GeodeticCoordType::E_WGS84System,
                              AtmosphereModel::AtmosphereModelType::E_NRLMSISE00Atmosphere, 2.2, 20, 150,150,ap,1.0, 20, false, true);

        preConfig2.Initializer(Mjd_UTC0,E_Earth,false,GravityModel::GravModelType::E_EGM08Model,20,20);
        preConfig_twoBody.Initializer(Mjd_UTC0,E_Earth,false);

        TwoBodyOrbitPredict twoBodyPre;

        Vector3d pos = cart0.Pos(), vel = cart0.Vel();

        /*
        cout << "Time (ModJDate)      x (km)         y (km)        z (km)   vx (km/sec)   vy (km/sec)   vz (km/sec)" <<endl;
        cout << "---------------    ---------       -------        -------   -----------   -----------  -----------" <<endl;
        cout.precision(12);
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        for (int i = 0; i < DayToSec/step; ++i)
        {
            preConfig_twoBody.SetMJD_UTC(Mjd_UTC);
            twoBodyPre.OrbitStep(preConfig_twoBody,step,RungeKutta::E_RungeKutta4, mass0, pos, vel);
            Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
            //cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
            //       vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;
        }
        cout << Mjd_UTC << "       " <<pos(0)/1000 << "       " <<pos(1)/1000 << "       " <<pos(2)/1000 << "       " <<
                vel(0)/1000<< "       " <<vel(1)/1000<< "       " <<vel(2)/1000<< "       " <<endl;

        */
        OrbitPredict orbit;

        cout << "Time (ModJDate)      x (km)         y (km)        z (km)   vx (km/sec)   vy (km/sec)   vz (km/sec)" <<endl;
        cout << "---------------    ---------       -------        -------   -----------   -----------  -----------" <<endl;
        cout.precision(15);
        fileStream.precision(15);
        fileLLA.precision(15);
        fileStream<< Mjd_UTC << "  " <<pos(0)/1000 <<  "  "  <<pos(1)/1000 <<  "  "  <<pos(2)/1000 <<  "  "  <<
                    vel(0)/1000<< "  "  <<vel(1)/1000<<  "  "  <<vel(2)/1000<<  "  "  <<endl;
        cout << Mjd_UTC << "  " <<pos(0)/1000 <<  "  "  <<pos(1)/1000 <<  "  "  <<pos(2)/1000 <<  "  " <<
                vel(0)/1000<<  "  "  <<vel(1)/1000<<  "  "  <<vel(2)/1000<<  "  "  <<endl;
        LLA = GEO.GetGeodeticCoord(pos,Mjd_UTC);
        fileLLA << Mjd_UTC <<  "  "  <<LLA.Altitude()/1000<<  "  "  <<LLA.Latitude()*RadToDeg<<  "  "  <<LLA.Longitude()*RadToDeg
             <<  "  "  <<pos(0)/1000 <<  "  "  <<pos(1)/1000 <<  "  "  <<pos(2)/1000 <<endl;
        for (int i = 0; i < 31*DayToSec/step; ++i)
        {
            preConfig1.SetMJD_UTC(Mjd_UTC);
            orbit.OrbitStep(preConfig1, step, RungeKutta::E_RungeKutta4, mass0, pos, vel);
            Mjd_UTC = Mjd_UTC0 + (i+1) * step/DayToSec;
            LLA = GEO.GetGeodeticCoord(pos,Mjd_UTC);
            fileLLA << Mjd_UTC <<  "  "  <<LLA.Altitude()/1000<<  "  "  <<LLA.Latitude()*RadToDeg<<  "  "  <<LLA.Longitude()*RadToDeg
                 <<  "  "  <<pos(0)/1000 <<  "  "  <<pos(1)/1000 <<  "  "  <<pos(2)/1000 <<endl;
            fileStream<< Mjd_UTC <<  "  "  <<pos(0)/1000 <<  "  "  <<pos(1)/1000 << "  "  <<pos(2)/1000 <<  "  " <<
                        vel(0)/1000<<  "  "  <<vel(1)/1000<<  "  "  <<vel(2)/1000<<  "  "  <<endl;
        }

        cout << Mjd_UTC <<  "  "  <<pos(0)/1000 <<  "  "  <<pos(1)/1000 <<  "  "  <<pos(2)/1000 << "  "  <<
                vel(0)/1000<<  "  "  <<vel(1)/1000<<  "  "  <<vel(2)/1000<<  "  "  <<endl;
        fileStream.close();
        fileLLA.close();

    }
};

int main(int argc, char *argv[])
{
    cout<<"SpaceDSL Test Run!"<<endl;
    try
    {
        MyTread t;
        t.Start();
        t.SetPriority(SpThread::Priority::HighestPriority);
        cout<<"Before Suspend!"<<endl;
        t.Suspend();
        cout<<"After Suspend!"<<endl;
        t.Resume();
        t.Wait();
    }
    catch (SPException &e)
    {
        e.what();
    }
    return 0;
}
